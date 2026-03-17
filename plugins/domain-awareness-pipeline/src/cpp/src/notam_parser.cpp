#include "da_pipeline/notam_parser.h"
#include <algorithm>
#include <cctype>
#include <regex>
#include <sstream>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static std::string to_upper(const std::string& s) {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(), ::toupper);
    return r;
}

double NotamParser::parse_dms_lat(const std::string& s) {
    // Format: DDMMSSN or DDMMSN or DDMMN — we handle DDMMSSN/DDMMSSS
    if (s.size() < 4) return 0;
    char hemi = std::toupper(s.back());
    std::string digits = s.substr(0, s.size() - 1);

    double deg = 0, min = 0, sec = 0;
    if (digits.size() >= 6) {
        deg = std::stod(digits.substr(0, 2));
        min = std::stod(digits.substr(2, 2));
        sec = std::stod(digits.substr(4));
    } else if (digits.size() >= 4) {
        deg = std::stod(digits.substr(0, 2));
        min = std::stod(digits.substr(2));
    } else {
        deg = std::stod(digits);
    }

    double val = deg + min / 60.0 + sec / 3600.0;
    if (hemi == 'S') val = -val;
    return val;
}

double NotamParser::parse_dms_lon(const std::string& s) {
    // Format: DDDMMSSW or DDDMMSW
    if (s.size() < 5) return 0;
    char hemi = std::toupper(s.back());
    std::string digits = s.substr(0, s.size() - 1);

    double deg = 0, min = 0, sec = 0;
    if (digits.size() >= 7) {
        deg = std::stod(digits.substr(0, 3));
        min = std::stod(digits.substr(3, 2));
        sec = std::stod(digits.substr(5));
    } else if (digits.size() >= 5) {
        deg = std::stod(digits.substr(0, 3));
        min = std::stod(digits.substr(3));
    } else {
        deg = std::stod(digits);
    }

    double val = deg + min / 60.0 + sec / 3600.0;
    if (hemi == 'W') val = -val;
    return val;
}

Timestamp NotamParser::parse_notam_datetime(const std::string& s) {
    // Format: YYMMDDHHMM
    if (s.size() < 10) return 0;
    int yy = std::stoi(s.substr(0, 2));
    int mm = std::stoi(s.substr(2, 2));
    int dd = std::stoi(s.substr(4, 2));
    int hh = std::stoi(s.substr(6, 2));
    int mn = std::stoi(s.substr(8, 2));

    int year = (yy >= 70) ? 1900 + yy : 2000 + yy;

    // Simple days-since-epoch calculation
    // Using a simplified approach: approximate Unix timestamp
    // Days from 1970-01-01 to year-01-01
    auto days_in_year = [](int y) -> int { return (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365; };
    int month_days[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (days_in_year(year) == 366) month_days[2] = 29;

    int64_t days = 0;
    for (int y = 1970; y < year; y++) days += days_in_year(y);
    for (int m = 1; m < mm; m++) days += month_days[m];
    days += dd - 1;

    return days * 86400 + hh * 3600 + mn * 60;
}

bool NotamParser::contains_any(const std::string& text, const std::vector<std::string>& keywords) {
    std::string upper = to_upper(text);
    for (auto& kw : keywords) {
        if (upper.find(to_upper(kw)) != std::string::npos) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// parse — FAA NOTAM format
// ---------------------------------------------------------------------------
std::optional<NOTAM> NotamParser::parse(const std::string& text) const {
    if (text.empty()) return std::nullopt;

    NOTAM n;
    n.notam_text = text;
    std::string upper = to_upper(text);

    // Extract NOTAM ID: pattern like "FDC N/NNNN" or "!FDC N/NNNN" or "A1234/24"
    {
        std::regex id_re(R"((!?FDC\s+)?(\d+/\d+|\w+\s+\d+/\d+))");
        std::smatch m;
        if (std::regex_search(text, m, id_re)) {
            n.notam_id = m[2].str();
        }
    }

    // Extract location ID (4-letter ICAO identifier after ! or at start)
    {
        std::regex loc_re(R"(!([A-Z]{3,4})\s)");
        std::smatch m;
        if (std::regex_search(text, m, loc_re)) {
            n.location_id = m[1].str();
        }
    }

    // Extract coordinates: DDMMSSN/DDDMMSSW pattern
    {
        std::regex coord_re(R"((\d{4,7}[NS])\s*/?\s*(\d{5,8}[EW]))");
        std::smatch m;
        if (std::regex_search(text, m, coord_re)) {
            n.center_lat = parse_dms_lat(m[1].str());
            n.center_lon = parse_dms_lon(m[2].str());
        }
    }

    // Extract radius: "NM RADIUS" or "RNNN"
    {
        std::regex rad_re(R"((\d+\.?\d*)\s*NM\s*(RADIUS|ARC))");
        std::smatch m;
        if (std::regex_search(upper, m, rad_re)) {
            n.radius_nm = std::stod(m[1].str());
        }
    }

    // Extract altitude: SFC or NNN FT to NNN FT / FL NNN
    {
        std::regex alt_re(R"((?:SFC|(\d+))\s*(?:FT)?\s*(?:TO|[-/])\s*(?:FL\s*(\d+)|(\d+)\s*FT|UNL))");
        std::smatch m;
        if (std::regex_search(upper, m, alt_re)) {
            if (m[1].matched) n.altitude_floor_ft = std::stod(m[1].str());
            if (m[2].matched) n.altitude_ceiling_ft = std::stod(m[2].str()) * 100;
            if (m[3].matched) n.altitude_ceiling_ft = std::stod(m[3].str());
            if (upper.find("UNL") != std::string::npos) n.altitude_ceiling_ft = 999999;
        }
    }

    // Extract time window: YYMMDDHHMM-YYMMDDHHMM
    {
        std::regex time_re(R"((\d{10})\s*[-/]\s*(\d{10}))");
        std::smatch m;
        if (std::regex_search(text, m, time_re)) {
            n.effective_start = parse_notam_datetime(m[1].str());
            n.effective_end = parse_notam_datetime(m[2].str());
        }
    }

    // Extract polygon vertices if present
    {
        std::regex poly_re(R"((\d{4,7}[NS])\s*/?\s*(\d{5,8}[EW]))");
        auto begin = std::sregex_iterator(text.begin(), text.end(), poly_re);
        auto end_it = std::sregex_iterator();
        int count = 0;
        for (auto it = begin; it != end_it; ++it) count++;

        if (count >= 3) {
            // Multiple coordinates → polygon
            for (auto it = std::sregex_iterator(text.begin(), text.end(), poly_re);
                 it != end_it; ++it) {
                LatLon v;
                v.lat = parse_dms_lat((*it)[1].str());
                v.lon = parse_dms_lon((*it)[2].str());
                n.polygon.vertices.push_back(v);
            }
        }
    }

    // Classify
    n.classification = classify(n);

    // Must have at least an ID or coordinates to be valid
    if (n.notam_id.empty() && n.center_lat == 0 && n.center_lon == 0 && n.polygon.vertices.empty()) {
        return std::nullopt;
    }

    return n;
}

// ---------------------------------------------------------------------------
// parse_icao — ICAO NOTAM format (Q-line based)
// ---------------------------------------------------------------------------
std::optional<NOTAM> NotamParser::parse_icao(const std::string& text) const {
    if (text.empty()) return std::nullopt;

    NOTAM n;
    n.notam_text = text;

    // ICAO ID: A1234/24 or B0001/24
    {
        std::regex id_re(R"(([A-Z]\d{4}/\d{2,4}))");
        std::smatch m;
        if (std::regex_search(text, m, id_re)) {
            n.notam_id = m[1].str();
        }
    }

    // Q-line: Q) FIR/CODE/TRAFFIC/PURPOSE/SCOPE/LOWER/UPPER/COORD
    {
        std::regex q_re(R"(Q\)\s*(\w+)/(\w+)/(\w+)/(\w+)/(\w+)/(\d{3})/(\d{3})/(\d{4}[NS]\d{5}[EW])(\d{3}))");
        std::smatch m;
        if (std::regex_search(text, m, q_re)) {
            n.location_id = m[1].str();
            n.altitude_floor_ft = std::stod(m[6].str()) * 100;
            n.altitude_ceiling_ft = std::stod(m[7].str()) * 100;

            std::string coord = m[8].str();
            // DDMMN DDDMME
            if (coord.size() >= 9) {
                std::string lat_s = coord.substr(0, 5);  // DDMMN
                std::string lon_s = coord.substr(5, 6);   // DDDMME
                // Simple DDMM parse
                double lat_d = std::stod(lat_s.substr(0, 2));
                double lat_m = std::stod(lat_s.substr(2, 2));
                n.center_lat = lat_d + lat_m / 60.0;
                if (lat_s.back() == 'S') n.center_lat = -n.center_lat;

                // Check if we have enough characters
                if (lon_s.size() >= 5) {
                    double lon_d = std::stod(lon_s.substr(0, 3));
                    double lon_m = std::stod(lon_s.substr(3, 2));
                    n.center_lon = lon_d + lon_m / 60.0;
                    if (lon_s.back() == 'W') n.center_lon = -n.center_lon;
                }
            }
            n.radius_nm = std::stod(m[9].str());
        }
    }

    // B) and C) lines for time
    {
        std::regex b_re(R"(B\)\s*(\d{10}))");
        std::smatch m;
        if (std::regex_search(text, m, b_re)) {
            n.effective_start = parse_notam_datetime(m[1].str());
        }
    }
    {
        std::regex c_re(R"(C\)\s*(\d{10}|PERM))");
        std::smatch m;
        if (std::regex_search(text, m, c_re)) {
            std::string val = m[1].str();
            if (val != "PERM") {
                n.effective_end = parse_notam_datetime(val);
            } else {
                n.effective_end = 0x7FFFFFFF; // far future
            }
        }
    }

    // Also try to extract DMS coordinates from E) line
    {
        std::regex coord_re(R"((\d{4,7}[NS])\s*/?\s*(\d{5,8}[EW]))");
        std::smatch m;
        if (std::regex_search(text, m, coord_re)) {
            if (n.center_lat == 0 && n.center_lon == 0) {
                n.center_lat = parse_dms_lat(m[1].str());
                n.center_lon = parse_dms_lon(m[2].str());
            }
        }
    }

    n.classification = classify(n);

    if (n.notam_id.empty()) return std::nullopt;
    return n;
}

// ---------------------------------------------------------------------------
// classify
// ---------------------------------------------------------------------------
NotamClassification NotamParser::classify(const NOTAM& n) const {
    static const std::vector<std::string> launch_kw = {
        "SPACE OPERATIONS", "SPACE LAUNCH", "ROCKET LAUNCH",
        "14 CFR 91.143", "SPACEX", "ULA", "SPACE VEHICLE",
        "LAUNCH OPERATIONS", "ROCKET OPERATIONS", "SPACE FLIGHT"
    };
    static const std::vector<std::string> reentry_kw = {
        "REENTRY", "RE-ENTRY", "SPLASHDOWN", "CAPSULE RECOVERY"
    };
    static const std::vector<std::string> hazard_kw = {
        "ROCKET DEBRIS", "BOOSTER RECOVERY", "FAIRING RECOVERY",
        "FALLING DEBRIS", "HAZARDOUS OPERATIONS"
    };
    static const std::vector<std::string> airspace_kw = {
        "TFR", "TEMPORARY FLIGHT RESTRICTION", "FLIGHT RESTRICTION"
    };

    if (contains_any(n.notam_text, launch_kw)) return NotamClassification::LAUNCH;
    if (contains_any(n.notam_text, reentry_kw)) return NotamClassification::REENTRY;
    if (contains_any(n.notam_text, hazard_kw)) return NotamClassification::HAZARD;
    if (contains_any(n.notam_text, airspace_kw)) return NotamClassification::AIRSPACE;
    return NotamClassification::ROUTINE;
}

bool NotamParser::is_launch_related(const NOTAM& n) const {
    auto c = n.classification;
    return c == NotamClassification::LAUNCH ||
           c == NotamClassification::REENTRY ||
           c == NotamClassification::HAZARD;
}

// ---------------------------------------------------------------------------
// notam_to_zone
// ---------------------------------------------------------------------------
ExclusionZone NotamParser::notam_to_zone(const NOTAM& n) const {
    ExclusionZone z;
    z.id = "NOTAM-" + n.notam_id;
    z.effective_start = n.effective_start;
    z.effective_end = n.effective_end;

    if (!n.polygon.vertices.empty() && n.polygon.vertices.size() >= 3) {
        z.type = GeometryType::POLYGON;
        z.polygon = n.polygon;
    } else if (n.radius_nm > 0) {
        z.type = GeometryType::CIRCLE;
        z.circle.center = {n.center_lat, n.center_lon};
        z.circle.radius_nm = n.radius_nm;
    } else if (n.center_lat != 0 || n.center_lon != 0) {
        // Point with no radius — default small radius
        z.type = GeometryType::CIRCLE;
        z.circle.center = {n.center_lat, n.center_lon};
        z.circle.radius_nm = 5.0;
    } else {
        z.type = GeometryType::UNKNOWN;
    }

    z.compute_bounds();
    return z;
}

// ---------------------------------------------------------------------------
// parse_batch
// ---------------------------------------------------------------------------
std::vector<NOTAM> NotamParser::parse_batch(const std::string& text) const {
    std::vector<NOTAM> results;
    // Split on double newlines or "!FDC" or "!TFR"
    std::istringstream iss(text);
    std::string line, block;
    while (std::getline(iss, line)) {
        if (line.empty() && !block.empty()) {
            auto n = parse(block);
            if (n) results.push_back(*n);
            block.clear();
        } else {
            if (!block.empty()) block += "\n";
            block += line;
        }
    }
    if (!block.empty()) {
        auto n = parse(block);
        if (n) results.push_back(*n);
    }
    return results;
}

} // namespace da_pipeline
