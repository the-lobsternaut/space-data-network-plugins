#include "da_pipeline/ntm_parser.h"
#include <algorithm>
#include <cctype>
#include <regex>
#include <sstream>

namespace da_pipeline {

static std::string to_upper(const std::string& s) {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(), ::toupper);
    return r;
}

// ---------------------------------------------------------------------------
// parse_lat — "28-25.0N" format
// ---------------------------------------------------------------------------
double NtmParser::parse_lat(const std::string& s) {
    // Format: DD-MM.MN or DD-MM.MMN
    std::regex re(R"((\d{1,3})-(\d{1,2}(?:\.\d+)?)\s*([NS]))");
    std::smatch m;
    if (!std::regex_search(s, m, re)) return 0;

    double deg = std::stod(m[1].str());
    double min = std::stod(m[2].str());
    double val = deg + min / 60.0;
    if (m[3].str() == "S") val = -val;
    return val;
}

// ---------------------------------------------------------------------------
// parse_lon — "080-20.0W" format
// ---------------------------------------------------------------------------
double NtmParser::parse_lon(const std::string& s) {
    std::regex re(R"((\d{1,3})-(\d{1,2}(?:\.\d+)?)\s*([EW]))");
    std::smatch m;
    if (!std::regex_search(s, m, re)) return 0;

    double deg = std::stod(m[1].str());
    double min = std::stod(m[2].str());
    double val = deg + min / 60.0;
    if (m[3].str() == "W") val = -val;
    return val;
}

// ---------------------------------------------------------------------------
// parse_nga_datetime — "DDHHMMZ MON YYYY" format
// ---------------------------------------------------------------------------
Timestamp NtmParser::parse_nga_datetime(const std::string& s) {
    // e.g. "151200Z MAR 2024"
    std::regex re(R"((\d{2})(\d{2})(\d{2})Z\s+(\w{3})\s+(\d{4}))");
    std::smatch m;
    if (!std::regex_search(s, m, re)) return 0;

    int dd = std::stoi(m[1].str());
    int hh = std::stoi(m[2].str());
    int mn = std::stoi(m[3].str());
    std::string mon = to_upper(m[4].str());
    int year = std::stoi(m[5].str());

    static const char* months[] = {
        "JAN","FEB","MAR","APR","MAY","JUN",
        "JUL","AUG","SEP","OCT","NOV","DEC"
    };
    int mm = 0;
    for (int i = 0; i < 12; i++) {
        if (mon == months[i]) { mm = i + 1; break; }
    }
    if (mm == 0) return 0;

    auto days_in_year = [](int y) -> int {
        return (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    };
    int month_days[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (days_in_year(year) == 366) month_days[2] = 29;

    int64_t days = 0;
    for (int y = 1970; y < year; y++) days += days_in_year(y);
    for (int i = 1; i < mm; i++) days += month_days[i];
    days += dd - 1;

    return days * 86400 + hh * 3600 + mn * 60;
}

// ---------------------------------------------------------------------------
// classify
// ---------------------------------------------------------------------------
HazardClassification NtmParser::classify(const std::string& text) const {
    std::string upper = to_upper(text);
    if (upper.find("ROCKET DEBRIS") != std::string::npos ||
        upper.find("FALLING ROCKET") != std::string::npos)
        return HazardClassification::ROCKET_DEBRIS;
    if (upper.find("SPLASHDOWN") != std::string::npos ||
        upper.find("LANDING ZONE") != std::string::npos)
        return HazardClassification::SPLASHDOWN;
    if (upper.find("BOOSTER RECOVERY") != std::string::npos ||
        upper.find("BOOSTER LANDING") != std::string::npos)
        return HazardClassification::BOOSTER_RECOVERY;
    if (upper.find("FAIRING RECOVERY") != std::string::npos ||
        upper.find("FAIRING") != std::string::npos)
        return HazardClassification::FAIRING_RECOVERY;
    if (upper.find("HAZARD") != std::string::npos ||
        upper.find("DANGER") != std::string::npos ||
        upper.find("ROCKET") != std::string::npos ||
        upper.find("SPACE") != std::string::npos ||
        upper.find("LAUNCH") != std::string::npos)
        return HazardClassification::GENERAL_HAZARD;
    return HazardClassification::UNKNOWN;
}

// ---------------------------------------------------------------------------
// extract_geometry
// ---------------------------------------------------------------------------
ExclusionZone NtmParser::extract_geometry(const std::string& text) const {
    ExclusionZone zone;

    // Try circle: "WITHIN NN NM OF DD-MM.MN DDD-MM.MW"
    {
        std::regex circ_re(R"(WITHIN\s+(\d+\.?\d*)\s*(?:NM|NAUTICAL MILES?)\s+(?:OF|RADIUS)\s+(\d{1,3}-\d{1,2}(?:\.\d+)?[NS])\s+(\d{1,3}-\d{1,2}(?:\.\d+)?[EW]))", std::regex::icase);
        std::smatch m;
        if (std::regex_search(text, m, circ_re)) {
            zone.type = GeometryType::CIRCLE;
            zone.circle.radius_nm = std::stod(m[1].str());
            zone.circle.center.lat = parse_lat(m[2].str());
            zone.circle.center.lon = parse_lon(m[3].str());
            zone.compute_bounds();
            return zone;
        }
    }

    // Try polygon: multiple coordinate pairs
    {
        std::regex coord_re(R"((\d{1,3}-\d{1,2}(?:\.\d+)?[NS])\s+(\d{1,3}-\d{1,2}(?:\.\d+)?[EW]))");
        auto begin = std::sregex_iterator(text.begin(), text.end(), coord_re);
        auto end_it = std::sregex_iterator();

        std::vector<LatLon> vertices;
        for (auto it = begin; it != end_it; ++it) {
            LatLon v;
            v.lat = parse_lat((*it)[1].str());
            v.lon = parse_lon((*it)[2].str());
            vertices.push_back(v);
        }

        if (vertices.size() >= 3) {
            zone.type = GeometryType::POLYGON;
            zone.polygon.vertices = vertices;
            zone.compute_bounds();
            return zone;
        } else if (vertices.size() == 1) {
            // Single point → default circle
            zone.type = GeometryType::CIRCLE;
            zone.circle.center = vertices[0];
            zone.circle.radius_nm = 20.0;
            zone.compute_bounds();
            return zone;
        }
    }

    zone.type = GeometryType::UNKNOWN;
    return zone;
}

// ---------------------------------------------------------------------------
// parse
// ---------------------------------------------------------------------------
std::optional<MaritimeNotice> NtmParser::parse(const std::string& text) const {
    if (text.empty()) return std::nullopt;

    MaritimeNotice notice;
    notice.raw_text = text;
    std::string upper = to_upper(text);

    // Determine broadcast area
    if (upper.find("HYDROLANT") != std::string::npos)
        notice.area = BroadcastArea::HYDROLANT;
    else if (upper.find("HYDROPAC") != std::string::npos)
        notice.area = BroadcastArea::HYDROPAC;
    else if (upper.find("NAVAREA IV") != std::string::npos || upper.find("NAVAREA_IV") != std::string::npos)
        notice.area = BroadcastArea::NAVAREA_IV;
    else if (upper.find("NAVAREA XII") != std::string::npos || upper.find("NAVAREA_XII") != std::string::npos)
        notice.area = BroadcastArea::NAVAREA_XII;

    // Extract notice ID: pattern like "NNNN/YYYY" or "HYDROLANT NNNN/YYYY"
    {
        std::regex id_re(R"((\d{3,5}/\d{2,4}))");
        std::smatch m;
        if (std::regex_search(text, m, id_re)) {
            notice.notice_id = m[1].str();
        }
    }

    // Authority
    {
        std::regex auth_re(R"((?:SOURCE|AUTHORITY)[:\s]+(.+?)(?:\n|$))", std::regex::icase);
        std::smatch m;
        if (std::regex_search(text, m, auth_re)) {
            notice.authority = m[1].str();
        }
    }

    // Cancellation
    if (upper.find("CANCEL") != std::string::npos) {
        notice.is_cancelled = true;
        std::regex cancel_re(R"(CANCEL[A-Z]*\s+(\d{3,5}/\d{2,4}))");
        std::smatch m;
        if (std::regex_search(upper, m, cancel_re)) {
            notice.cancel_notice_id = m[1].str();
        }
    }

    // Datetime: "DDHHMMZ MON YYYY TO DDHHMMZ MON YYYY"
    {
        std::regex dt_re(R"((\d{6}Z\s+\w{3}\s+\d{4})\s+(?:TO|THRU|-)\s+(\d{6}Z\s+\w{3}\s+\d{4}))");
        std::smatch m;
        if (std::regex_search(text, m, dt_re)) {
            notice.effective_start = parse_nga_datetime(m[1].str());
            notice.effective_end = parse_nga_datetime(m[2].str());
        }
    }

    // Classification
    notice.classification = classify(text);

    // Geometry
    notice.zone = extract_geometry(text);
    notice.zone.id = "NTM-" + notice.notice_id;
    notice.zone.effective_start = notice.effective_start;
    notice.zone.effective_end = notice.effective_end;

    // Confidence based on how much info we extracted
    double conf = 0;
    if (!notice.notice_id.empty()) conf += 0.2;
    if (notice.zone.type != GeometryType::UNKNOWN) conf += 0.3;
    if (notice.effective_start != 0) conf += 0.2;
    if (notice.classification != HazardClassification::UNKNOWN) conf += 0.3;
    notice.confidence = conf;

    if (notice.notice_id.empty() && notice.zone.type == GeometryType::UNKNOWN) {
        return std::nullopt;
    }

    return notice;
}

// ---------------------------------------------------------------------------
// parse_batch
// ---------------------------------------------------------------------------
std::vector<MaritimeNotice> NtmParser::parse_batch(const std::string& text) const {
    std::vector<MaritimeNotice> results;

    // Split on "NNNN" or double blank lines
    std::istringstream iss(text);
    std::string line, block;
    while (std::getline(iss, line)) {
        std::string trimmed = line;
        while (!trimmed.empty() && std::isspace(trimmed.back())) trimmed.pop_back();

        if (trimmed == "NNNN" || (line.empty() && !block.empty() && block.back() == '\n')) {
            if (!block.empty()) {
                auto n = parse(block);
                if (n) results.push_back(*n);
                block.clear();
            }
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
