#include "ntm_scraper/parser.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
#include <sstream>

namespace ntm_scraper {

// ─── Helper: uppercase a string ─────────────────────────────────────────────

static std::string to_upper(const std::string& s) {
    std::string result = s;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
}

// ─── Helper: trim whitespace ────────────────────────────────────────────────

static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// ─── Month name to number ───────────────────────────────────────────────────

static int month_from_name(const std::string& name) {
    std::string upper = to_upper(name);
    if (upper == "JAN") return 1;
    if (upper == "FEB") return 2;
    if (upper == "MAR") return 3;
    if (upper == "APR") return 4;
    if (upper == "MAY") return 5;
    if (upper == "JUN") return 6;
    if (upper == "JUL") return 7;
    if (upper == "AUG") return 8;
    if (upper == "SEP") return 9;
    if (upper == "OCT") return 10;
    if (upper == "NOV") return 11;
    if (upper == "DEC") return 12;
    return 0;
}

// ─── Parse a single latitude component like "28-25.0N" or "28-25-30N" ──────

double NtmParser::parse_lat(const std::string& s) {
    std::string trimmed = trim(s);
    if (trimmed.empty()) return 0.0;

    // Format: DD-MM.MN or DD-MM-SS.SN
    std::regex dm_re(R"((\d{1,3})-([\d.]+)([NS]))");
    std::regex dms_re(R"((\d{1,3})-(\d{1,2})-([\d.]+)([NS]))");
    std::smatch m;

    if (std::regex_match(trimmed, m, dms_re)) {
        double deg = std::stod(m[1]);
        double min = std::stod(m[2]);
        double sec = std::stod(m[3]);
        double val = deg + min / 60.0 + sec / 3600.0;
        if (m[4] == "S") val = -val;
        return val;
    }

    if (std::regex_match(trimmed, m, dm_re)) {
        double deg = std::stod(m[1]);
        double min = std::stod(m[2]);
        double val = deg + min / 60.0;
        if (m[3] == "S") val = -val;
        return val;
    }

    return 0.0;
}

// ─── Parse a single longitude component like "080-20.0W" or "080-20-30W" ──

double NtmParser::parse_lon(const std::string& s) {
    std::string trimmed = trim(s);
    if (trimmed.empty()) return 0.0;

    // Format: DDD-MM.MW or DDD-MM-SS.SW
    std::regex dm_re(R"((\d{1,3})-([\d.]+)([EW]))");
    std::regex dms_re(R"((\d{1,3})-(\d{1,2})-([\d.]+)([EW]))");
    std::smatch m;

    if (std::regex_match(trimmed, m, dms_re)) {
        double deg = std::stod(m[1]);
        double min = std::stod(m[2]);
        double sec = std::stod(m[3]);
        double val = deg + min / 60.0 + sec / 3600.0;
        if (m[4] == "W") val = -val;
        return val;
    }

    if (std::regex_match(trimmed, m, dm_re)) {
        double deg = std::stod(m[1]);
        double min = std::stod(m[2]);
        double val = deg + min / 60.0;
        if (m[3] == "W") val = -val;
        return val;
    }

    return 0.0;
}

// ─── Parse NGA datetime format "DDHHMMZ MON YYYY" ──────────────────────────

int64_t NtmParser::parse_nga_datetime(const std::string& s) {
    std::string trimmed = trim(s);
    if (trimmed.empty()) return 0;

    // Format: DDHHMMZ MON YYYY
    std::regex nga_re(R"((\d{2})(\d{2})(\d{2})Z\s+([A-Za-z]{3})\s+(\d{4}))");
    std::smatch m;
    if (std::regex_search(trimmed, m, nga_re)) {
        int day = std::stoi(m[1]);
        int hour = std::stoi(m[2]);
        int min = std::stoi(m[3]);
        int mon = month_from_name(m[4]);
        int year = std::stoi(m[5]);

        if (mon == 0) return 0;

        Timestamp ts;
        ts.year = year;
        ts.month = mon;
        ts.day = day;
        ts.hour = hour;
        ts.minute = min;
        return ts.to_unix();
    }

    return 0;
}

// ─── Parse broadcast area from header text ──────────────────────────────────

BroadcastArea NtmParser::parse_broadcast_area(const std::string& text) {
    std::string upper = to_upper(text);
    if (upper.find("HYDROLANT") != std::string::npos) return BroadcastArea::HYDROLANT;
    if (upper.find("HYDROPAC") != std::string::npos) return BroadcastArea::HYDROPAC;
    if (upper.find("NAVAREA IV") != std::string::npos) return BroadcastArea::NAVAREA_IV;
    if (upper.find("NAVAREA XII") != std::string::npos) return BroadcastArea::NAVAREA_XII;
    return BroadcastArea::UNKNOWN;
}

// ─── Extract notice ID from first line ──────────────────────────────────────

std::string NtmParser::parse_notice_id(const std::string& text) {
    std::regex id_re(R"((HYDROLANT|HYDROPAC)\s+(\d+/\d+))", std::regex::icase);
    std::smatch m;
    if (std::regex_search(text, m, id_re)) {
        return to_upper(m[1].str()) + " " + m[2].str();
    }
    return "";
}

// ─── Classify hazard from text ──────────────────────────────────────────────

HazardClassification NtmParser::classify(const std::string& text) const {
    std::string upper = to_upper(text);

    // Check most specific first
    if (upper.find("BOOSTER RECOVERY") != std::string::npos ||
        upper.find("BOOSTER LANDING") != std::string::npos) {
        return HazardClassification::BOOSTER_RECOVERY;
    }

    if (upper.find("FAIRING RECOVERY") != std::string::npos ||
        upper.find("FAIRING") != std::string::npos) {
        return HazardClassification::FAIRING_RECOVERY;
    }

    if (upper.find("SPLASHDOWN") != std::string::npos) {
        return HazardClassification::SPLASHDOWN;
    }

    if (upper.find("ROCKET DEBRIS") != std::string::npos ||
        upper.find("FALLING DEBRIS") != std::string::npos) {
        return HazardClassification::ROCKET_DEBRIS;
    }

    if (upper.find("SPACE OPERATIONS") != std::string::npos ||
        upper.find("HAZARDOUS OPERATIONS") != std::string::npos) {
        return HazardClassification::GENERAL_HAZARD;
    }

    return HazardClassification::UNKNOWN;
}

// ─── Extract geometry from text ─────────────────────────────────────────────

ExclusionZone NtmParser::extract_geometry(const std::string& text) const {
    ExclusionZone zone;
    std::string upper = to_upper(text);

    // Check for circle: "WITHIN XXnm RADIUS OF DD-MM.MN DDD-MM.MW"
    std::regex circle_re(
        R"((\d+)\s*NM\s+RADIUS\s+OF\s+(\d{1,3}-[\d.]+[NS])\s+(\d{1,3}-[\d.]+[EW]))",
        std::regex::icase);
    std::smatch cm;
    if (std::regex_search(text, cm, circle_re)) {
        zone.type = GeometryType::CIRCLE;
        zone.circle.radius_nm = std::stod(cm[1]);
        zone.circle.center.lat = parse_lat(cm[2]);
        zone.circle.center.lon = parse_lon(cm[3]);
        zone.compute_bounds();
        return zone;
    }

    // Check for polygon: coordinate pairs on separate lines
    // Match DD-MM.MN DDD-MM.MW pattern (decimal minutes with dash)
    std::regex coord_pair_re(
        R"((\d{1,3}-[\d.]+[NS])\s+(\d{1,3}-[\d.]+[EW]))");

    auto begin = std::sregex_iterator(text.begin(), text.end(), coord_pair_re);
    auto end = std::sregex_iterator();

    for (auto it = begin; it != end; ++it) {
        std::smatch m = *it;
        LatLon pt;
        pt.lat = parse_lat(m[1]);
        pt.lon = parse_lon(m[2]);
        zone.polygon.vertices.push_back(pt);
    }

    if (!zone.polygon.vertices.empty()) {
        zone.type = GeometryType::POLYGON;
        zone.compute_bounds();
        return zone;
    }

    return zone;
}

// ─── Parse a single NGA broadcast warning ───────────────────────────────────

std::optional<MaritimeNotice> NtmParser::parse(const std::string& text) const {
    if (text.empty()) return std::nullopt;

    MaritimeNotice notice;
    notice.raw_text = text;

    // Extract notice ID
    notice.notice_id = parse_notice_id(text);
    if (notice.notice_id.empty()) return std::nullopt;

    // Extract broadcast area
    notice.area = parse_broadcast_area(text);

    // Extract dates: look for "DDHHMMZ MON YYYY TO DDHHMMZ MON YYYY"
    std::regex date_range_re(
        R"((\d{6}Z\s+[A-Za-z]{3}\s+\d{4})\s+TO\s+(\d{6}Z\s+[A-Za-z]{3}\s+\d{4}))",
        std::regex::icase);
    std::smatch dm;
    if (std::regex_search(text, dm, date_range_re)) {
        notice.effective_start = parse_nga_datetime(dm[1]);
        notice.effective_end = parse_nga_datetime(dm[2]);
    }

    // Extract geometry
    notice.zone = extract_geometry(text);

    // Classify hazard
    notice.classification = classify(text);

    // Compute confidence based on how much info we extracted
    double conf = 0.0;
    if (!notice.notice_id.empty()) conf += 0.2;
    if (notice.effective_start > 0 && notice.effective_end > 0) conf += 0.2;
    if (notice.zone.type != GeometryType::UNKNOWN) conf += 0.3;
    if (notice.classification != HazardClassification::UNKNOWN) conf += 0.2;
    if (notice.area != BroadcastArea::UNKNOWN) conf += 0.1;
    notice.confidence = conf;

    // Extract authority line
    std::regex auth_re(R"(AUTHORITY[:\s]+(.+))", std::regex::icase);
    std::smatch am;
    if (std::regex_search(text, am, auth_re)) {
        notice.authority = trim(am[1].str());
    }

    // Check for cancellation reference to a previous notice
    std::regex cancel_ref_re(
        R"(CANCEL\s+(HYDROLANT|HYDROPAC)\s+(\d+/\d+))", std::regex::icase);
    std::smatch crm;
    if (std::regex_search(text, crm, cancel_ref_re)) {
        notice.cancel_notice_id = to_upper(crm[1].str()) + " " + crm[2].str();
    }

    return notice;
}

// ─── Parse batch of notices ─────────────────────────────────────────────────

std::vector<MaritimeNotice> NtmParser::parse_batch(const std::string& text) const {
    std::vector<MaritimeNotice> results;

    // Split on HYDROLANT/HYDROPAC headers
    std::regex split_re(R"((?=HYDROLANT|HYDROPAC))", std::regex::icase);
    std::sregex_token_iterator it(text.begin(), text.end(), split_re, -1);
    std::sregex_token_iterator end;

    for (; it != end; ++it) {
        std::string chunk = trim(it->str());
        if (chunk.empty()) continue;

        auto notice = parse(chunk);
        if (notice.has_value()) {
            results.push_back(notice.value());
        }
    }

    return results;
}

} // namespace ntm_scraper
