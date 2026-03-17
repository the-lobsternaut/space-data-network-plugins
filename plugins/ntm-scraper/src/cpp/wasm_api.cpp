#include "ntm_scraper/parser.h"
#include "ntm_scraper/archive.h"
#include "ntm_scraper/types.h"

#include <emscripten/emscripten.h>
#include <emscripten/bind.h>

#include <string>
#include <sstream>

using namespace ntm_scraper;

// ─── Global instances ───────────────────────────────────────────────────────

static NtmParser      g_parser;
static NoticeArchive  g_archive;

// ─── Helper: classification to string ───────────────────────────────────────

static const char* classification_str(HazardClassification c) {
    switch (c) {
        case HazardClassification::ROCKET_DEBRIS:    return "rocket_debris";
        case HazardClassification::SPLASHDOWN:       return "splashdown";
        case HazardClassification::BOOSTER_RECOVERY: return "booster_recovery";
        case HazardClassification::FAIRING_RECOVERY: return "fairing_recovery";
        case HazardClassification::GENERAL_HAZARD:   return "general_hazard";
        case HazardClassification::UNKNOWN:          return "unknown";
    }
    return "unknown";
}

// ─── Helper: broadcast area to string ───────────────────────────────────────

static const char* area_str(BroadcastArea a) {
    switch (a) {
        case BroadcastArea::HYDROLANT:   return "hydrolant";
        case BroadcastArea::HYDROPAC:    return "hydropac";
        case BroadcastArea::NAVAREA_IV:  return "navarea_iv";
        case BroadcastArea::NAVAREA_XII: return "navarea_xii";
        case BroadcastArea::UNKNOWN:     return "unknown";
    }
    return "unknown";
}

// ─── WASM API Functions ─────────────────────────────────────────────────────

std::string wasm_parse_notice(const std::string& text) {
    auto result = g_parser.parse(text);
    if (!result.has_value()) {
        return R"({"error": "Failed to parse notice"})";
    }

    auto& n = result.value();
    std::ostringstream json;
    json << "{";
    json << "\"notice_id\":\"" << n.notice_id << "\",";
    json << "\"area\":\"" << area_str(n.area) << "\",";
    json << "\"classification\":\"" << classification_str(n.classification) << "\",";
    json << "\"effective_start\":" << n.effective_start << ",";
    json << "\"effective_end\":" << n.effective_end << ",";
    json << "\"confidence\":" << n.confidence << ",";
    json << "\"authority\":\"" << n.authority << "\",";
    json << "\"geometry_type\":\"";
    switch (n.zone.type) {
        case GeometryType::CIRCLE:  json << "circle"; break;
        case GeometryType::POLYGON: json << "polygon"; break;
        default: json << "unknown"; break;
    }
    json << "\"";
    if (n.zone.type == GeometryType::CIRCLE) {
        json << ",\"center_lat\":" << n.zone.circle.center.lat;
        json << ",\"center_lon\":" << n.zone.circle.center.lon;
        json << ",\"radius_nm\":" << n.zone.circle.radius_nm;
    } else if (n.zone.type == GeometryType::POLYGON) {
        json << ",\"vertices\":[";
        for (size_t i = 0; i < n.zone.polygon.vertices.size(); ++i) {
            if (i > 0) json << ",";
            json << "[" << n.zone.polygon.vertices[i].lat << ","
                 << n.zone.polygon.vertices[i].lon << "]";
        }
        json << "]";
    }
    json << "}";
    return json.str();
}

std::string wasm_parse_batch(const std::string& text) {
    auto notices = g_parser.parse_batch(text);
    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < notices.size(); ++i) {
        if (i > 0) json << ",";
        json << "{\"notice_id\":\"" << notices[i].notice_id << "\",";
        json << "\"classification\":\"" << classification_str(notices[i].classification) << "\"}";
    }
    json << "]";
    return json.str();
}

std::string wasm_classify(const std::string& text) {
    auto cls = g_parser.classify(text);
    std::ostringstream json;
    json << "{\"classification\":\"" << classification_str(cls) << "\"}";
    return json.str();
}

std::string wasm_find_by_region(double min_lat, double max_lat,
                                 double min_lon, double max_lon) {
    auto results = g_archive.query_by_region(min_lat, max_lat, min_lon, max_lon);
    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < results.size(); ++i) {
        if (i > 0) json << ",";
        json << "{\"notice_id\":\"" << results[i].notice_id << "\",";
        json << "\"classification\":\"" << classification_str(results[i].classification) << "\"}";
    }
    json << "]";
    return json.str();
}

// ─── New API Functions ──────────────────────────────────────────────────────

std::string wasm_extract_geometry(const std::string& text) {
    auto zone = g_parser.extract_geometry(text);
    std::ostringstream json;
    json << "{\"geometry_type\":\"";
    switch (zone.type) {
        case GeometryType::CIRCLE:  json << "circle"; break;
        case GeometryType::POLYGON: json << "polygon"; break;
        default: json << "unknown"; break;
    }
    json << "\"";
    if (zone.type == GeometryType::CIRCLE) {
        json << ",\"coordinates\":{\"center_lat\":" << zone.circle.center.lat
             << ",\"center_lon\":" << zone.circle.center.lon
             << ",\"radius_nm\":" << zone.circle.radius_nm << "}";
    } else if (zone.type == GeometryType::POLYGON) {
        json << ",\"coordinates\":[";
        for (size_t i = 0; i < zone.polygon.vertices.size(); ++i) {
            if (i > 0) json << ",";
            json << "[" << zone.polygon.vertices[i].lat << ","
                 << zone.polygon.vertices[i].lon << "]";
        }
        json << "]";
    }
    json << "}";
    return json.str();
}

double wasm_parse_lat(const std::string& s) {
    return NtmParser::parse_lat(s);
}

double wasm_parse_lon(const std::string& s) {
    return NtmParser::parse_lon(s);
}

int64_t wasm_parse_nga_datetime(const std::string& s) {
    return NtmParser::parse_nga_datetime(s);
}

std::string wasm_parse_broadcast_area(const std::string& text) {
    auto a = NtmParser::parse_broadcast_area(text);
    std::ostringstream json;
    json << "{\"area\":\"" << area_str(a) << "\"}";
    return json.str();
}

std::string wasm_parse_notice_id(const std::string& text) {
    auto id = NtmParser::parse_notice_id(text);
    std::ostringstream json;
    json << "{\"notice_id\":\"" << id << "\"}";
    return json.str();
}

// Helper: extract a JSON string value by key using simple string parsing
static std::string json_extract_string(const std::string& json, const std::string& key) {
    std::string search = "\"" + key + "\"";
    auto pos = json.find(search);
    if (pos == std::string::npos) return "";
    pos = json.find(':', pos + search.size());
    if (pos == std::string::npos) return "";
    pos = json.find('"', pos + 1);
    if (pos == std::string::npos) return "";
    auto end = json.find('"', pos + 1);
    if (end == std::string::npos) return "";
    return json.substr(pos + 1, end - pos - 1);
}

// Helper: extract a JSON numeric value by key
static double json_extract_number(const std::string& json, const std::string& key) {
    std::string search = "\"" + key + "\"";
    auto pos = json.find(search);
    if (pos == std::string::npos) return 0.0;
    pos = json.find(':', pos + search.size());
    if (pos == std::string::npos) return 0.0;
    // Skip whitespace
    pos++;
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    std::string num_str;
    while (pos < json.size() && (std::isdigit(json[pos]) || json[pos] == '.' ||
           json[pos] == '-' || json[pos] == 'e' || json[pos] == 'E' || json[pos] == '+')) {
        num_str += json[pos++];
    }
    if (num_str.empty()) return 0.0;
    return std::stod(num_str);
}

static HazardClassification parse_classification_string(const std::string& s) {
    if (s == "rocket_debris")    return HazardClassification::ROCKET_DEBRIS;
    if (s == "splashdown")       return HazardClassification::SPLASHDOWN;
    if (s == "booster_recovery") return HazardClassification::BOOSTER_RECOVERY;
    if (s == "fairing_recovery") return HazardClassification::FAIRING_RECOVERY;
    if (s == "general_hazard")   return HazardClassification::GENERAL_HAZARD;
    return HazardClassification::UNKNOWN;
}

static MaritimeNotice notice_from_json(const std::string& json) {
    MaritimeNotice n;
    n.notice_id = json_extract_string(json, "notice_id");
    n.raw_text = json_extract_string(json, "raw_text");
    n.authority = json_extract_string(json, "authority");
    n.cancel_notice_id = json_extract_string(json, "cancel_notice_id");
    n.effective_start = static_cast<int64_t>(json_extract_number(json, "effective_start"));
    n.effective_end = static_cast<int64_t>(json_extract_number(json, "effective_end"));
    n.confidence = json_extract_number(json, "confidence");

    std::string cls_str = json_extract_string(json, "classification");
    n.classification = parse_classification_string(cls_str);

    std::string area_s = json_extract_string(json, "area");
    if (area_s == "hydrolant")    n.area = BroadcastArea::HYDROLANT;
    else if (area_s == "hydropac")    n.area = BroadcastArea::HYDROPAC;
    else if (area_s == "navarea_iv")  n.area = BroadcastArea::NAVAREA_IV;
    else if (area_s == "navarea_xii") n.area = BroadcastArea::NAVAREA_XII;

    std::string cancelled_str = json_extract_string(json, "is_cancelled");
    n.is_cancelled = (cancelled_str == "true");

    return n;
}

static std::string notice_to_brief_json(const MaritimeNotice& n) {
    std::ostringstream json;
    json << "{\"notice_id\":\"" << n.notice_id << "\",";
    json << "\"area\":\"" << area_str(n.area) << "\",";
    json << "\"classification\":\"" << classification_str(n.classification) << "\",";
    json << "\"effective_start\":" << n.effective_start << ",";
    json << "\"effective_end\":" << n.effective_end << ",";
    json << "\"confidence\":" << n.confidence << ",";
    json << "\"is_cancelled\":" << (n.is_cancelled ? "true" : "false") << "}";
    return json.str();
}

std::string wasm_archive_add(const std::string& notice_json) {
    MaritimeNotice n = notice_from_json(notice_json);
    g_archive.add(n);
    std::ostringstream json;
    json << "{\"status\":\"ok\",\"notice_id\":\"" << n.notice_id << "\"}";
    return json.str();
}

std::string wasm_archive_add_batch(const std::string& text) {
    auto notices = g_parser.parse_batch(text);
    g_archive.add_batch(notices);
    std::ostringstream json;
    json << "{\"count\":" << notices.size() << "}";
    return json.str();
}

std::string wasm_archive_query_by_time(int64_t start, int64_t end) {
    auto results = g_archive.query_by_time(start, end);
    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < results.size(); ++i) {
        if (i > 0) json << ",";
        json << notice_to_brief_json(results[i]);
    }
    json << "]";
    return json.str();
}

std::string wasm_archive_query_by_classification(const std::string& cls_str) {
    HazardClassification cls = parse_classification_string(cls_str);
    auto results = g_archive.query_by_classification(cls);
    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < results.size(); ++i) {
        if (i > 0) json << ",";
        json << notice_to_brief_json(results[i]);
    }
    json << "]";
    return json.str();
}

std::string wasm_archive_get_active() {
    auto results = g_archive.get_active_notices();
    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < results.size(); ++i) {
        if (i > 0) json << ",";
        json << notice_to_brief_json(results[i]);
    }
    json << "]";
    return json.str();
}

std::string wasm_archive_process_cancellations() {
    size_t before = g_archive.size();
    g_archive.process_cancellations();
    size_t cancelled = 0;
    auto all = g_archive.get_all();
    for (auto& n : all) {
        if (n.is_cancelled) cancelled++;
    }
    std::ostringstream json;
    json << "{\"count\":" << cancelled << "}";
    return json.str();
}

int wasm_archive_size() {
    return static_cast<int>(g_archive.size());
}

void wasm_archive_clear() {
    g_archive.clear();
}

// ─── Emscripten Bindings ────────────────────────────────────────────────────

EMSCRIPTEN_BINDINGS(ntm_scraper_wasm) {
    emscripten::function("parseNotice",                  &wasm_parse_notice);
    emscripten::function("parseBatch",                   &wasm_parse_batch);
    emscripten::function("classify",                     &wasm_classify);
    emscripten::function("findByRegion",                 &wasm_find_by_region);
    emscripten::function("extractGeometry",              &wasm_extract_geometry);
    emscripten::function("parseLat",                     &wasm_parse_lat);
    emscripten::function("parseLon",                     &wasm_parse_lon);
    emscripten::function("parseNgaDatetime",             &wasm_parse_nga_datetime);
    emscripten::function("parseBroadcastArea",           &wasm_parse_broadcast_area);
    emscripten::function("parseNoticeId",                &wasm_parse_notice_id);
    emscripten::function("archiveAdd",                   &wasm_archive_add);
    emscripten::function("archiveAddBatch",              &wasm_archive_add_batch);
    emscripten::function("archiveQueryByTime",           &wasm_archive_query_by_time);
    emscripten::function("archiveQueryByClassification", &wasm_archive_query_by_classification);
    emscripten::function("archiveGetActive",             &wasm_archive_get_active);
    emscripten::function("archiveProcessCancellations",  &wasm_archive_process_cancellations);
    emscripten::function("archiveSize",                  &wasm_archive_size);
    emscripten::function("archiveClear",                 &wasm_archive_clear);
}
