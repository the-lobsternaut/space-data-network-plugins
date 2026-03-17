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

// ─── Emscripten Bindings ────────────────────────────────────────────────────

EMSCRIPTEN_BINDINGS(ntm_scraper_wasm) {
    emscripten::function("parseNotice",   &wasm_parse_notice);
    emscripten::function("parseBatch",    &wasm_parse_batch);
    emscripten::function("classify",      &wasm_classify);
    emscripten::function("findByRegion",  &wasm_find_by_region);
}
