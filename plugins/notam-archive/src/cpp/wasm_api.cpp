#include "notam_archive/archive.h"
#include "notam_archive/analyzer.h"
#include <string>
#include <sstream>
#include <cstdint>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define WASM_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define WASM_EXPORT
#endif

namespace {
    notam_archive::NotamArchive g_archive;

    // Simple JSON helpers (no external dependency)
    std::string escape_json(const std::string& s) {
        std::string out;
        for (char c : s) {
            switch (c) {
                case '"': out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                default: out += c;
            }
        }
        return out;
    }

    std::string notam_to_json(const notam_archive::NOTAM& n) {
        std::ostringstream os;
        os << "{\"notam_id\":\"" << escape_json(n.notam_id) << "\""
           << ",\"location_id\":\"" << escape_json(n.location_id) << "\""
           << ",\"issued\":" << n.issued
           << ",\"effective_start\":" << n.effective_start
           << ",\"effective_end\":" << n.effective_end
           << ",\"altitude_floor_ft\":" << n.altitude_floor_ft
           << ",\"altitude_ceiling_ft\":" << n.altitude_ceiling_ft
           << ",\"center_lat\":" << n.center_lat
           << ",\"center_lon\":" << n.center_lon
           << ",\"radius_nm\":" << n.radius_nm
           << ",\"notam_text\":\"" << escape_json(n.notam_text) << "\""
           << ",\"classification\":" << static_cast<int>(n.classification)
           << "}";
        return os.str();
    }

    std::string notams_to_json(const std::vector<notam_archive::NOTAM>& notams) {
        std::ostringstream os;
        os << "[";
        for (size_t i = 0; i < notams.size(); ++i) {
            if (i > 0) os << ",";
            os << notam_to_json(notams[i]);
        }
        os << "]";
        return os.str();
    }

    std::string change_to_json(const notam_archive::NotamChange& c) {
        std::ostringstream os;
        os << "{\"old_notam_id\":\"" << escape_json(c.old_notam_id) << "\""
           << ",\"new_notam_id\":\"" << escape_json(c.new_notam_id) << "\""
           << ",\"change_type\":" << static_cast<int>(c.change_type)
           << ",\"detected_at\":" << c.detected_at
           << ",\"old_start\":" << c.old_start
           << ",\"old_end\":" << c.old_end
           << ",\"new_start\":" << c.new_start
           << ",\"new_end\":" << c.new_end
           << ",\"description\":\"" << escape_json(c.description) << "\""
           << ",\"is_scrub\":" << (g_archive.is_scrub_indicator(c) ? "true" : "false")
           << ",\"is_reschedule\":" << (g_archive.is_reschedule_indicator(c) ? "true" : "false")
           << "}";
        return os.str();
    }

    std::string lead_time_to_json(const notam_archive::LeadTimeRecord& lt) {
        std::ostringstream os;
        os << "{\"notam_id\":\"" << escape_json(lt.notam_id) << "\""
           << ",\"site_id\":\"" << escape_json(lt.site_id) << "\""
           << ",\"vehicle\":\"" << escape_json(lt.vehicle) << "\""
           << ",\"orbit\":" << static_cast<int>(lt.orbit)
           << ",\"lead_time_hours\":" << lt.lead_time_hours
           << ",\"notam_issued\":" << lt.notam_issued
           << ",\"launch_time\":" << lt.launch_time
           << "}";
        return os.str();
    }

    // Minimal JSON string field extraction
    std::string json_get_string(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\":\"";
        auto pos = json.find(search);
        if (pos == std::string::npos) return "";
        pos += search.size();
        auto end = json.find("\"", pos);
        if (end == std::string::npos) return "";
        return json.substr(pos, end - pos);
    }

    int64_t json_get_int64(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\":";
        auto pos = json.find(search);
        if (pos == std::string::npos) return 0;
        pos += search.size();
        return std::stoll(json.substr(pos));
    }

    double json_get_double(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\":";
        auto pos = json.find(search);
        if (pos == std::string::npos) return 0.0;
        pos += search.size();
        return std::stod(json.substr(pos));
    }

    notam_archive::NotamClassification parse_classification(const std::string& s) {
        if (s == "LAUNCH") return notam_archive::NotamClassification::LAUNCH;
        if (s == "REENTRY") return notam_archive::NotamClassification::REENTRY;
        if (s == "HAZARD") return notam_archive::NotamClassification::HAZARD;
        if (s == "AIRSPACE") return notam_archive::NotamClassification::AIRSPACE;
        return notam_archive::NotamClassification::ROUTINE;
    }

    notam_archive::OrbitType parse_orbit(const std::string& s) {
        if (s == "LEO") return notam_archive::OrbitType::LEO;
        if (s == "GTO") return notam_archive::OrbitType::GTO;
        if (s == "SSO") return notam_archive::OrbitType::SSO;
        if (s == "POLAR") return notam_archive::OrbitType::POLAR;
        if (s == "ISS") return notam_archive::OrbitType::ISS;
        if (s == "MEO") return notam_archive::OrbitType::MEO;
        if (s == "HEO") return notam_archive::OrbitType::HEO;
        if (s == "ESCAPE") return notam_archive::OrbitType::ESCAPE;
        return notam_archive::OrbitType::UNKNOWN;
    }
}

extern "C" {

WASM_EXPORT
const char* wasm_add_notam(const char* json_str) {
    static std::string result;
    std::string json(json_str);

    notam_archive::NOTAM notam;
    notam.notam_id = json_get_string(json, "notam_id");
    notam.location_id = json_get_string(json, "location_id");
    notam.issued = json_get_int64(json, "issued");
    notam.effective_start = json_get_int64(json, "effective_start");
    notam.effective_end = json_get_int64(json, "effective_end");
    notam.altitude_floor_ft = json_get_double(json, "altitude_floor_ft");
    notam.altitude_ceiling_ft = json_get_double(json, "altitude_ceiling_ft");
    notam.center_lat = json_get_double(json, "center_lat");
    notam.center_lon = json_get_double(json, "center_lon");
    notam.radius_nm = json_get_double(json, "radius_nm");
    notam.notam_text = json_get_string(json, "notam_text");
    notam.classification = parse_classification(json_get_string(json, "classification"));

    g_archive.add(notam);
    result = "{\"status\":\"ok\",\"size\":" + std::to_string(g_archive.size()) + "}";
    return result.c_str();
}

WASM_EXPORT
const char* wasm_query_by_time(int64_t start, int64_t end) {
    static std::string result;
    auto notams = g_archive.query_by_time(start, end);
    result = notams_to_json(notams);
    return result.c_str();
}

WASM_EXPORT
const char* wasm_query_by_location(double lat, double lon, double radius_nm) {
    static std::string result;
    auto notams = g_archive.query_by_location(lat, lon, radius_nm);
    result = notams_to_json(notams);
    return result.c_str();
}

WASM_EXPORT
const char* wasm_query_by_classification(const char* classification_str) {
    static std::string result;
    auto cls = parse_classification(std::string(classification_str));
    auto notams = g_archive.query_by_classification(cls);
    result = notams_to_json(notams);
    return result.c_str();
}

WASM_EXPORT
const char* wasm_query_active_at(int64_t time) {
    static std::string result;
    auto notams = g_archive.query_active_at(time);
    result = notams_to_json(notams);
    return result.c_str();
}

WASM_EXPORT
const char* wasm_detect_changes() {
    static std::string result;
    auto changes = g_archive.detect_changes();
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < changes.size(); ++i) {
        if (i > 0) os << ",";
        os << change_to_json(changes[i]);
    }
    os << "]";
    result = os.str();
    return result.c_str();
}

WASM_EXPORT
const char* wasm_add_launch(const char* json_str) {
    static std::string result;
    std::string json(json_str);

    notam_archive::LaunchRecord launch;
    launch.launch_id = json_get_string(json, "launch_id");
    launch.site_id = json_get_string(json, "site_id");
    launch.vehicle = json_get_string(json, "vehicle");
    launch.orbit = parse_orbit(json_get_string(json, "orbit"));
    launch.launch_time = json_get_int64(json, "launch_time");
    // Parse notam_ids (simplified: comma-separated in a string field)
    std::string ids = json_get_string(json, "notam_ids");
    if (!ids.empty()) {
        std::istringstream iss(ids);
        std::string id;
        while (std::getline(iss, id, ',')) {
            launch.notam_ids.push_back(id);
        }
    }
    std::string scrubbed = json_get_string(json, "scrubbed");
    launch.scrubbed = (scrubbed == "true" || scrubbed == "1");

    g_archive.add_launch(launch);

    // Record lead times for all associated NOTAMs
    for (const auto& nid : launch.notam_ids) {
        g_archive.record_lead_time(nid, launch);
    }

    result = "{\"status\":\"ok\",\"launches\":" +
             std::to_string(g_archive.get_launches().size()) + "}";
    return result.c_str();
}

WASM_EXPORT
const char* wasm_get_lead_times() {
    static std::string result;
    auto lts = g_archive.get_lead_times();
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < lts.size(); ++i) {
        if (i > 0) os << ",";
        os << lead_time_to_json(lts[i]);
    }
    os << "]";
    result = os.str();
    return result.c_str();
}

WASM_EXPORT
const char* wasm_compute_statistics() {
    static std::string result;
    notam_archive::NotamAnalyzer analyzer(g_archive);
    auto stats = analyzer.compute_statistics();

    std::ostringstream os;
    os << "{\"total_notams\":" << stats.total_notams
       << ",\"total_launches\":" << stats.total_launches
       << ",\"total_changes\":" << stats.total_changes
       << ",\"earliest_notam\":" << stats.earliest_notam
       << ",\"latest_notam\":" << stats.latest_notam
       << ",\"overall_avg_lead_time_hours\":" << stats.overall_avg_lead_time_hours
       << ",\"overall_scrub_rate\":" << stats.overall_scrub_rate
       << ",\"sites\":" << stats.by_site.size()
       << ",\"vehicles\":" << stats.by_vehicle.size()
       << ",\"orbit_types\":" << stats.by_orbit.size()
       << "}";
    result = os.str();
    return result.c_str();
}

WASM_EXPORT
const char* wasm_site_statistics(const char* site_id_str) {
    static std::string result;
    notam_archive::NotamAnalyzer analyzer(g_archive);
    auto stats = analyzer.compute_site_stats(std::string(site_id_str));

    std::ostringstream os;
    os << "{\"site_id\":\"" << escape_json(stats.site_id) << "\""
       << ",\"total_notams\":" << stats.total_notams
       << ",\"launch_notams\":" << stats.launch_notams
       << ",\"hazard_notams\":" << stats.hazard_notams
       << ",\"avg_lead_time_hours\":" << stats.avg_lead_time_hours
       << ",\"median_lead_time_hours\":" << stats.median_lead_time_hours
       << ",\"min_lead_time_hours\":" << stats.min_lead_time_hours
       << ",\"max_lead_time_hours\":" << stats.max_lead_time_hours
       << ",\"stddev_lead_time_hours\":" << stats.stddev_lead_time_hours
       << ",\"total_launches\":" << stats.total_launches
       << ",\"scrubbed_launches\":" << stats.scrubbed_launches
       << ",\"scrub_rate\":" << stats.scrub_rate
       << "}";
    result = os.str();
    return result.c_str();
}

WASM_EXPORT
const char* wasm_vehicle_statistics(const char* vehicle_str) {
    static std::string result;
    notam_archive::NotamAnalyzer analyzer(g_archive);
    auto stats = analyzer.compute_vehicle_stats(std::string(vehicle_str));

    std::ostringstream os;
    os << "{\"vehicle\":\"" << escape_json(stats.vehicle) << "\""
       << ",\"total_launches\":" << stats.total_launches
       << ",\"avg_lead_time_hours\":" << stats.avg_lead_time_hours
       << ",\"median_lead_time_hours\":" << stats.median_lead_time_hours
       << ",\"avg_notams_per_launch\":" << stats.avg_notams_per_launch
       << ",\"scrubbed\":" << stats.scrubbed
       << ",\"scrub_rate\":" << stats.scrub_rate
       << "}";
    result = os.str();
    return result.c_str();
}

WASM_EXPORT
const char* wasm_activity_timeline(int64_t start, int64_t end, int step_hours) {
    static std::string result;
    notam_archive::NotamAnalyzer analyzer(g_archive);
    auto timeline = analyzer.activity_timeline(start, end, step_hours);

    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < timeline.size(); ++i) {
        if (i > 0) os << ",";
        os << "{\"time\":" << timeline[i].first
           << ",\"count\":" << timeline[i].second << "}";
    }
    os << "]";
    result = os.str();
    return result.c_str();
}

WASM_EXPORT
const char* wasm_detect_anomalies() {
    static std::string result;
    notam_archive::NotamAnalyzer analyzer(g_archive);
    auto anomalies = analyzer.detect_anomalies();

    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < anomalies.size(); ++i) {
        if (i > 0) os << ",";
        os << "{\"notam_id\":\"" << escape_json(anomalies[i].notam_id) << "\""
           << ",\"description\":\"" << escape_json(anomalies[i].description) << "\""
           << ",\"severity\":" << anomalies[i].severity
           << "}";
    }
    os << "]";
    result = os.str();
    return result.c_str();
}

WASM_EXPORT
int wasm_archive_size() {
    return static_cast<int>(g_archive.size());
}

WASM_EXPORT
void wasm_clear() {
    g_archive.clear();
}

} // extern "C"
