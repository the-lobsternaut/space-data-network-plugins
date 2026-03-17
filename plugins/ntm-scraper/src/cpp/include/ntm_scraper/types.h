#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include <ctime>

namespace ntm_scraper {

// Geographic coordinate
struct LatLon {
    double lat = 0.0;
    double lon = 0.0;
};

// Polygon defined by vertices
struct Polygon {
    std::vector<LatLon> vertices;
};

// Circle defined by center + radius
struct Circle {
    LatLon center;
    double radius_nm = 0.0;
};

// Types of exclusion zone geometry
enum class GeometryType {
    CIRCLE,
    POLYGON,
    UNKNOWN
};

// Geometry wrapper
struct ExclusionZone {
    GeometryType type = GeometryType::UNKNOWN;
    Circle circle;
    Polygon polygon;

    // Bounding box for quick rejection
    double min_lat = 90.0, max_lat = -90.0;
    double min_lon = 180.0, max_lon = -180.0;

    void compute_bounds() {
        if (type == GeometryType::CIRCLE) {
            double dlat = circle.radius_nm / 60.0;
            double dlon = dlat / std::cos(circle.center.lat * M_PI / 180.0);
            min_lat = circle.center.lat - dlat;
            max_lat = circle.center.lat + dlat;
            min_lon = circle.center.lon - dlon;
            max_lon = circle.center.lon + dlon;
        } else if (type == GeometryType::POLYGON) {
            for (auto& v : polygon.vertices) {
                if (v.lat < min_lat) min_lat = v.lat;
                if (v.lat > max_lat) max_lat = v.lat;
                if (v.lon < min_lon) min_lon = v.lon;
                if (v.lon > max_lon) max_lon = v.lon;
            }
        }
    }
};

// Classification of the notice hazard
enum class HazardClassification {
    ROCKET_DEBRIS,
    SPLASHDOWN,
    BOOSTER_RECOVERY,
    FAIRING_RECOVERY,
    GENERAL_HAZARD,
    UNKNOWN
};

// NGA Broadcast warning area
enum class BroadcastArea {
    HYDROLANT,   // Atlantic
    HYDROPAC,    // Pacific
    NAVAREA_IV,  // North Atlantic
    NAVAREA_XII, // North Pacific
    UNKNOWN
};

// Parsed maritime notice
struct MaritimeNotice {
    std::string notice_id;          // e.g. "HYDROLANT 1234/2024"
    std::string raw_text;           // Original NGA text
    BroadcastArea area = BroadcastArea::UNKNOWN;
    int64_t effective_start = 0;    // Unix timestamp
    int64_t effective_end = 0;      // Unix timestamp
    HazardClassification classification = HazardClassification::UNKNOWN;
    ExclusionZone zone;
    std::string authority;          // Issuing authority
    std::string cancel_notice_id;   // If this cancels a previous notice
    bool is_cancelled = false;
    double confidence = 0.0;        // Classification confidence [0,1]
};

// Timestamp helper
struct Timestamp {
    int year = 0, month = 0, day = 0;
    int hour = 0, minute = 0;

    int64_t to_unix() const {
        struct tm t = {};
        t.tm_year = year - 1900;
        t.tm_mon = month - 1;
        t.tm_mday = day;
        t.tm_hour = hour;
        t.tm_min = minute;
        t.tm_sec = 0;
        return static_cast<int64_t>(timegm(&t));
    }
};

} // namespace ntm_scraper
