#include "tle_correlator/geometry.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace tle_correlator {

// ---- point_in_polygon (ray casting) -----------------------------------------

bool Geometry::point_in_polygon(const LatLon& point, const std::vector<LatLon>& polygon) {
    if (polygon.size() < 3) return false;

    bool inside = false;
    size_t n = polygon.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double yi = polygon[i].lat;
        double yj = polygon[j].lat;
        double xi = polygon[i].lon;
        double xj = polygon[j].lon;

        bool intersect = ((yi > point.lat) != (yj > point.lat)) &&
                         (point.lon < (xj - xi) * (point.lat - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }

    return inside;
}

// ---- point_in_circle --------------------------------------------------------

bool Geometry::point_in_circle(const LatLon& point, const LatLon& center, double radius_nm) {
    double dist = haversine_nm(point, center);
    return dist <= radius_nm;
}

// ---- point_in_zone ----------------------------------------------------------

bool Geometry::point_in_zone(const LatLon& point, const ExclusionZone& zone) {
    if (zone.is_circle) {
        return point_in_circle(point, zone.center, zone.radius_nm);
    }
    return point_in_polygon(point, zone.vertices);
}

// ---- haversine_km -----------------------------------------------------------

double Geometry::haversine_km(const LatLon& a, const LatLon& b) {
    double dlat = (b.lat - a.lat) * DEG2RAD;
    double dlon = (b.lon - a.lon) * DEG2RAD;
    double lat1 = a.lat * DEG2RAD;
    double lat2 = b.lat * DEG2RAD;

    double h = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::asin(std::sqrt(std::min(1.0, h)));
    return R_EARTH_KM * c;
}

// ---- haversine_nm -----------------------------------------------------------

double Geometry::haversine_nm(const LatLon& a, const LatLon& b) {
    // 1 NM = 1.852 km
    return haversine_km(a, b) / 1.852;
}

// ---- bearing_deg ------------------------------------------------------------

double Geometry::bearing_deg(const LatLon& a, const LatLon& b) {
    double lat1 = a.lat * DEG2RAD;
    double lat2 = b.lat * DEG2RAD;
    double dlon = (b.lon - a.lon) * DEG2RAD;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double bearing = std::atan2(x, y) * RAD2DEG;
    return std::fmod(bearing + 360.0, 360.0);
}

// ---- min distance from point to line segment (great circle approximation) ---

static double point_to_segment_km(const LatLon& p, const LatLon& a, const LatLon& b) {
    // Use a projection approach in local coordinates
    // Compute distance from p to the line segment a-b
    double d_ab = Geometry::haversine_km(a, b);
    if (d_ab < 1e-9) return Geometry::haversine_km(p, a);

    double d_pa = Geometry::haversine_km(p, a);
    double d_pb = Geometry::haversine_km(p, b);

    // Use the cross-track distance formula
    // bearing from a to p
    double brng_ap = Geometry::bearing_deg(a, p) * DEG2RAD;
    double brng_ab = Geometry::bearing_deg(a, b) * DEG2RAD;

    // Cross-track distance
    double d_xt = std::asin(std::sin(d_pa / R_EARTH_KM) * std::sin(brng_ap - brng_ab)) * R_EARTH_KM;
    double abs_dxt = std::abs(d_xt);

    // Along-track distance from a
    double d_at = std::acos(std::cos(d_pa / R_EARTH_KM) / std::cos(abs_dxt / R_EARTH_KM)) * R_EARTH_KM;

    // Check if the projection falls within the segment
    if (d_at >= 0 && d_at <= d_ab) {
        return abs_dxt;
    }

    // Otherwise, return distance to nearest endpoint
    return std::min(d_pa, d_pb);
}

// ---- min_distance_to_polygon_km ---------------------------------------------

double Geometry::min_distance_to_polygon_km(const LatLon& point, const std::vector<LatLon>& polygon) {
    if (polygon.empty()) return std::numeric_limits<double>::max();

    // Check if point is inside first
    if (point_in_polygon(point, polygon)) return 0.0;

    double min_dist = std::numeric_limits<double>::max();
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        double d = point_to_segment_km(point, polygon[i], polygon[j]);
        min_dist = std::min(min_dist, d);
    }

    return min_dist;
}

// ---- min_distance_to_zone_km ------------------------------------------------

double Geometry::min_distance_to_zone_km(const std::vector<GroundTrackPoint>& track,
                                          const ExclusionZone& zone) {
    double min_dist = std::numeric_limits<double>::max();

    for (const auto& gtp : track) {
        double d;
        if (zone.is_circle) {
            d = haversine_km(gtp.position, zone.center) - zone.radius_nm * 1.852;
            if (d < 0) d = 0.0;
        } else {
            if (zone.vertices.size() < 3) {
                d = haversine_km(gtp.position, zone.center);
            } else {
                d = min_distance_to_polygon_km(gtp.position, zone.vertices);
            }
        }
        min_dist = std::min(min_dist, d);
    }

    return min_dist;
}

// ---- bbox_overlap -----------------------------------------------------------

bool Geometry::bbox_overlap(const ExclusionZone& zone, double lat, double lon, double margin_deg) {
    return lat >= (zone.min_lat - margin_deg) &&
           lat <= (zone.max_lat + margin_deg) &&
           lon >= (zone.min_lon - margin_deg) &&
           lon <= (zone.max_lon + margin_deg);
}

} // namespace tle_correlator
