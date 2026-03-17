#include "da_pipeline/exclusion_zone.h"
#include <algorithm>
#include <cmath>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// Built-in launch site database
// ---------------------------------------------------------------------------
ExclusionZoneAnalyzer::ExclusionZoneAnalyzer() {
    known_sites_ = {
        {"CCAFS",  "Cape Canaveral AFS",       28.4889, -80.5778, {90, 45, 55}},
        {"KSC",    "Kennedy Space Center",      28.5729, -80.6490, {90, 45, 55}},
        {"VAFB",   "Vandenberg SFB",            34.7420, -120.5724, {180, 196, 234}},
        {"BOCA",   "SpaceX Boca Chica",         25.9972, -97.1558, {90, 97}},
        {"WALLOPS","Wallops Flight Facility",    37.8402, -75.4781, {90, 44, 150}},
        {"KODIAK", "Pacific Spaceport Complex",  57.4357, -152.3378, {180, 196, 110}},
        {"BAIKONUR","Baikonur Cosmodrome",       45.9650,  63.3050, {64.8, 51.6}},
        {"PLESETSK","Plesetsk Cosmodrome",       62.9271,  40.5777, {82.9, 97.5}},
        {"KOUROU", "Guiana Space Centre",         5.2360, -52.7690, {90, 0, 10}},
        {"JIUQUAN","Jiuquan Satellite Launch",   40.9581,  100.2914, {97, 65}},
        {"WENCHANG","Wenchang Spacecraft Launch", 19.6145, 110.9510, {90, 97, 55}},
        {"SRIHARIKOTA","Satish Dhawan Space Centre",13.7199, 80.2304, {90, 140}},
    };
}

// ---------------------------------------------------------------------------
// haversine_km
// ---------------------------------------------------------------------------
double ExclusionZoneAnalyzer::haversine_km(const LatLon& a, const LatLon& b) const {
    double lat1 = a.lat * DEG2RAD, lat2 = b.lat * DEG2RAD;
    double dlat = (b.lat - a.lat) * DEG2RAD;
    double dlon = (b.lon - a.lon) * DEG2RAD;

    double h = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(h), std::sqrt(1 - h));
    return R_EARTH_KM * c;
}

// ---------------------------------------------------------------------------
// bearing_deg
// ---------------------------------------------------------------------------
double ExclusionZoneAnalyzer::bearing_deg(const LatLon& a, const LatLon& b) const {
    double lat1 = a.lat * DEG2RAD, lat2 = b.lat * DEG2RAD;
    double dlon = (b.lon - a.lon) * DEG2RAD;

    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double brng = std::atan2(y, x) * RAD2DEG;
    return std::fmod(brng + 360.0, 360.0);
}

// ---------------------------------------------------------------------------
// zone_centroid
// ---------------------------------------------------------------------------
LatLon ExclusionZoneAnalyzer::zone_centroid(const ExclusionZone& zone) const {
    if (zone.type == GeometryType::CIRCLE) {
        return zone.circle.center;
    }
    if (zone.type == GeometryType::POLYGON && !zone.polygon.vertices.empty()) {
        double lat_sum = 0, lon_sum = 0;
        for (auto& v : zone.polygon.vertices) {
            lat_sum += v.lat;
            lon_sum += v.lon;
        }
        int n = static_cast<int>(zone.polygon.vertices.size());
        return {lat_sum / n, lon_sum / n};
    }
    return {(zone.min_lat + zone.max_lat) / 2.0, (zone.min_lon + zone.max_lon) / 2.0};
}

// ---------------------------------------------------------------------------
// point_in_polygon — ray casting
// ---------------------------------------------------------------------------
bool ExclusionZoneAnalyzer::point_in_polygon(const LatLon& pt, const Polygon& poly) const {
    int n = static_cast<int>(poly.vertices.size());
    if (n < 3) return false;

    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double yi = poly.vertices[i].lat, yj = poly.vertices[j].lat;
        double xi = poly.vertices[i].lon, xj = poly.vertices[j].lon;

        if (((yi > pt.lat) != (yj > pt.lat)) &&
            (pt.lon < (xj - xi) * (pt.lat - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

// ---------------------------------------------------------------------------
// point_in_circle
// ---------------------------------------------------------------------------
bool ExclusionZoneAnalyzer::point_in_circle(const LatLon& pt, const Circle& circ) const {
    double dist_km = haversine_km(pt, circ.center);
    double radius_km = circ.radius_nm * NM_TO_KM;
    return dist_km <= radius_km;
}

// ---------------------------------------------------------------------------
// point_in_zone
// ---------------------------------------------------------------------------
bool ExclusionZoneAnalyzer::point_in_zone(const LatLon& pt, const ExclusionZone& zone) const {
    if (zone.type == GeometryType::CIRCLE) {
        return point_in_circle(pt, zone.circle);
    }
    if (zone.type == GeometryType::POLYGON) {
        return point_in_polygon(pt, zone.polygon);
    }
    return false;
}

// ---------------------------------------------------------------------------
// group_zones
// ---------------------------------------------------------------------------
std::vector<ExclusionZoneGroup> ExclusionZoneAnalyzer::group_zones(
    const std::vector<ExclusionZone>& notam_zones,
    const std::vector<ExclusionZone>& ntm_zones,
    const std::vector<LaunchSite>& known_sites) const
{
    // Use provided sites, or fall back to built-in
    const auto& sites = known_sites.empty() ? known_sites_ : known_sites;

    // For each site, find zones within 2000 km
    constexpr double MAX_DIST_KM = 2000.0;
    std::vector<ExclusionZoneGroup> groups;

    for (auto& site : sites) {
        ExclusionZoneGroup group;
        group.associated_site = site;
        group.group_id = "GRP-" + site.site_id;

        LatLon site_pos = {site.lat, site.lon};

        for (auto& nz : notam_zones) {
            LatLon c = zone_centroid(nz);
            if (haversine_km(site_pos, c) < MAX_DIST_KM) {
                group.notam_zones.push_back(nz);
            }
        }

        for (auto& nz : ntm_zones) {
            LatLon c = zone_centroid(nz);
            if (haversine_km(site_pos, c) < MAX_DIST_KM) {
                group.ntm_zones.push_back(nz);
            }
        }

        if (!group.notam_zones.empty() || !group.ntm_zones.empty()) {
            group.azimuth_deg = estimate_azimuth(group);
            group.activity = classify_activity(group);
            groups.push_back(group);
        }
    }

    return groups;
}

// ---------------------------------------------------------------------------
// estimate_azimuth — bearing from launch site through zone chain centroids
// ---------------------------------------------------------------------------
double ExclusionZoneAnalyzer::estimate_azimuth(const ExclusionZoneGroup& group) const {
    LatLon site_pos = {group.associated_site.lat, group.associated_site.lon};

    // Collect all zone centroids
    std::vector<LatLon> centroids;
    for (auto& z : group.notam_zones) centroids.push_back(zone_centroid(z));
    for (auto& z : group.ntm_zones) centroids.push_back(zone_centroid(z));

    if (centroids.empty()) {
        // Return first typical azimuth if available
        if (!group.associated_site.typical_azimuths.empty())
            return group.associated_site.typical_azimuths[0];
        return 0;
    }

    // Sort by distance from site
    std::sort(centroids.begin(), centroids.end(), [&](const LatLon& a, const LatLon& b) {
        return haversine_km(site_pos, a) < haversine_km(site_pos, b);
    });

    // If we have distant zones (downrange), use bearing to the farthest
    // Otherwise use bearing to nearest non-trivial zone
    LatLon target = centroids.back();
    if (haversine_km(site_pos, target) < 50.0) {
        // All zones are very close — use typical azimuth
        if (!group.associated_site.typical_azimuths.empty())
            return group.associated_site.typical_azimuths[0];
    }

    return bearing_deg(site_pos, target);
}

// ---------------------------------------------------------------------------
// classify_activity
// ---------------------------------------------------------------------------
ActivityType ExclusionZoneAnalyzer::classify_activity(const ExclusionZoneGroup& group) const {
    int total_zones = static_cast<int>(group.notam_zones.size() + group.ntm_zones.size());

    // Multiple downrange zones (NTMs) suggest active launch
    if (!group.ntm_zones.empty() && !group.notam_zones.empty()) {
        // Both NOTAM and NTM zones active → likely launch preparation or active launch
        if (total_zones >= 4) return ActivityType::LAUNCH_DETECTED;
        return ActivityType::LAUNCH_PREPARATION;
    }

    // Only NOTAMs
    if (!group.notam_zones.empty() && group.ntm_zones.empty()) {
        if (total_zones >= 2) return ActivityType::LAUNCH_PREPARATION;
        return ActivityType::ROUTINE_ACTIVITY;
    }

    // Only NTMs — could be reentry or recovery
    if (group.notam_zones.empty() && !group.ntm_zones.empty()) {
        return ActivityType::REENTRY;
    }

    return ActivityType::UNKNOWN;
}

} // namespace da_pipeline
