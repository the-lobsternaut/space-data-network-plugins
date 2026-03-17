#pragma once
#include "da_pipeline/types.h"
#include <vector>

namespace da_pipeline {

class ExclusionZoneAnalyzer {
public:
    ExclusionZoneAnalyzer();

    /// Group exclusion zones by proximity to known launch sites
    std::vector<ExclusionZoneGroup> group_zones(
        const std::vector<ExclusionZone>& notam_zones,
        const std::vector<ExclusionZone>& ntm_zones,
        const std::vector<LaunchSite>& known_sites) const;

    /// Estimate launch azimuth from zone geometry chain
    double estimate_azimuth(const ExclusionZoneGroup& group) const;

    /// Classify activity type from zone grouping
    ActivityType classify_activity(const ExclusionZoneGroup& group) const;

    /// Point-in-zone test (ray cast for polygon, haversine for circle)
    bool point_in_zone(const LatLon& pt, const ExclusionZone& zone) const;

    /// Haversine distance in km
    double haversine_km(const LatLon& a, const LatLon& b) const;

    /// Initial bearing in degrees [0,360)
    double bearing_deg(const LatLon& a, const LatLon& b) const;

    /// Get the built-in launch site database
    const std::vector<LaunchSite>& known_sites() const { return known_sites_; }

private:
    std::vector<LaunchSite> known_sites_;

    bool point_in_polygon(const LatLon& pt, const Polygon& poly) const;
    bool point_in_circle(const LatLon& pt, const Circle& circ) const;
    LatLon zone_centroid(const ExclusionZone& zone) const;
};

} // namespace da_pipeline
