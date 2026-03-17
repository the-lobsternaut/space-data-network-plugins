#pragma once
#include "tle_correlator/types.h"
#include <vector>

namespace tle_correlator {

class Geometry {
public:
    // Point-in-polygon test (ray casting algorithm)
    static bool point_in_polygon(const LatLon& point, const std::vector<LatLon>& polygon);

    // Point-in-circle test
    static bool point_in_circle(const LatLon& point, const LatLon& center, double radius_nm);

    // Point in exclusion zone (dispatches to polygon or circle)
    static bool point_in_zone(const LatLon& point, const ExclusionZone& zone);

    // Haversine distance in km
    static double haversine_km(const LatLon& a, const LatLon& b);

    // Haversine distance in NM
    static double haversine_nm(const LatLon& a, const LatLon& b);

    // Compute bearing from a to b (degrees)
    static double bearing_deg(const LatLon& a, const LatLon& b);

    // Find minimum distance from a point to a polygon edge
    static double min_distance_to_polygon_km(const LatLon& point, const std::vector<LatLon>& polygon);

    // Find minimum distance from a ground track to an exclusion zone
    static double min_distance_to_zone_km(const std::vector<GroundTrackPoint>& track, const ExclusionZone& zone);

    // Quick bounding box rejection test
    static bool bbox_overlap(const ExclusionZone& zone, double lat, double lon, double margin_deg = 1.0);
};

} // namespace tle_correlator
