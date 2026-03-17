#include "da_pipeline/tle_correlator.h"
#include "da_pipeline/sgp4.h"
#include "da_pipeline/exclusion_zone.h"
#include <cmath>
#include <algorithm>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// compute_confidence
// ---------------------------------------------------------------------------
double TLECorrelator::compute_confidence(double min_dist_km, double zone_radius_km,
                                         double incl_match) {
    if (zone_radius_km <= 0) zone_radius_km = 50.0;

    // Distance factor: 1.0 when inside, decays as ratio grows
    double dist_factor = 0;
    if (min_dist_km <= zone_radius_km) {
        dist_factor = 1.0 - 0.5 * (min_dist_km / zone_radius_km);
    } else {
        double ratio = min_dist_km / zone_radius_km;
        dist_factor = std::max(0.0, 1.0 - ratio * 0.5);
    }

    // Inclination match factor (0..1, higher = better match)
    double incl_factor = std::max(0.0, std::min(1.0, incl_match));

    // Combined confidence
    double conf = 0.7 * dist_factor + 0.3 * incl_factor;
    return std::max(0.0, std::min(1.0, conf));
}

// ---------------------------------------------------------------------------
// correlate_single
// ---------------------------------------------------------------------------
Correlation TLECorrelator::correlate_single(
    const ExclusionZone& zone, const TLE& tle, double step_sec) const
{
    Correlation corr;
    corr.zone_id = zone.id;
    corr.norad_id = tle.norad_id;
    corr.object_name = tle.name;
    corr.confidence = 0;
    corr.min_distance_km = 1e9;

    // Determine zone center and radius
    LatLon zone_center;
    double zone_radius_km = 0;
    if (zone.type == GeometryType::CIRCLE) {
        zone_center = zone.circle.center;
        zone_radius_km = zone.circle.radius_nm * NM_TO_KM;
    } else if (zone.type == GeometryType::POLYGON && !zone.polygon.vertices.empty()) {
        double lat_sum = 0, lon_sum = 0;
        for (auto& v : zone.polygon.vertices) {
            lat_sum += v.lat;
            lon_sum += v.lon;
        }
        int n = static_cast<int>(zone.polygon.vertices.size());
        zone_center = {lat_sum / n, lon_sum / n};
        // Estimate radius as max distance from centroid
        ExclusionZoneAnalyzer ez;
        for (auto& v : zone.polygon.vertices) {
            double d = ez.haversine_km(zone_center, v);
            if (d > zone_radius_km) zone_radius_km = d;
        }
    } else {
        return corr; // Unknown geometry
    }

    if (zone_radius_km < 1.0) zone_radius_km = 50.0;

    // Generate ground track over the zone's time window
    Timestamp start = zone.effective_start;
    Timestamp end = zone.effective_end;

    // If no time window, use epoch +/- 12 hours
    if (start == 0 || end == 0) {
        Timestamp epoch_unix = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
        start = epoch_unix - 43200;
        end = epoch_unix + 43200;
    }

    // Clamp window to 48h max to avoid excessive computation
    if (end - start > 172800) end = start + 172800;

    auto track = SGP4::ground_track(tle, start, end, step_sec);

    ExclusionZoneAnalyzer ez;
    for (auto& gtp : track) {
        double dist = ez.haversine_km(gtp.position, zone_center);
        if (dist < corr.min_distance_km) {
            corr.min_distance_km = dist;
            corr.crossing_point = gtp.position;
            corr.predicted_crossing_time = gtp.timestamp;
            corr.ascending_pass = gtp.ascending;
        }
    }

    // Inclination match: does the zone latitude fall within the inclination band?
    double zone_lat = std::abs(zone_center.lat);
    double incl = tle.inclination_deg;
    double incl_match = 0;
    if (zone_lat <= incl) {
        incl_match = 1.0 - (zone_lat / incl) * 0.3;
    } else {
        incl_match = std::max(0.0, 1.0 - (zone_lat - incl) / 10.0);
    }
    corr.inclination_match = incl_match;

    corr.confidence = compute_confidence(corr.min_distance_km, zone_radius_km, incl_match);

    return corr;
}

// ---------------------------------------------------------------------------
// correlate
// ---------------------------------------------------------------------------
std::vector<Correlation> TLECorrelator::correlate(
    const ExclusionZone& zone, const std::vector<TLE>& catalog, double step_sec) const
{
    std::vector<Correlation> results;
    for (auto& tle : catalog) {
        auto corr = correlate_single(zone, tle, step_sec);
        if (corr.confidence > 0.1) {
            results.push_back(corr);
        }
    }

    // Sort by confidence descending
    std::sort(results.begin(), results.end(),
              [](const Correlation& a, const Correlation& b) {
                  return a.confidence > b.confidence;
              });

    return results;
}

// ---------------------------------------------------------------------------
// batch_correlate
// ---------------------------------------------------------------------------
std::vector<Correlation> TLECorrelator::batch_correlate(
    const std::vector<ExclusionZone>& zones, const std::vector<TLE>& catalog) const
{
    std::vector<Correlation> all;
    for (auto& zone : zones) {
        auto results = correlate(zone, catalog);
        all.insert(all.end(), results.begin(), results.end());
    }

    std::sort(all.begin(), all.end(),
              [](const Correlation& a, const Correlation& b) {
                  return a.confidence > b.confidence;
              });

    return all;
}

} // namespace da_pipeline
