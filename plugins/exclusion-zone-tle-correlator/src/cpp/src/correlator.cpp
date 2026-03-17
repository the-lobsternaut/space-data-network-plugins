#include "tle_correlator/correlator.h"
#include "tle_correlator/sgp4.h"
#include "tle_correlator/geometry.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace tle_correlator {

// ---- compute_confidence -----------------------------------------------------

double Correlator::compute_confidence(double min_distance_km,
                                       double zone_radius_km,
                                       double inclination_match) {
    // Distance factor: 1.0 if inside zone, decays with distance
    double dist_factor;
    if (min_distance_km <= 0.0) {
        dist_factor = 1.0;
    } else if (zone_radius_km > 0 && min_distance_km < zone_radius_km) {
        dist_factor = 1.0 - (min_distance_km / zone_radius_km) * 0.3;
    } else {
        // Exponential decay beyond zone
        double ref_dist = (zone_radius_km > 0) ? zone_radius_km : 100.0;
        dist_factor = std::exp(-min_distance_km / (ref_dist * 2.0));
    }
    dist_factor = std::max(0.0, std::min(1.0, dist_factor));

    // Combined score: 60% distance, 40% inclination match
    double score = 0.6 * dist_factor + 0.4 * inclination_match;
    return std::max(0.0, std::min(1.0, score));
}

// ---- correlate_single -------------------------------------------------------

Correlation Correlator::correlate_single(const ExclusionZone& zone,
                                          const TLE& tle,
                                          double step_sec) const {
    Correlation result;
    result.zone_id = zone.id;
    result.norad_id = tle.norad_id;
    result.object_name = tle.name;

    // Check inclination compatibility
    double zone_lat = std::abs(zone.center.lat);
    double incl = tle.inclination_deg;
    if (zone_lat <= incl) {
        double ratio = zone_lat / std::max(incl, 1.0);
        result.inclination_match = 0.5 + 0.5 * ratio;
    } else {
        double gap = zone_lat - incl;
        double max_gap = 30.0;
        result.inclination_match = std::max(0.0, 1.0 - gap / max_gap);
    }

    // Generate ground track over zone time window
    int64_t start = zone.effective_start;
    int64_t end = zone.effective_end;
    if (end <= start) {
        // Default: use epoch +/- one orbital period
        int64_t epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
        double period_sec = (tle.mean_motion > 0) ? SEC_PER_DAY / tle.mean_motion : 5400.0;
        start = epoch;
        end = epoch + static_cast<int64_t>(period_sec * 2);
    }

    auto track = SGP4::ground_track(tle, start, end, step_sec);
    if (track.empty()) {
        result.confidence = 0.0;
        result.min_distance_km = 99999.0;
        return result;
    }

    // Find closest approach and check intersection
    double min_dist = std::numeric_limits<double>::max();
    int64_t closest_time = start;
    LatLon closest_point = {0, 0};
    bool is_ascending = false;

    for (const auto& gtp : track) {
        bool inside = Geometry::point_in_zone(gtp.position, zone);
        if (inside) {
            min_dist = 0.0;
            closest_time = gtp.timestamp;
            closest_point = gtp.position;
            is_ascending = gtp.ascending;
            break;
        }

        double d;
        if (zone.is_circle) {
            d = Geometry::haversine_km(gtp.position, zone.center) - zone.radius_nm * 1.852;
            if (d < 0) d = 0.0;
        } else if (zone.vertices.size() >= 3) {
            d = Geometry::min_distance_to_polygon_km(gtp.position, zone.vertices);
        } else {
            d = Geometry::haversine_km(gtp.position, zone.center);
        }

        if (d < min_dist) {
            min_dist = d;
            closest_time = gtp.timestamp;
            closest_point = gtp.position;
            is_ascending = gtp.ascending;
        }
    }

    result.min_distance_km = min_dist;
    result.predicted_crossing_time = closest_time;
    result.crossing_point = closest_point;
    result.ascending_pass = is_ascending;

    // Compute zone radius in km for confidence calculation
    double zone_radius_km = zone.radius_nm * 1.852;
    if (!zone.is_circle && zone.vertices.size() >= 3) {
        // Approximate radius from bounding box
        double dlat = zone.max_lat - zone.min_lat;
        double dlon = zone.max_lon - zone.min_lon;
        zone_radius_km = std::max(dlat, dlon) * 111.0 / 2.0;
    }

    result.confidence = compute_confidence(min_dist, zone_radius_km, result.inclination_match);

    return result;
}

// ---- correlate --------------------------------------------------------------

std::vector<Correlation> Correlator::correlate(const ExclusionZone& zone,
                                                const std::vector<TLE>& catalog,
                                                double step_sec) const {
    std::vector<Correlation> results;

    for (const auto& tle : catalog) {
        // Quick inclination rejection: if zone latitude exceeds inclination + 5 deg, skip
        double zone_lat = std::abs(zone.center.lat);
        if (zone_lat > tle.inclination_deg + 5.0) continue;

        Correlation corr = correlate_single(zone, tle, step_sec);

        // Filter by minimum confidence threshold
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

// ---- predict_launch_time ----------------------------------------------------

LaunchTimePrediction Correlator::predict_launch_time(const ExclusionZone& zone,
                                                      const TLE& tle) const {
    LaunchTimePrediction pred;
    pred.norad_id = tle.norad_id;

    // Generate ground track over the zone time window
    int64_t start = zone.effective_start;
    int64_t end = zone.effective_end;
    if (end <= start) {
        int64_t epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
        start = epoch;
        end = epoch + 86400;
    }

    auto track = SGP4::ground_track(tle, start, end, 30.0);

    // Find ascending node crossings (latitude transitions from negative to positive)
    // near the zone longitude
    int64_t best_time = 0;
    double best_lon_diff = 999.0;

    for (size_t i = 1; i < track.size(); ++i) {
        double prev_lat = track[i - 1].position.lat;
        double curr_lat = track[i].position.lat;

        // Ascending node crossing: latitude crosses 0 from south to north
        if (prev_lat < 0.0 && curr_lat >= 0.0) {
            // Linear interpolation for crossing longitude
            double frac = (0.0 - prev_lat) / (curr_lat - prev_lat);
            double cross_lon = track[i - 1].position.lon +
                              frac * (track[i].position.lon - track[i - 1].position.lon);
            int64_t cross_time = track[i - 1].timestamp +
                                static_cast<int64_t>(frac * (track[i].timestamp - track[i - 1].timestamp));

            double lon_diff = std::abs(cross_lon - zone.center.lon);
            if (lon_diff > 180.0) lon_diff = 360.0 - lon_diff;

            if (lon_diff < best_lon_diff) {
                best_lon_diff = lon_diff;
                best_time = cross_time;
            }
        }
    }

    if (best_time > 0) {
        pred.launch_time = best_time;
        // Confidence based on longitude match (within 30 degrees is useful)
        pred.confidence = std::max(0.0, 1.0 - best_lon_diff / 30.0);
        pred.rationale = "Ascending node crossing at lon diff " +
                        std::to_string(static_cast<int>(best_lon_diff)) +
                        " deg from zone center, inclination " +
                        std::to_string(static_cast<int>(tle.inclination_deg)) + " deg";
    } else {
        pred.launch_time = 0;
        pred.confidence = 0.0;
        pred.rationale = "No ascending node crossing found near zone longitude";
    }

    return pred;
}

// ---- batch_correlate --------------------------------------------------------

std::vector<Correlation> Correlator::batch_correlate(
    const std::vector<ExclusionZone>& zones,
    const std::vector<TLE>& catalog) const {

    std::vector<Correlation> all_results;

    for (const auto& zone : zones) {
        auto results = correlate(zone, catalog);
        all_results.insert(all_results.end(), results.begin(), results.end());
    }

    // Sort all results by confidence descending
    std::sort(all_results.begin(), all_results.end(),
        [](const Correlation& a, const Correlation& b) {
            return a.confidence > b.confidence;
        });

    return all_results;
}

} // namespace tle_correlator
