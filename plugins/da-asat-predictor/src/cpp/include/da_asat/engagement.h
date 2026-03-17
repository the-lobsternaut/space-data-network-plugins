#pragma once

#include "da_asat/types.h"
#include <vector>

namespace da_asat {

// ─── Engagement Zone Calculator ────────────────────────────────────────────
//
// Evaluates DA-ASAT engagement feasibility for target satellites from
// specified launch sites. Computes engagement windows, required delta-v,
// intercept geometry, and geographic engagement zone boundaries.

class EngagementCalculator {
public:
    // For each target satellite and launch time window, determine feasibility
    std::vector<EngagementResult> evaluate_engagements(
        const LaunchSite& site,
        const std::vector<TLE>& targets,
        const InterceptorParams& params,
        Timestamp window_start,
        Timestamp window_end,
        double time_step_sec = 60.0) const;

    // Find all engagement windows for a specific target
    std::vector<EngagementWindow> find_engagement_windows(
        const LaunchSite& site,
        const TLE& target,
        const InterceptorParams& params,
        Timestamp window_start,
        Timestamp window_end) const;

    // Evaluate a single engagement at a specific launch time
    EngagementResult evaluate_single(
        const LaunchSite& site,
        const TLE& target,
        const InterceptorParams& params,
        Timestamp launch_time) const;

    // Compute maximum engagement altitude for given interceptor
    double max_engagement_altitude(const InterceptorParams& params) const;

    // Compute engagement zone boundary (geographic locus of feasible intercept points)
    std::vector<LatLon> compute_engagement_zone_boundary(
        const LaunchSite& site,
        const InterceptorParams& params,
        double target_altitude_km) const;

private:
    static constexpr double MIN_ENGAGEMENT_ALT_KM = 150.0;
    static constexpr double MAX_ENGAGEMENT_ALT_KM = 2000.0;
    static constexpr double MIN_CLOSING_VELOCITY   = 2.0;  // km/s

    // Check basic geometric feasibility
    Feasibility check_feasibility(const LaunchSite& site,
                                   const OrbitalState& target,
                                   const InterceptorParams& params) const;
};

} // namespace da_asat
