#pragma once
#include "da_pipeline/types.h"
#include <vector>

namespace da_pipeline {

class EngagementAnalyzer {
public:
    EngagementAnalyzer() = default;

    /// Evaluate DA-ASAT engagement feasibility for multiple targets
    std::vector<EngagementResult> evaluate(
        const LaunchSite& site,
        const std::vector<TLE>& targets,
        const InterceptorParams& params,
        Timestamp window_start,
        Timestamp window_end,
        double step_sec = 60.0) const;

    /// Evaluate a single engagement at a specific launch time
    EngagementResult evaluate_single(
        const LaunchSite& site,
        const TLE& target,
        const InterceptorParams& params,
        Timestamp launch_time) const;

    /// Find engagement windows over a time span
    std::vector<EngagementWindow> find_windows(
        const LaunchSite& site,
        const TLE& target,
        const InterceptorParams& params,
        Timestamp start,
        Timestamp end) const;

    /// Full threat assessment for a launch site
    ThreatAssessment assess_threat(
        const LaunchSite& site,
        const std::vector<TLE>& catalog,
        const InterceptorParams& params,
        Timestamp window_start,
        Timestamp window_end) const;

    /// Default 3-stage generic DA-ASAT interceptor
    static InterceptorParams default_interceptor();

    /// Maximum engagement altitude for an interceptor
    double max_engagement_altitude(const InterceptorParams& params) const;
};

} // namespace da_pipeline
