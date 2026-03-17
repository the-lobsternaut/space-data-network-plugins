#pragma once
#include "da_pipeline/types.h"
#include <vector>

namespace da_pipeline {

class TLECorrelator {
public:
    TLECorrelator() = default;

    /// Correlate a single exclusion zone against a TLE catalog
    std::vector<Correlation> correlate(
        const ExclusionZone& zone,
        const std::vector<TLE>& catalog,
        double step_sec = 30.0) const;

    /// Correlate a single zone against a single TLE
    Correlation correlate_single(
        const ExclusionZone& zone,
        const TLE& tle,
        double step_sec = 30.0) const;

    /// Batch correlate: all zones x all TLEs
    std::vector<Correlation> batch_correlate(
        const std::vector<ExclusionZone>& zones,
        const std::vector<TLE>& catalog) const;

    /// Compute confidence from distance, zone radius, inclination match
    static double compute_confidence(double min_dist_km, double zone_radius_km, double incl_match);
};

} // namespace da_pipeline
