#pragma once
#include "tle_correlator/types.h"
#include <vector>

namespace tle_correlator {

class Correlator {
public:
    // Find all TLEs whose ground tracks pass through an exclusion zone
    std::vector<Correlation> correlate(const ExclusionZone& zone,
                                        const std::vector<TLE>& catalog,
                                        double step_sec = 30.0) const;

    // Correlate a single TLE against a zone
    Correlation correlate_single(const ExclusionZone& zone,
                                  const TLE& tle,
                                  double step_sec = 30.0) const;

    // Predict launch time based on ascending node crossing
    LaunchTimePrediction predict_launch_time(const ExclusionZone& zone,
                                              const TLE& tle) const;

    // Compute confidence score for a correlation
    static double compute_confidence(double min_distance_km,
                                      double zone_radius_km,
                                      double inclination_match);

    // Process a batch of zones against a catalog
    std::vector<Correlation> batch_correlate(const std::vector<ExclusionZone>& zones,
                                              const std::vector<TLE>& catalog) const;
};

} // namespace tle_correlator
