#pragma once
#include "da_pipeline/types.h"
#include <vector>

namespace da_pipeline {

class AscentEstimator {
public:
    AscentEstimator() = default;

    /// Reconstruct ascent trajectory from tracklet + initial guess
    ReconstructionResult reconstruct(
        const Tracklet& tracklet,
        const AscentModelParams& guess) const;

    /// Detect staging events from acceleration discontinuities
    std::vector<StagingEvent> detect_staging(const Tracklet& tracklet) const;

    /// Estimate launch site from early tracklet observations
    LatLonAlt estimate_launch_site(const Tracklet& tracklet) const;

    /// Estimate launch time by extrapolating early observations
    double estimate_launch_time(const Tracklet& tracklet) const;

    /// Generate a family of ascent trajectories around a nominal azimuth
    AscentTrajectoryFamily generate_family(
        const LaunchSite& site,
        double azimuth_deg,
        double azimuth_spread_deg = 10.0) const;

    /// Get a default 2-stage Falcon-9-like vehicle model
    static AscentModelParams default_vehicle(double lat, double lon, double azimuth_deg);
};

} // namespace da_pipeline
