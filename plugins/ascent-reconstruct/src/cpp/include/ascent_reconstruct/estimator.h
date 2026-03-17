#pragma once
#include "ascent_reconstruct/types.h"
#include <vector>

namespace ascent_reconstruct {

class AscentEstimator {
public:
    // Main entry: reconstruct trajectory from a tracklet
    ReconstructionResult reconstruct(const Tracklet& tracklet,
                                       const AscentModelParams& initial_guess) const;

    // Detect staging events from observation acceleration discontinuities
    std::vector<StagingEvent> detect_staging(const Tracklet& tracklet) const;

    // Estimate launch site from early observations
    LatLonAlt estimate_launch_site(const Tracklet& tracklet) const;

    // Estimate launch time by extrapolating trajectory backward
    double estimate_launch_time(const Tracklet& tracklet) const;

    // Compute residuals between simulated trajectory and observations
    std::vector<double> compute_residuals(const AscentModelParams& params,
                                            const Tracklet& tracklet) const;

    // Compute RMS of residuals
    double rms_residual(const std::vector<double>& residuals) const;

    // Generate a synthetic tracklet for testing (from model params)
    static Tracklet generate_synthetic_tracklet(const AscentModelParams& params,
                                                  const LatLonAlt& sensor_loc,
                                                  double t_start, double t_end,
                                                  double dt = 2.0,
                                                  double noise_m = 50.0);

    // Set max iterations and convergence tolerance
    void set_max_iterations(int n) { max_iter_ = n; }
    void set_tolerance(double tol) { tol_ = tol; }

private:
    int max_iter_ = 20;
    double tol_ = 100.0; // meters RMS convergence

    // Levenberg-Marquardt-style parameter update
    AscentModelParams update_params(const AscentModelParams& params,
                                      const Tracklet& tracklet) const;
};

} // namespace ascent_reconstruct
