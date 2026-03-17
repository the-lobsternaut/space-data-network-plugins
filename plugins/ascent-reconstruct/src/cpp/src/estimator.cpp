#include "ascent_reconstruct/estimator.h"
#include "ascent_reconstruct/trajectory.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <numeric>

namespace ascent_reconstruct {

std::vector<StagingEvent> AscentEstimator::detect_staging(const Tracklet& tracklet) const {
    std::vector<StagingEvent> events;
    const auto& obs = tracklet.observations;
    if (obs.size() < 6) return events;

    // Compute acceleration magnitudes using a wider baseline (4 observations each side)
    // to reduce noise from position measurement errors
    std::vector<double> accels;
    std::vector<double> times;
    int half_win = 4;

    for (size_t i = half_win; i + half_win < obs.size(); ++i) {
        double dt_before = obs[i].time - obs[i - half_win].time;
        double dt_after = obs[i + half_win].time - obs[i].time;
        if (dt_before <= 0 || dt_after <= 0) continue;

        Vec3 v_before = (obs[i].position - obs[i - half_win].position) * (1.0 / dt_before);
        Vec3 v_after = (obs[i + half_win].position - obs[i].position) * (1.0 / dt_after);

        double dt_mid = (dt_before + dt_after) * 0.5;
        Vec3 accel = (v_after - v_before) * (1.0 / dt_mid);
        accels.push_back(accel.norm());
        times.push_back(obs[i].time);
    }

    if (accels.size() < 15) return events;

    // Detect staging by finding local acceleration minima where the post-dip
    // acceleration exceeds the pre-dip acceleration. This distinguishes staging
    // (where a new stage produces higher thrust-to-weight) from gravity turn
    // transitions (where acceleration drops and stays low).
    struct Candidate {
        size_t idx;
        double score;
        double time;
    };

    std::vector<Candidate> candidates;

    // Window for computing before/after averages (in data points)
    int avg_win = 5;
    for (size_t i = avg_win; i + avg_win < accels.size(); ++i) {
        double val = accels[i];

        // Compute average acceleration in windows before and after this point
        double avg_before = 0, avg_after = 0;
        for (int j = 1; j <= avg_win; ++j) {
            avg_before += accels[i - j];
            avg_after += accels[i + j];
        }
        avg_before /= avg_win;
        avg_after /= avg_win;

        // Staging signature:
        // 1. Current accel is lower than both before and after averages
        // 2. After-average is HIGHER than before-average (new stage produces more accel)
        // 3. The dip is significant (at least 30% below both)
        if (val < avg_before * 0.7 && val < avg_after * 0.7 &&
            avg_after > avg_before * 1.1 && avg_before > 2.0) {
            double dip = std::min(avg_before - val, avg_after - val);
            candidates.push_back({i, dip, times[i]});
        }
    }

    // Merge candidates that are close together, keeping the deepest dip
    double min_separation = 20.0;
    std::vector<Candidate> merged;
    for (const auto& c : candidates) {
        if (merged.empty() || (c.time - merged.back().time) > min_separation) {
            merged.push_back(c);
        } else if (c.score > merged.back().score) {
            merged.back() = c;
        }
    }

    int stage_num = 1;
    for (const auto& c : merged) {
        StagingEvent ev;
        ev.time = c.time;
        ev.from_stage = stage_num;
        ev.to_stage = stage_num + 1;
        ev.delta_accel = c.score;

        // Find nearest observation for position/altitude
        size_t obs_idx = c.idx + half_win;
        if (obs_idx < obs.size()) {
            ev.position = obs[obs_idx].position;
            LatLonAlt lla = TrajectorySimulator::ecef_to_geodetic(obs[obs_idx].position);
            ev.altitude_m = lla.alt;

            if (obs_idx > 0) {
                double dt_local = obs[obs_idx].time - obs[obs_idx-1].time;
                if (dt_local > 0) {
                    Vec3 v = (obs[obs_idx].position - obs[obs_idx-1].position) * (1.0 / dt_local);
                    ev.speed_mps = v.norm();
                }
            }
        }

        events.push_back(ev);
        stage_num++;
    }

    return events;
}

LatLonAlt AscentEstimator::estimate_launch_site(const Tracklet& tracklet) const {
    const auto& obs = tracklet.observations;
    if (obs.empty()) return {0, 0, 0};

    if (obs.size() < 2) {
        return TrajectorySimulator::ecef_to_geodetic(obs[0].position);
    }

    // Use first few observations to extrapolate backward to ground
    size_t n_use = std::min((size_t)5, obs.size());

    // Compute average velocity from first observations
    Vec3 vel = {0, 0, 0};
    int count = 0;
    for (size_t i = 1; i < n_use; ++i) {
        double dt = obs[i].time - obs[i-1].time;
        if (dt > 0) {
            Vec3 v = (obs[i].position - obs[i-1].position) * (1.0 / dt);
            vel = vel + v;
            count++;
        }
    }
    if (count > 0) {
        vel = vel * (1.0 / count);
    }

    // Compute average acceleration to handle parabolic trajectory
    Vec3 accel = {0, 0, 0};
    int accel_count = 0;
    for (size_t i = 1; i + 1 < n_use; ++i) {
        double dt1 = obs[i].time - obs[i-1].time;
        double dt2 = obs[i+1].time - obs[i].time;
        if (dt1 > 0 && dt2 > 0) {
            Vec3 v1 = (obs[i].position - obs[i-1].position) * (1.0 / dt1);
            Vec3 v2 = (obs[i+1].position - obs[i].position) * (1.0 / dt2);
            double dt_mid = (dt1 + dt2) * 0.5;
            Vec3 a = (v2 - v1) * (1.0 / dt_mid);
            accel = accel + a;
            accel_count++;
        }
    }
    if (accel_count > 0) {
        accel = accel * (1.0 / accel_count);
    }

    // Extrapolate backward from first observation: pos(t) = pos0 + vel*t + 0.5*accel*t^2
    // Find t where altitude = 0
    double dt_back = obs[0].time; // time from launch to first obs (we estimate this)
    Vec3 pos0 = obs[0].position;

    // Search for ground by stepping backward
    Vec3 best_pos = pos0;
    double best_alt = 1e12;

    for (double t = 0; t < obs[0].time + 200.0; t += 1.0) {
        Vec3 p = pos0 - vel * t + accel * (0.5 * t * t);
        LatLonAlt lla = TrajectorySimulator::ecef_to_geodetic(p);
        if (std::abs(lla.alt) < best_alt) {
            best_alt = std::abs(lla.alt);
            best_pos = p;
        }
    }

    return TrajectorySimulator::ecef_to_geodetic(best_pos);
}

double AscentEstimator::estimate_launch_time(const Tracklet& tracklet) const {
    const auto& obs = tracklet.observations;
    if (obs.size() < 2) return obs.empty() ? 0.0 : obs[0].time;

    // Compute altitude at first few observations
    size_t n_use = std::min((size_t)5, obs.size());

    std::vector<double> times;
    std::vector<double> alts;
    for (size_t i = 0; i < n_use; ++i) {
        LatLonAlt lla = TrajectorySimulator::ecef_to_geodetic(obs[i].position);
        times.push_back(obs[i].time);
        alts.push_back(lla.alt);
    }

    // Fit linear model: alt = a + b*(t - t0) to find when alt = 0
    // Simple linear regression
    if (times.size() >= 2) {
        double sum_t = 0, sum_a = 0, sum_tt = 0, sum_ta = 0;
        int n = (int)times.size();
        for (int i = 0; i < n; ++i) {
            sum_t += times[i];
            sum_a += alts[i];
            sum_tt += times[i] * times[i];
            sum_ta += times[i] * alts[i];
        }
        double denom = n * sum_tt - sum_t * sum_t;
        if (std::abs(denom) > 1e-10) {
            double b = (n * sum_ta - sum_t * sum_a) / denom;
            double a = (sum_a - b * sum_t) / n;
            if (std::abs(b) > 1e-6) {
                double t_launch = -a / b;
                // Sanity: launch time should be before first observation
                if (t_launch < obs[0].time) {
                    return t_launch;
                }
            }
        }
    }

    // Fallback: simple extrapolation from first two points
    double dt = obs[1].time - obs[0].time;
    if (dt > 0) {
        double alt0 = alts[0];
        double alt1 = alts[1];
        double rate = (alt1 - alt0) / dt;
        if (rate > 0) {
            return obs[0].time - alt0 / rate;
        }
    }

    return obs[0].time;
}

std::vector<double> AscentEstimator::compute_residuals(
        const AscentModelParams& params,
        const Tracklet& tracklet) const {

    TrajectorySimulator sim;
    const auto& obs = tracklet.observations;
    if (obs.empty()) return {};

    double t_end = obs.back().time - params.launch_time + 10.0;
    if (t_end < 1.0) t_end = 1.0;

    auto traj = sim.simulate(params, t_end, 0.5);
    if (traj.empty()) return std::vector<double>(obs.size(), 1e6);

    std::vector<double> residuals;

    for (const auto& ob : obs) {
        if (!ob.has_position) {
            residuals.push_back(0.0);
            continue;
        }

        // Find closest simulated point in time
        double obs_t = ob.time - params.launch_time;
        double best_dist = 1e12;

        for (const auto& sv : traj) {
            double dt = std::abs(sv.time - obs_t);
            if (dt > 5.0) continue;

            double dist = (sv.position - ob.position).norm();
            if (dist < best_dist) {
                best_dist = dist;
            }
        }

        // If no close point found, interpolate
        if (best_dist > 1e11) {
            // Find bracketing points
            for (size_t j = 1; j < traj.size(); ++j) {
                if (traj[j-1].time <= obs_t && traj[j].time >= obs_t) {
                    double frac = (obs_t - traj[j-1].time) / (traj[j].time - traj[j-1].time);
                    Vec3 interp = traj[j-1].position + (traj[j].position - traj[j-1].position) * frac;
                    best_dist = (interp - ob.position).norm();
                    break;
                }
            }
        }

        if (best_dist > 1e11) best_dist = 1e6;
        residuals.push_back(best_dist);
    }

    return residuals;
}

double AscentEstimator::rms_residual(const std::vector<double>& residuals) const {
    if (residuals.empty()) return 0.0;
    double sum_sq = 0.0;
    for (double r : residuals) {
        sum_sq += r * r;
    }
    return std::sqrt(sum_sq / residuals.size());
}

AscentModelParams AscentEstimator::update_params(
        const AscentModelParams& params,
        const Tracklet& tracklet) const {

    // Numerical Jacobian with Gauss-Newton update
    // Fit: launch_time, launch_azimuth, initial_pitch, and per-stage thrust

    // Collect parameter vector
    std::vector<double> p;
    p.push_back(params.launch_time);
    p.push_back(params.launch_azimuth);
    p.push_back(params.initial_pitch);
    for (const auto& s : params.stages) {
        p.push_back(s.thrust);
    }
    int np = (int)p.size();

    // Current residuals
    auto r0 = compute_residuals(params, tracklet);
    int nr = (int)r0.size();
    if (nr == 0) return params;

    // Perturbation sizes
    std::vector<double> dp(np);
    dp[0] = 0.5;   // launch_time (seconds)
    dp[1] = 0.001; // launch_azimuth (radians)
    dp[2] = 0.001; // initial_pitch (radians)
    for (int i = 3; i < np; ++i) {
        dp[i] = std::max(100.0, p[i] * 0.001); // thrust
    }

    // Compute Jacobian
    std::vector<std::vector<double>> J(nr, std::vector<double>(np, 0.0));

    for (int j = 0; j < np; ++j) {
        // Perturbed params
        AscentModelParams pp = params;
        double pert = dp[j];

        if (j == 0) pp.launch_time = params.launch_time + pert;
        else if (j == 1) pp.launch_azimuth = params.launch_azimuth + pert;
        else if (j == 2) pp.initial_pitch = params.initial_pitch + pert;
        else {
            int stage_idx = j - 3;
            if (stage_idx < (int)pp.stages.size()) {
                pp.stages[stage_idx].thrust += pert;
            }
        }

        auto rp = compute_residuals(pp, tracklet);

        for (int i = 0; i < nr; ++i) {
            J[i][j] = (rp[i] - r0[i]) / pert;
        }
    }

    // Compute J^T * J and J^T * r
    std::vector<std::vector<double>> JtJ(np, std::vector<double>(np, 0.0));
    std::vector<double> Jtr(np, 0.0);

    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            for (int k = 0; k < nr; ++k) {
                JtJ[i][j] += J[k][i] * J[k][j];
            }
        }
        for (int k = 0; k < nr; ++k) {
            Jtr[i] += J[k][i] * r0[k];
        }
    }

    // Add Levenberg-Marquardt damping
    double lambda = 1.0;
    for (int i = 0; i < np; ++i) {
        JtJ[i][i] += lambda * (JtJ[i][i] + 1.0);
    }

    // Solve (J^T J + lambda I) * delta = J^T r using simple Gaussian elimination
    // Augmented matrix
    std::vector<std::vector<double>> aug(np, std::vector<double>(np + 1, 0.0));
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            aug[i][j] = JtJ[i][j];
        }
        aug[i][np] = Jtr[i];
    }

    // Forward elimination
    for (int i = 0; i < np; ++i) {
        // Partial pivoting
        int max_row = i;
        for (int k = i + 1; k < np; ++k) {
            if (std::abs(aug[k][i]) > std::abs(aug[max_row][i])) max_row = k;
        }
        std::swap(aug[i], aug[max_row]);

        if (std::abs(aug[i][i]) < 1e-15) continue;

        for (int k = i + 1; k < np; ++k) {
            double factor = aug[k][i] / aug[i][i];
            for (int j = i; j <= np; ++j) {
                aug[k][j] -= factor * aug[i][j];
            }
        }
    }

    // Back substitution
    std::vector<double> delta(np, 0.0);
    for (int i = np - 1; i >= 0; --i) {
        if (std::abs(aug[i][i]) < 1e-15) continue;
        delta[i] = aug[i][np];
        for (int j = i + 1; j < np; ++j) {
            delta[i] -= aug[i][j] * delta[j];
        }
        delta[i] /= aug[i][i];
    }

    // Apply damped update
    double step_scale = 0.5; // conservative step
    AscentModelParams result = params;
    result.launch_time -= delta[0] * step_scale;
    result.launch_azimuth -= delta[1] * step_scale;
    result.initial_pitch -= delta[2] * step_scale;

    // Clamp initial_pitch to reasonable range
    if (result.initial_pitch < 30.0 * DEG2RAD) result.initial_pitch = 30.0 * DEG2RAD;
    if (result.initial_pitch > 89.0 * DEG2RAD) result.initial_pitch = 89.0 * DEG2RAD;

    for (int i = 3; i < np; ++i) {
        int stage_idx = i - 3;
        if (stage_idx < (int)result.stages.size()) {
            result.stages[stage_idx].thrust -= delta[i] * step_scale;
            // Keep thrust positive
            if (result.stages[stage_idx].thrust < 1000.0) {
                result.stages[stage_idx].thrust = 1000.0;
            }
        }
    }

    return result;
}

ReconstructionResult AscentEstimator::reconstruct(
        const Tracklet& tracklet,
        const AscentModelParams& initial_guess) const {

    ReconstructionResult result;
    result.converged = false;

    // Detect staging events
    result.staging_events = detect_staging(tracklet);

    // Estimate launch site
    result.estimated_launch_site = estimate_launch_site(tracklet);

    // Estimate launch time
    result.estimated_launch_time = estimate_launch_time(tracklet);

    // Set up initial parameters
    AscentModelParams params = initial_guess;
    params.launch_lat = result.estimated_launch_site.lat;
    params.launch_lon = result.estimated_launch_site.lon;
    params.launch_alt = result.estimated_launch_site.alt;
    if (params.launch_alt < 0) params.launch_alt = 0;
    params.launch_time = result.estimated_launch_time;

    // Iterative refinement
    double prev_rms = 1e12;
    for (int iter = 0; iter < max_iter_; ++iter) {
        auto residuals = compute_residuals(params, tracklet);
        double rms = rms_residual(residuals);

        result.iterations = iter + 1;
        result.rms_residual = rms;

        if (rms < tol_) {
            result.converged = true;
            break;
        }

        if (std::abs(prev_rms - rms) < 1.0) {
            // Converged (not improving)
            if (rms < tol_ * 10.0) {
                result.converged = true;
            }
            break;
        }

        prev_rms = rms;
        params = update_params(params, tracklet);
    }

    // Final simulation with converged parameters
    TrajectorySimulator sim;
    double t_end = 0;
    for (const auto& s : params.stages) {
        t_end += s.burn_time;
    }
    t_end += 10.0;

    result.trajectory = sim.simulate(params, t_end, 0.5);
    result.fitted_params = params;

    // Set launch azimuth in degrees
    result.launch_azimuth_deg = params.launch_azimuth * RAD2DEG;

    // Set burnout state
    if (!result.trajectory.empty()) {
        result.burnout_state = result.trajectory.back();
    }

    // Detect staging events from reconstructed trajectory
    if (result.staging_events.empty() && params.stages.size() > 1) {
        double t_accum = 0;
        for (size_t i = 0; i + 1 < params.stages.size(); ++i) {
            t_accum += params.stages[i].burn_time;

            StagingEvent ev;
            ev.time = t_accum;
            ev.from_stage = (int)i + 1;
            ev.to_stage = (int)i + 2;

            // Find trajectory point at staging time
            for (const auto& sv : result.trajectory) {
                if (std::abs(sv.time - t_accum) < 1.0) {
                    ev.altitude_m = sv.altitude_m;
                    ev.speed_mps = sv.speed_mps;
                    ev.position = sv.position;
                    break;
                }
            }
            ev.delta_accel = 0;
            result.staging_events.push_back(ev);
        }
    }

    return result;
}

Tracklet AscentEstimator::generate_synthetic_tracklet(
        const AscentModelParams& params,
        const LatLonAlt& sensor_loc,
        double t_start, double t_end,
        double dt, double noise_m) {

    TrajectorySimulator sim;
    auto trajectory = sim.simulate(params, t_end + 10.0, 0.5);

    Tracklet tracklet;
    tracklet.tracklet_id = "synthetic";
    tracklet.sensor_id = "test_sensor";
    tracklet.sensor_location = sensor_loc;
    tracklet.epoch_unix = 0;

    std::mt19937 rng(42); // deterministic seed
    std::normal_distribution<double> noise_dist(0.0, noise_m);

    for (double t = t_start; t <= t_end; t += dt) {
        // Find closest trajectory point
        Vec3 interp_pos = {0, 0, 0};
        bool found = false;

        for (size_t i = 1; i < trajectory.size(); ++i) {
            if (trajectory[i-1].time <= t && trajectory[i].time >= t) {
                double frac = 0;
                double dt_seg = trajectory[i].time - trajectory[i-1].time;
                if (dt_seg > 0) {
                    frac = (t - trajectory[i-1].time) / dt_seg;
                }
                interp_pos = trajectory[i-1].position +
                    (trajectory[i].position - trajectory[i-1].position) * frac;
                found = true;
                break;
            }
        }

        if (!found) continue;

        // Add noise
        Observation obs;
        obs.time = t;
        obs.position = {
            interp_pos.x + noise_dist(rng),
            interp_pos.y + noise_dist(rng),
            interp_pos.z + noise_dist(rng)
        };
        obs.has_position = true;
        obs.weight = 1.0;

        tracklet.observations.push_back(obs);
    }

    return tracklet;
}

} // namespace ascent_reconstruct
