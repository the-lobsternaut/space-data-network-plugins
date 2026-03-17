#include "da_pipeline/ascent_estimator.h"
#include "da_pipeline/trajectory.h"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// default_vehicle — Falcon-9-like 2-stage model
// ---------------------------------------------------------------------------
AscentModelParams AscentEstimator::default_vehicle(double lat, double lon, double azimuth_deg) {
    AscentModelParams p;
    p.launch_lat = lat;
    p.launch_lon = lon;
    p.launch_alt = 0;
    p.launch_time = 0;
    p.launch_azimuth = azimuth_deg;
    p.initial_pitch = 90.0; // degrees from horizontal (vertical)

    // Stage 1: Merlin-1D cluster analog
    // mass_initial = dry mass of this stage + propellant (not including upper stages)
    AscentModelParams::StageParams s1;
    s1.thrust = 7607000.0;     // N (~7607 kN, 9 Merlin-1D)
    s1.isp = 282.0;            // s (sea level)
    s1.mass_initial = 421300.0; // kg (stage 1 structure + propellant)
    s1.mass_final = 25600.0;   // kg (stage 1 dry mass)
    s1.burn_time = 162.0;      // s
    s1.pitch_rate = 0.3;       // deg/s gravity turn rate

    // Stage 2: Merlin Vacuum analog
    AscentModelParams::StageParams s2;
    s2.thrust = 934000.0;      // N (~934 kN)
    s2.isp = 348.0;            // s (vacuum)
    s2.mass_initial = 111500.0; // kg (stage 2 structure + propellant + payload)
    s2.mass_final = 18850.0;   // kg (stage 2 dry + payload ~16t)
    s2.burn_time = 397.0;      // s
    s2.pitch_rate = 0.1;

    p.stages = {s1, s2};
    return p;
}

// ---------------------------------------------------------------------------
// estimate_launch_site — extrapolate from early observations
// ---------------------------------------------------------------------------
LatLonAlt AscentEstimator::estimate_launch_site(const Tracklet& tracklet) const {
    if (tracklet.observations.empty()) {
        return {0, 0, 0};
    }

    // Use the earliest observations with position data
    std::vector<const Observation*> pos_obs;
    for (auto& obs : tracklet.observations) {
        if (obs.has_position) pos_obs.push_back(&obs);
    }

    if (pos_obs.size() < 2) {
        // Fallback: use sensor location
        return tracklet.sensor_location;
    }

    // Linear regression on altitude vs time, extrapolate to alt=0
    double t0 = pos_obs[0]->time;
    double t1 = pos_obs[1]->time;
    double dt = t1 - t0;
    if (std::abs(dt) < 1e-6) return tracklet.sensor_location;

    // Compute altitudes
    auto alt_from_pos = [](const Vec3& p) {
        return p.norm() - R_EARTH_M;
    };

    double a0 = alt_from_pos(pos_obs[0]->position);
    double a1 = alt_from_pos(pos_obs[1]->position);
    double da_dt = (a1 - a0) / dt;

    if (da_dt <= 0) {
        // Not ascending — use first position projected to ground
        Vec3 ground = pos_obs[0]->position.normalized() * R_EARTH_M;
        auto lla = TrajectorySimulator::ecef_to_geodetic(ground);
        return lla;
    }

    // Time to ground: t_ground = t0 - a0 / da_dt
    double t_ground = t0 - a0 / da_dt;

    // Extrapolate position back
    Vec3 v_avg = (pos_obs[1]->position - pos_obs[0]->position) * (1.0 / dt);
    Vec3 pos_ground = pos_obs[0]->position + v_avg * (t_ground - t0);
    pos_ground = pos_ground.normalized() * R_EARTH_M;

    return TrajectorySimulator::ecef_to_geodetic(pos_ground);
}

// ---------------------------------------------------------------------------
// estimate_launch_time
// ---------------------------------------------------------------------------
double AscentEstimator::estimate_launch_time(const Tracklet& tracklet) const {
    if (tracklet.observations.empty()) return 0;

    // Find observations with position
    std::vector<const Observation*> pos_obs;
    for (auto& obs : tracklet.observations) {
        if (obs.has_position) pos_obs.push_back(&obs);
    }

    if (pos_obs.size() < 2) {
        return tracklet.observations[0].time - 60.0; // default 60s before first obs
    }

    // Linear extrapolation of altitude to zero
    double t0 = pos_obs[0]->time;
    double t1 = pos_obs[1]->time;
    double a0 = pos_obs[0]->position.norm() - R_EARTH_M;
    double a1 = pos_obs[1]->position.norm() - R_EARTH_M;

    double da_dt = (a1 - a0) / (t1 - t0);
    if (da_dt <= 0) return t0 - 60.0;

    return t0 - a0 / da_dt;
}

// ---------------------------------------------------------------------------
// detect_staging — look for acceleration discontinuities
// ---------------------------------------------------------------------------
std::vector<StagingEvent> AscentEstimator::detect_staging(const Tracklet& tracklet) const {
    std::vector<StagingEvent> events;

    if (tracklet.observations.size() < 4) return events;

    // Compute speeds from position differences
    std::vector<double> times, speeds;
    for (size_t i = 1; i < tracklet.observations.size(); i++) {
        auto& o0 = tracklet.observations[i - 1];
        auto& o1 = tracklet.observations[i];
        if (!o0.has_position || !o1.has_position) continue;

        double dt = o1.time - o0.time;
        if (dt < 0.01) continue;

        double ds = (o1.position - o0.position).norm();
        times.push_back((o0.time + o1.time) / 2.0);
        speeds.push_back(ds / dt);
    }

    if (speeds.size() < 3) return events;

    // Compute accelerations
    std::vector<double> accels;
    std::vector<double> accel_times;
    for (size_t i = 1; i < speeds.size(); i++) {
        double dt = times[i] - times[i - 1];
        if (dt < 0.01) continue;
        accels.push_back((speeds[i] - speeds[i - 1]) / dt);
        accel_times.push_back((times[i] + times[i - 1]) / 2.0);
    }

    // Look for sudden acceleration changes (staging)
    for (size_t i = 1; i < accels.size(); i++) {
        double da = accels[i] - accels[i - 1];
        // Significant acceleration jump suggests staging
        if (std::abs(da) > 5.0) { // 5 m/s^2 threshold
            StagingEvent ev;
            ev.time = accel_times[i];
            ev.delta_accel = da;
            // Determine stage numbers from order
            ev.from_stage = static_cast<int>(events.size());
            ev.to_stage = ev.from_stage + 1;

            // Estimate altitude and speed at staging
            // Find nearest observation
            double min_dt = 1e9;
            for (auto& obs : tracklet.observations) {
                if (obs.has_position && std::abs(obs.time - ev.time) < min_dt) {
                    min_dt = std::abs(obs.time - ev.time);
                    ev.position = obs.position;
                    ev.altitude_m = obs.position.norm() - R_EARTH_M;
                }
            }
            // Speed from local speeds array
            size_t closest = 0;
            min_dt = 1e9;
            for (size_t j = 0; j < times.size(); j++) {
                if (std::abs(times[j] - ev.time) < min_dt) {
                    min_dt = std::abs(times[j] - ev.time);
                    closest = j;
                }
            }
            ev.speed_mps = speeds[closest];

            events.push_back(ev);
        }
    }

    return events;
}

// ---------------------------------------------------------------------------
// reconstruct — iterative trajectory fitting
// ---------------------------------------------------------------------------
ReconstructionResult AscentEstimator::reconstruct(
    const Tracklet& tracklet, const AscentModelParams& guess) const
{
    ReconstructionResult result;
    result.fitted_params = guess;

    if (tracklet.observations.empty()) {
        result.converged = false;
        return result;
    }

    // Estimate launch site and time from tracklet
    result.estimated_launch_site = estimate_launch_site(tracklet);
    result.estimated_launch_time = estimate_launch_time(tracklet);
    result.staging_events = detect_staging(tracklet);

    // Set up parameters from estimates
    AscentModelParams params = guess;
    params.launch_lat = result.estimated_launch_site.lat;
    params.launch_lon = result.estimated_launch_site.lon;
    params.launch_alt = result.estimated_launch_site.alt;
    params.launch_time = result.estimated_launch_time;

    // Determine simulation end time
    double t_last = tracklet.observations.back().time;
    double t_end = t_last - result.estimated_launch_time + 10.0;
    if (t_end < 60) t_end = 600; // Default 10 min

    // Simple Gauss-Newton-like iteration
    // We vary azimuth and pitch to minimize position residuals
    double best_rms = 1e18;
    double best_az = params.launch_azimuth;
    double best_pitch = params.initial_pitch;

    TrajectorySimulator sim;

    // Grid search over azimuth and pitch
    for (int az_step = -5; az_step <= 5; az_step += 1) {
        double az_try = guess.launch_azimuth + az_step * 2.0; // 2-deg steps

        AscentModelParams trial = params;
        trial.launch_azimuth = az_try;

        auto traj = sim.simulate(trial, t_end, 1.0);
        if (traj.empty()) continue;

        // Compute residuals against observations
        double sum_sq = 0;
        int count = 0;
        for (auto& obs : tracklet.observations) {
            if (!obs.has_position) continue;
            double obs_t = obs.time - result.estimated_launch_time;
            if (obs_t < 0) continue;

            // Find nearest trajectory point
            double min_dt_abs = 1e9;
            Vec3 traj_pos;
            for (auto& sv : traj) {
                if (std::abs(sv.time - obs_t) < min_dt_abs) {
                    min_dt_abs = std::abs(sv.time - obs_t);
                    traj_pos = sv.position;
                }
            }

            double residual = (obs.position - traj_pos).norm();
            sum_sq += residual * residual;
            count++;
        }

        if (count > 0) {
            double rms = std::sqrt(sum_sq / count);
            if (rms < best_rms) {
                best_rms = rms;
                best_az = az_try;
                result.trajectory = traj;
            }
        }
    }

    result.rms_residual = best_rms;
    result.launch_azimuth_deg = best_az;
    result.converged = (best_rms < 50000.0); // 50 km threshold for convergence
    result.iterations = 11; // 11 azimuth steps tried

    // Set fitted params
    result.fitted_params = params;
    result.fitted_params.launch_azimuth = best_az;

    // Set burnout state if trajectory exists
    if (!result.trajectory.empty()) {
        result.burnout_state = result.trajectory.back();
    }

    return result;
}

// ---------------------------------------------------------------------------
// generate_family — multiple trajectories with dispersions
// ---------------------------------------------------------------------------
AscentTrajectoryFamily AscentEstimator::generate_family(
    const LaunchSite& site, double azimuth_deg, double azimuth_spread_deg) const
{
    AscentTrajectoryFamily family;
    family.family_id = "FAM-" + site.site_id + "-" + std::to_string(static_cast<int>(azimuth_deg));
    family.min_azimuth_deg = azimuth_deg - azimuth_spread_deg;
    family.max_azimuth_deg = azimuth_deg + azimuth_spread_deg;

    // Estimated inclination from launch azimuth and latitude
    double lat_r = site.lat * DEG2RAD;
    double az_r = azimuth_deg * DEG2RAD;
    double cos_i = std::sin(az_r) * std::cos(lat_r);
    family.estimated_inclination_deg = std::acos(std::abs(cos_i)) * RAD2DEG;

    TrajectorySimulator sim;

    // Nominal trajectory
    AscentModelParams nominal = default_vehicle(site.lat, site.lon, azimuth_deg);
    family.nominal_trajectory = sim.simulate(nominal, 600.0, 1.0);

    // Dispersions: vary azimuth in steps
    int num_dispersions = 5;
    double step = azimuth_spread_deg * 2.0 / (num_dispersions - 1);
    for (int i = 0; i < num_dispersions; i++) {
        double az = family.min_azimuth_deg + i * step;
        AscentModelParams disp = default_vehicle(site.lat, site.lon, az);
        auto traj = sim.simulate(disp, 600.0, 2.0);
        family.dispersions.push_back(traj);
    }

    return family;
}

} // namespace da_pipeline
