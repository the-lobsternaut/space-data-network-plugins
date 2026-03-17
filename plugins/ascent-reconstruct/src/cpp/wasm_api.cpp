#include "ascent_reconstruct/estimator.h"
#include "ascent_reconstruct/trajectory.h"
#include "ascent_reconstruct/types.h"

#include <emscripten/emscripten.h>
#include <emscripten/bind.h>

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdlib>

using namespace ascent_reconstruct;

// Global instances
static TrajectorySimulator g_simulator;
static AscentEstimator g_estimator;

// ─── Helpers ────────────────────────────────────────────────────────────────

static std::vector<double> parse_csv(const std::string& csv) {
    std::vector<double> vals;
    std::istringstream ss(csv);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) {
            vals.push_back(std::stod(tok));
        }
    }
    return vals;
}

static Tracklet make_tracklet(double slat, double slon, double salt,
                               const std::vector<double>& times,
                               const std::vector<double>& xs,
                               const std::vector<double>& ys,
                               const std::vector<double>& zs) {
    Tracklet t;
    t.tracklet_id = "wasm";
    t.sensor_id = "wasm_sensor";
    t.sensor_location = {slat, slon, salt};
    t.epoch_unix = static_cast<int64_t>(times.empty() ? 0 : times[0]);
    size_t n = std::min({times.size(), xs.size(), ys.size(), zs.size()});
    for (size_t i = 0; i < n; ++i) {
        Observation obs;
        obs.time = times[i];
        obs.position = {xs[i], ys[i], zs[i]};
        obs.has_position = true;
        t.observations.push_back(obs);
    }
    return t;
}

static std::string state_vector_to_json(const StateVector& sv) {
    std::ostringstream json;
    json << "{\"time\":" << sv.time
         << ",\"x\":" << sv.position.x
         << ",\"y\":" << sv.position.y
         << ",\"z\":" << sv.position.z
         << ",\"vx\":" << sv.velocity.x
         << ",\"vy\":" << sv.velocity.y
         << ",\"vz\":" << sv.velocity.z
         << ",\"alt_m\":" << sv.altitude_m
         << ",\"speed_mps\":" << sv.speed_mps
         << ",\"stage\":" << sv.stage << "}";
    return json.str();
}

static std::string trajectory_to_json(const std::vector<StateVector>& traj) {
    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i > 0) json << ",";
        json << state_vector_to_json(traj[i]);
    }
    json << "]";
    return json.str();
}

// ─── WASM API Functions ─────────────────────────────────────────────────────

// Simulate single-stage ascent trajectory
std::string wasm_simulate_ascent(double launch_lat, double launch_lon, double launch_alt,
                                  double launch_azimuth_deg, double launch_pitch_deg,
                                  double stage1_thrust, double stage1_isp,
                                  double stage1_mass_init, double stage1_mass_final,
                                  double stage1_burn_time,
                                  double t_end, double dt) {
    AscentModelParams params;
    params.launch_lat = launch_lat;
    params.launch_lon = launch_lon;
    params.launch_alt = launch_alt;
    params.launch_azimuth = launch_azimuth_deg * DEG2RAD;
    params.initial_pitch = launch_pitch_deg * DEG2RAD;

    AscentModelParams::StageParams s1;
    s1.thrust = stage1_thrust;
    s1.isp = stage1_isp;
    s1.mass_initial = stage1_mass_init;
    s1.mass_final = stage1_mass_final;
    s1.burn_time = stage1_burn_time;
    s1.pitch_rate = 0.0;
    params.stages.push_back(s1);

    auto traj = g_simulator.simulate(params, t_end, dt);
    return trajectory_to_json(traj);
}

// Simulate two-stage ascent trajectory
std::string wasm_simulate_two_stage(double launch_lat, double launch_lon, double launch_alt,
                                     double launch_azimuth_deg, double launch_pitch_deg,
                                     double s1_thrust, double s1_isp,
                                     double s1_mass_init, double s1_mass_final, double s1_burn,
                                     double s2_thrust, double s2_isp,
                                     double s2_mass_init, double s2_mass_final, double s2_burn,
                                     double t_end, double dt) {
    AscentModelParams params;
    params.launch_lat = launch_lat;
    params.launch_lon = launch_lon;
    params.launch_alt = launch_alt;
    params.launch_azimuth = launch_azimuth_deg * DEG2RAD;
    params.initial_pitch = launch_pitch_deg * DEG2RAD;

    AscentModelParams::StageParams stage1;
    stage1.thrust = s1_thrust;
    stage1.isp = s1_isp;
    stage1.mass_initial = s1_mass_init;
    stage1.mass_final = s1_mass_final;
    stage1.burn_time = s1_burn;
    stage1.pitch_rate = 0.0;
    params.stages.push_back(stage1);

    AscentModelParams::StageParams stage2;
    stage2.thrust = s2_thrust;
    stage2.isp = s2_isp;
    stage2.mass_initial = s2_mass_init;
    stage2.mass_final = s2_mass_final;
    stage2.burn_time = s2_burn;
    stage2.pitch_rate = 0.0;
    params.stages.push_back(stage2);

    auto traj = g_simulator.simulate(params, t_end, dt);
    return trajectory_to_json(traj);
}

// Reconstruct trajectory from tracklet observations
std::string wasm_reconstruct(double sensor_lat, double sensor_lon, double sensor_alt,
                              const std::string& obs_times_csv,
                              const std::string& obs_x_csv,
                              const std::string& obs_y_csv,
                              const std::string& obs_z_csv,
                              double guess_lat, double guess_lon, double guess_az_deg) {
    auto times = parse_csv(obs_times_csv);
    auto xs = parse_csv(obs_x_csv);
    auto ys = parse_csv(obs_y_csv);
    auto zs = parse_csv(obs_z_csv);

    Tracklet tracklet = make_tracklet(sensor_lat, sensor_lon, sensor_alt, times, xs, ys, zs);

    AscentModelParams initial_guess;
    initial_guess.launch_lat = guess_lat;
    initial_guess.launch_lon = guess_lon;
    initial_guess.launch_alt = 0.0;
    initial_guess.launch_azimuth = guess_az_deg * DEG2RAD;
    initial_guess.initial_pitch = 85.0 * DEG2RAD;

    // Add a default single stage as initial guess
    AscentModelParams::StageParams s1;
    s1.thrust = 500000.0;
    s1.isp = 280.0;
    s1.mass_initial = 50000.0;
    s1.mass_final = 10000.0;
    s1.burn_time = 120.0;
    s1.pitch_rate = 0.0;
    initial_guess.stages.push_back(s1);

    auto result = g_estimator.reconstruct(tracklet, initial_guess);

    std::ostringstream json;
    json << "{\"converged\":" << (result.converged ? "true" : "false")
         << ",\"iterations\":" << result.iterations
         << ",\"rms_residual\":" << result.rms_residual
         << ",\"estimated_launch_lat\":" << result.estimated_launch_site.lat
         << ",\"estimated_launch_lon\":" << result.estimated_launch_site.lon
         << ",\"estimated_launch_time\":" << result.estimated_launch_time
         << ",\"launch_azimuth_deg\":" << result.launch_azimuth_deg
         << ",\"trajectory\":" << trajectory_to_json(result.trajectory)
         << "}";
    return json.str();
}

// Detect staging events from observations
std::string wasm_detect_staging(double sensor_lat, double sensor_lon, double sensor_alt,
                                 const std::string& obs_times_csv,
                                 const std::string& obs_x_csv,
                                 const std::string& obs_y_csv,
                                 const std::string& obs_z_csv) {
    auto times = parse_csv(obs_times_csv);
    auto xs = parse_csv(obs_x_csv);
    auto ys = parse_csv(obs_y_csv);
    auto zs = parse_csv(obs_z_csv);

    Tracklet tracklet = make_tracklet(sensor_lat, sensor_lon, sensor_alt, times, xs, ys, zs);
    auto events = g_estimator.detect_staging(tracklet);

    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < events.size(); ++i) {
        if (i > 0) json << ",";
        json << "{\"time\":" << events[i].time
             << ",\"from_stage\":" << events[i].from_stage
             << ",\"to_stage\":" << events[i].to_stage
             << ",\"altitude_m\":" << events[i].altitude_m
             << ",\"speed_mps\":" << events[i].speed_mps
             << ",\"delta_accel\":" << events[i].delta_accel << "}";
    }
    json << "]";
    return json.str();
}

// Estimate launch site from observations
std::string wasm_estimate_launch_site(double sensor_lat, double sensor_lon, double sensor_alt,
                                       const std::string& obs_times_csv,
                                       const std::string& obs_x_csv,
                                       const std::string& obs_y_csv,
                                       const std::string& obs_z_csv) {
    auto times = parse_csv(obs_times_csv);
    auto xs = parse_csv(obs_x_csv);
    auto ys = parse_csv(obs_y_csv);
    auto zs = parse_csv(obs_z_csv);

    Tracklet tracklet = make_tracklet(sensor_lat, sensor_lon, sensor_alt, times, xs, ys, zs);
    auto site = g_estimator.estimate_launch_site(tracklet);

    std::ostringstream json;
    json << "{\"lat\":" << site.lat << ",\"lon\":" << site.lon << ",\"alt\":" << site.alt << "}";
    return json.str();
}

// Estimate launch time from observations
double wasm_estimate_launch_time(double sensor_lat, double sensor_lon, double sensor_alt,
                                  const std::string& obs_times_csv,
                                  const std::string& obs_x_csv,
                                  const std::string& obs_y_csv,
                                  const std::string& obs_z_csv) {
    auto times = parse_csv(obs_times_csv);
    auto xs = parse_csv(obs_x_csv);
    auto ys = parse_csv(obs_y_csv);
    auto zs = parse_csv(obs_z_csv);

    Tracklet tracklet = make_tracklet(sensor_lat, sensor_lon, sensor_alt, times, xs, ys, zs);
    return g_estimator.estimate_launch_time(tracklet);
}

// Generate synthetic tracklet for testing
std::string wasm_generate_synthetic_tracklet(double launch_lat, double launch_lon, double launch_alt,
                                              double launch_azimuth_deg,
                                              double sensor_lat, double sensor_lon, double sensor_alt,
                                              double t_start, double t_end, double dt,
                                              double noise_m) {
    AscentModelParams params;
    params.launch_lat = launch_lat;
    params.launch_lon = launch_lon;
    params.launch_alt = launch_alt;
    params.launch_azimuth = launch_azimuth_deg * DEG2RAD;
    params.initial_pitch = 85.0 * DEG2RAD;

    // Default single stage for synthetic data generation
    AscentModelParams::StageParams s1;
    s1.thrust = 500000.0;
    s1.isp = 280.0;
    s1.mass_initial = 50000.0;
    s1.mass_final = 10000.0;
    s1.burn_time = 120.0;
    s1.pitch_rate = 0.0;
    params.stages.push_back(s1);

    LatLonAlt sensor_loc = {sensor_lat, sensor_lon, sensor_alt};
    auto tracklet = AscentEstimator::generate_synthetic_tracklet(params, sensor_loc,
                                                                  t_start, t_end, dt, noise_m);

    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < tracklet.observations.size(); ++i) {
        if (i > 0) json << ",";
        auto& obs = tracklet.observations[i];
        json << "{\"time\":" << obs.time
             << ",\"x\":" << obs.position.x
             << ",\"y\":" << obs.position.y
             << ",\"z\":" << obs.position.z << "}";
    }
    json << "]";
    return json.str();
}

// Compute gravity acceleration at a position
std::string wasm_gravity(double x, double y, double z) {
    Vec3 pos = {x, y, z};
    Vec3 g = TrajectorySimulator::gravity(pos);
    std::ostringstream json;
    json << "{\"ax\":" << g.x << ",\"ay\":" << g.y << ",\"az\":" << g.z << "}";
    return json.str();
}

// Convert geodetic coordinates to ECEF
std::string wasm_geodetic_to_ecef(double lat, double lon, double alt) {
    auto pos = TrajectorySimulator::geodetic_to_ecef(lat, lon, alt);
    std::ostringstream json;
    json << "{\"x\":" << pos.x << ",\"y\":" << pos.y << ",\"z\":" << pos.z << "}";
    return json.str();
}

// Convert ECEF to geodetic
std::string wasm_ecef_to_geodetic(double x, double y, double z) {
    auto lla = TrajectorySimulator::ecef_to_geodetic({x, y, z});
    std::ostringstream json;
    json << "{\"lat\":" << lla.lat << ",\"lon\":" << lla.lon << ",\"alt\":" << lla.alt << "}";
    return json.str();
}

// Get atmospheric density at altitude
double wasm_atm_density(double alt_m) {
    return TrajectorySimulator::atm_density(alt_m);
}

// Emscripten bindings
EMSCRIPTEN_BINDINGS(ascent_reconstruct_wasm) {
    emscripten::function("simulateAscent",            &wasm_simulate_ascent);
    emscripten::function("simulateTwoStage",          &wasm_simulate_two_stage);
    emscripten::function("reconstruct",               &wasm_reconstruct);
    emscripten::function("detectStaging",             &wasm_detect_staging);
    emscripten::function("estimateLaunchSite",        &wasm_estimate_launch_site);
    emscripten::function("estimateLaunchTime",        &wasm_estimate_launch_time);
    emscripten::function("generateSyntheticTracklet", &wasm_generate_synthetic_tracklet);
    emscripten::function("gravity",                   &wasm_gravity);
    emscripten::function("geodeticToEcef",            &wasm_geodetic_to_ecef);
    emscripten::function("ecefToGeodetic",            &wasm_ecef_to_geodetic);
    emscripten::function("atmDensity",                &wasm_atm_density);
}
