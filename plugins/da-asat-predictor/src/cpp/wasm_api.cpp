#include "da_asat/propagator.h"
#include "da_asat/interceptor.h"
#include "da_asat/engagement.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#include <emscripten/bind.h>
#endif

#include <string>
#include <sstream>
#include <vector>

using namespace da_asat;

// ─── Global instances ───────────────────────────────────────────────────────

static EngagementCalculator g_engagement;
static Interceptor          g_interceptor;

// ─── WASM API Functions ─────────────────────────────────────────────────────

// Parse a TLE and return JSON with orbital elements
std::string wasm_parse_tle(const std::string& name,
                            const std::string& line1,
                            const std::string& line2) {
    auto result = Propagator::parse_tle(name, line1, line2);
    if (!result.has_value()) {
        return R"({"error": "Failed to parse TLE"})";
    }

    auto& tle = result.value();
    std::ostringstream json;
    json << "{";
    json << "\"norad_id\":" << tle.norad_id << ",";
    json << "\"name\":\"" << tle.name << "\",";
    json << "\"inclination\":" << tle.inclination << ",";
    json << "\"raan\":" << tle.raan << ",";
    json << "\"eccentricity\":" << tle.eccentricity << ",";
    json << "\"arg_perigee\":" << tle.arg_perigee << ",";
    json << "\"mean_anomaly\":" << tle.mean_anomaly << ",";
    json << "\"mean_motion\":" << tle.mean_motion << ",";
    json << "\"bstar\":" << tle.bstar << ",";
    json << "\"period_sec\":" << Propagator::orbital_period_sec(tle);
    json << "}";
    return json.str();
}

// Propagate a TLE to a given time and return state vector as JSON
std::string wasm_propagate(const std::string& name,
                            const std::string& line1,
                            const std::string& line2,
                            int64_t time_unix) {
    auto tle_opt = Propagator::parse_tle(name, line1, line2);
    if (!tle_opt.has_value()) {
        return R"({"error": "Failed to parse TLE"})";
    }

    auto state = Propagator::propagate(*tle_opt, static_cast<Timestamp>(time_unix));
    if (!state.has_value()) {
        return R"({"error": "Propagation failed"})";
    }

    auto geo = Propagator::state_to_geodetic(*state);

    std::ostringstream json;
    json << "{";
    json << "\"position\":{\"x\":" << state->position.x
         << ",\"y\":" << state->position.y
         << ",\"z\":" << state->position.z << "},";
    json << "\"velocity\":{\"x\":" << state->velocity.x
         << ",\"y\":" << state->velocity.y
         << ",\"z\":" << state->velocity.z << "},";
    json << "\"geodetic\":{\"lat\":" << geo.lat
         << ",\"lon\":" << geo.lon
         << ",\"alt_km\":" << geo.alt_km << "}";
    json << "}";
    return json.str();
}

// Evaluate a single engagement and return JSON result
std::string wasm_evaluate_engagement(const std::string& tle_name,
                                      const std::string& tle_line1,
                                      const std::string& tle_line2,
                                      const std::string& site_id,
                                      const std::string& site_name,
                                      double site_lat,
                                      double site_lon,
                                      double site_alt_km,
                                      int64_t launch_time_unix) {
    auto tle_opt = Propagator::parse_tle(tle_name, tle_line1, tle_line2);
    if (!tle_opt.has_value()) {
        return R"({"error": "Failed to parse TLE"})";
    }

    LaunchSite site;
    site.site_id = site_id;
    site.name = site_name;
    site.lat = site_lat;
    site.lon = site_lon;
    site.altitude_km = site_alt_km;

    auto params = Interceptor::default_interceptor();
    auto result = g_engagement.evaluate_single(site, *tle_opt, params,
                                                static_cast<Timestamp>(launch_time_unix));

    std::ostringstream json;
    json << "{";
    json << "\"target_norad_id\":" << result.target_norad_id << ",";
    json << "\"target_name\":\"" << result.target_name << "\",";
    json << "\"feasibility\":\"";
    switch (result.feasibility) {
        case Feasibility::FEASIBLE:            json << "FEASIBLE"; break;
        case Feasibility::INSUFFICIENT_ENERGY: json << "INSUFFICIENT_ENERGY"; break;
        case Feasibility::GEOMETRY_INFEASIBLE: json << "GEOMETRY_INFEASIBLE"; break;
        case Feasibility::TIMING_INFEASIBLE:   json << "TIMING_INFEASIBLE"; break;
        case Feasibility::ALTITUDE_TOO_LOW:    json << "ALTITUDE_TOO_LOW"; break;
        case Feasibility::ALTITUDE_TOO_HIGH:   json << "ALTITUDE_TOO_HIGH"; break;
    }
    json << "\",";
    json << "\"launch_azimuth_deg\":" << result.launch_azimuth_deg << ",";
    json << "\"launch_elevation_deg\":" << result.launch_elevation_deg << ",";
    json << "\"total_delta_v_kms\":" << result.total_delta_v_kms << ",";
    json << "\"time_of_flight_sec\":" << result.time_of_flight_sec << ",";
    json << "\"intercept_altitude_km\":" << result.intercept.altitude_km << ",";
    json << "\"intercept_range_km\":" << result.intercept.range_km << ",";
    json << "\"closing_velocity_kms\":" << result.intercept.closing_velocity_kms << ",";
    json << "\"confidence\":" << result.confidence;
    json << "}";
    return json.str();
}

// Get default interceptor parameters as JSON
std::string wasm_get_default_interceptor() {
    auto params = Interceptor::default_interceptor();
    std::ostringstream json;
    json << "{";
    json << "\"vehicle_name\":\"" << params.vehicle_name << "\",";
    json << "\"num_stages\":" << params.num_stages << ",";
    json << "\"kill_vehicle_mass_kg\":" << params.kill_vehicle_mass_kg << ",";
    json << "\"max_lateral_accel_g\":" << params.max_lateral_accel_g;
    json << "}";
    return json.str();
}

// Compute engagement zone boundary as JSON array of lat/lon points
std::string wasm_engagement_zone_boundary(double site_lat, double site_lon,
                                           double site_alt_km,
                                           double target_altitude_km) {
    LaunchSite site;
    site.site_id = "wasm";
    site.name = "WASM Site";
    site.lat = site_lat;
    site.lon = site_lon;
    site.altitude_km = site_alt_km;

    auto params = Interceptor::default_interceptor();
    auto boundary = g_engagement.compute_engagement_zone_boundary(site, params,
                                                                    target_altitude_km);

    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < boundary.size(); ++i) {
        if (i > 0) json << ",";
        json << "{\"lat\":" << boundary[i].lat
             << ",\"lon\":" << boundary[i].lon << "}";
    }
    json << "]";
    return json.str();
}

// ─── Helper: split string by delimiter ───────────────────────────────────────

static std::vector<std::string> split_lines(const std::string& s) {
    std::vector<std::string> lines;
    std::istringstream ss(s);
    std::string line;
    while (std::getline(ss, line)) {
        // Trim trailing CR
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

static std::string feasibility_to_string(Feasibility f) {
    switch (f) {
        case Feasibility::FEASIBLE:            return "FEASIBLE";
        case Feasibility::INSUFFICIENT_ENERGY: return "INSUFFICIENT_ENERGY";
        case Feasibility::GEOMETRY_INFEASIBLE: return "GEOMETRY_INFEASIBLE";
        case Feasibility::TIMING_INFEASIBLE:   return "TIMING_INFEASIBLE";
        case Feasibility::ALTITUDE_TOO_LOW:    return "ALTITUDE_TOO_LOW";
        case Feasibility::ALTITUDE_TOO_HIGH:   return "ALTITUDE_TOO_HIGH";
    }
    return "UNKNOWN";
}

// ─── New WASM API Functions ──────────────────────────────────────────────────

std::string wasm_find_engagement_windows(const std::string& tle_name,
                                          const std::string& tle_line1,
                                          const std::string& tle_line2,
                                          double site_lat, double site_lon,
                                          double site_alt_km,
                                          int64_t window_start, int64_t window_end) {
    auto tle_opt = Propagator::parse_tle(tle_name, tle_line1, tle_line2);
    if (!tle_opt.has_value()) {
        return R"({"error": "Failed to parse TLE"})";
    }

    LaunchSite site;
    site.site_id = "wasm";
    site.name = "WASM Site";
    site.lat = site_lat;
    site.lon = site_lon;
    site.altitude_km = site_alt_km;

    auto params = Interceptor::default_interceptor();
    auto windows = g_engagement.find_engagement_windows(site, *tle_opt, params,
                                                         static_cast<Timestamp>(window_start),
                                                         static_cast<Timestamp>(window_end));

    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < windows.size(); ++i) {
        if (i > 0) json << ",";
        json << "{\"window_start\":" << windows[i].window_start
             << ",\"window_end\":" << windows[i].window_end
             << ",\"best_launch_azimuth_deg\":" << windows[i].best_launch_azimuth_deg
             << ",\"min_delta_v_kms\":" << windows[i].min_delta_v_kms
             << ",\"feasibility\":\"" << feasibility_to_string(windows[i].best_engagement.feasibility) << "\""
             << ",\"confidence\":" << windows[i].best_engagement.confidence << "}";
    }
    json << "]";
    return json.str();
}

std::string wasm_evaluate_engagements_batch(const std::string& tle_catalog_text,
                                             double site_lat, double site_lon,
                                             double site_alt_km,
                                             int64_t window_start, int64_t window_end,
                                             double step_sec) {
    LaunchSite site;
    site.site_id = "wasm";
    site.name = "WASM Site";
    site.lat = site_lat;
    site.lon = site_lon;
    site.altitude_km = site_alt_km;

    // Parse TLE catalog (3-line format)
    auto lines = split_lines(tle_catalog_text);
    // Remove empty trailing lines
    while (!lines.empty() && lines.back().empty()) lines.pop_back();

    std::vector<TLE> targets;
    for (size_t i = 0; i + 2 < lines.size(); i += 3) {
        auto tle_opt = Propagator::parse_tle(lines[i], lines[i+1], lines[i+2]);
        if (tle_opt.has_value()) {
            targets.push_back(*tle_opt);
        }
    }

    auto params = Interceptor::default_interceptor();
    auto results = g_engagement.evaluate_engagements(site, targets, params,
                                                      static_cast<Timestamp>(window_start),
                                                      static_cast<Timestamp>(window_end),
                                                      step_sec);

    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < results.size(); ++i) {
        if (i > 0) json << ",";
        auto& r = results[i];
        json << "{\"target_norad_id\":" << r.target_norad_id
             << ",\"target_name\":\"" << r.target_name << "\""
             << ",\"feasibility\":\"" << feasibility_to_string(r.feasibility) << "\""
             << ",\"launch_azimuth_deg\":" << r.launch_azimuth_deg
             << ",\"launch_elevation_deg\":" << r.launch_elevation_deg
             << ",\"total_delta_v_kms\":" << r.total_delta_v_kms
             << ",\"time_of_flight_sec\":" << r.time_of_flight_sec
             << ",\"intercept_altitude_km\":" << r.intercept.altitude_km
             << ",\"intercept_range_km\":" << r.intercept.range_km
             << ",\"closing_velocity_kms\":" << r.intercept.closing_velocity_kms
             << ",\"confidence\":" << r.confidence << "}";
    }
    json << "]";
    return json.str();
}

double wasm_max_engagement_altitude() {
    auto params = Interceptor::default_interceptor();
    return g_engagement.max_engagement_altitude(params);
}

double wasm_compute_delta_v(double site_lat, double site_lon, double site_alt_km,
                             double intercept_x, double intercept_y, double intercept_z,
                             double tof_sec) {
    LaunchSite site;
    site.site_id = "wasm";
    site.name = "WASM Site";
    site.lat = site_lat;
    site.lon = site_lon;
    site.altitude_km = site_alt_km;

    Vec3 intercept_pos = {intercept_x, intercept_y, intercept_z};
    return g_interceptor.compute_delta_v(site, intercept_pos, tof_sec);
}

std::string wasm_simulate_interceptor_trajectory(double site_lat, double site_lon,
                                                   double site_alt_km,
                                                   double launch_azimuth_deg,
                                                   double launch_elevation_deg,
                                                   double dt_sec, double max_time_sec) {
    LaunchSite site;
    site.site_id = "wasm";
    site.name = "WASM Site";
    site.lat = site_lat;
    site.lon = site_lon;
    site.altitude_km = site_alt_km;

    auto params = Interceptor::default_interceptor();
    std::vector<OrbitalState> empty_targets; // no target for pure trajectory sim

    auto traj = g_interceptor.simulate_trajectory(site, launch_azimuth_deg, launch_elevation_deg,
                                                    params, empty_targets, dt_sec, max_time_sec);

    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i > 0) json << ",";
        json << "{\"time\":" << traj[i].time_sec
             << ",\"x\":" << traj[i].position.x
             << ",\"y\":" << traj[i].position.y
             << ",\"z\":" << traj[i].position.z
             << ",\"alt_km\":" << traj[i].altitude_km
             << ",\"stage\":" << traj[i].current_stage << "}";
    }
    json << "]";
    return json.str();
}

double wasm_orbital_period(const std::string& tle_name,
                            const std::string& tle_line1,
                            const std::string& tle_line2) {
    auto tle_opt = Propagator::parse_tle(tle_name, tle_line1, tle_line2);
    if (!tle_opt.has_value()) return 0.0;
    return Propagator::orbital_period_sec(*tle_opt);
}

std::string wasm_state_to_geodetic(double x, double y, double z,
                                    double vx, double vy, double vz,
                                    int64_t time_unix) {
    OrbitalState state;
    state.position = {x, y, z};
    state.velocity = {vx, vy, vz};
    state.epoch = static_cast<Timestamp>(time_unix);

    auto geo = Propagator::state_to_geodetic(state);

    std::ostringstream json;
    json << "{\"lat\":" << geo.lat << ",\"lon\":" << geo.lon << ",\"alt_km\":" << geo.alt_km << "}";
    return json.str();
}

std::string wasm_compute_pn_acceleration(double los_x, double los_y, double los_z,
                                          double los_rate_x, double los_rate_y, double los_rate_z,
                                          double closing_speed, double nav_gain) {
    Vec3 los = {los_x, los_y, los_z};
    Vec3 los_rate = {los_rate_x, los_rate_y, los_rate_z};

    Vec3 accel = g_interceptor.compute_pn_acceleration(los, los_rate, closing_speed, nav_gain);

    std::ostringstream json;
    json << "{\"ax\":" << accel.x << ",\"ay\":" << accel.y << ",\"az\":" << accel.z << "}";
    return json.str();
}

// ─── Emscripten Bindings ────────────────────────────────────────────────────

#ifdef __EMSCRIPTEN__
EMSCRIPTEN_BINDINGS(da_asat_wasm) {
    emscripten::function("parseTLE",                       &wasm_parse_tle);
    emscripten::function("propagate",                      &wasm_propagate);
    emscripten::function("evaluateEngagement",             &wasm_evaluate_engagement);
    emscripten::function("getDefaultInterceptor",          &wasm_get_default_interceptor);
    emscripten::function("engagementZoneBoundary",         &wasm_engagement_zone_boundary);
    emscripten::function("findEngagementWindows",          &wasm_find_engagement_windows);
    emscripten::function("evaluateEngagementsBatch",       &wasm_evaluate_engagements_batch);
    emscripten::function("maxEngagementAltitude",          &wasm_max_engagement_altitude);
    emscripten::function("computeDeltaV",                  &wasm_compute_delta_v);
    emscripten::function("simulateInterceptorTrajectory",  &wasm_simulate_interceptor_trajectory);
    emscripten::function("orbitalPeriod",                  &wasm_orbital_period);
    emscripten::function("stateToGeodetic",                &wasm_state_to_geodetic);
    emscripten::function("computePnAcceleration",          &wasm_compute_pn_acceleration);
}
#endif
