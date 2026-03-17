#include "da_asat/propagator.h"
#include "da_asat/interceptor.h"
#include "da_asat/engagement.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#include <emscripten/bind.h>
#endif

#include <string>
#include <sstream>

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

// ─── Emscripten Bindings ────────────────────────────────────────────────────

#ifdef __EMSCRIPTEN__
EMSCRIPTEN_BINDINGS(da_asat_wasm) {
    emscripten::function("parseTLE",                  &wasm_parse_tle);
    emscripten::function("propagate",                 &wasm_propagate);
    emscripten::function("evaluateEngagement",        &wasm_evaluate_engagement);
    emscripten::function("getDefaultInterceptor",     &wasm_get_default_interceptor);
    emscripten::function("engagementZoneBoundary",    &wasm_engagement_zone_boundary);
}
#endif
