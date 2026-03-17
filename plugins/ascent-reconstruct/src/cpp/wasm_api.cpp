#include "ascent_reconstruct/estimator.h"
#include "ascent_reconstruct/trajectory.h"
#include "ascent_reconstruct/types.h"

#include <emscripten/emscripten.h>
#include <emscripten/bind.h>

#include <string>
#include <sstream>

using namespace ascent_reconstruct;

// Global instances
static TrajectorySimulator g_simulator;
static AscentEstimator g_estimator;

// Simulate an ascent trajectory given JSON parameters, return JSON trajectory
std::string wasm_simulate_ascent(const std::string& params_json) {
    // For now, return a placeholder - full JSON parsing would require a JSON library
    // In production, this would parse the JSON into AscentModelParams and simulate
    std::ostringstream json;
    json << R"({"status":"ok","message":"simulate endpoint ready"})";
    return json.str();
}

// Reconstruct trajectory from tracklet observations JSON
std::string wasm_reconstruct(const std::string& tracklet_json) {
    std::ostringstream json;
    json << R"({"status":"ok","message":"reconstruct endpoint ready"})";
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
    emscripten::function("simulateAscent",   &wasm_simulate_ascent);
    emscripten::function("reconstruct",      &wasm_reconstruct);
    emscripten::function("geodeticToEcef",   &wasm_geodetic_to_ecef);
    emscripten::function("ecefToGeodetic",   &wasm_ecef_to_geodetic);
    emscripten::function("atmDensity",       &wasm_atm_density);
}
