#include "tle_correlator/types.h"
#include "tle_correlator/sgp4.h"
#include "tle_correlator/geometry.h"
#include "tle_correlator/correlator.h"

#include <emscripten/emscripten.h>
#include <emscripten/bind.h>

#include <string>
#include <sstream>

using namespace tle_correlator;

// ---- Global instances -------------------------------------------------------

static Correlator g_correlator;

// ---- WASM API Functions -----------------------------------------------------

// Parse a TLE and return JSON with parsed fields
std::string wasm_parse_tle(const std::string& name, const std::string& line1, const std::string& line2) {
    TLE tle = SGP4::parse_tle(name, line1, line2);
    std::ostringstream json;
    json << "{";
    json << "\"name\":\"" << tle.name << "\",";
    json << "\"norad_id\":" << tle.norad_id << ",";
    json << "\"intl_designator\":\"" << tle.intl_designator << "\",";
    json << "\"epoch_year\":" << tle.epoch_year << ",";
    json << "\"epoch_day\":" << tle.epoch_day << ",";
    json << "\"inclination_deg\":" << tle.inclination_deg << ",";
    json << "\"raan_deg\":" << tle.raan_deg << ",";
    json << "\"eccentricity\":" << tle.eccentricity << ",";
    json << "\"arg_perigee_deg\":" << tle.arg_perigee_deg << ",";
    json << "\"mean_anomaly_deg\":" << tle.mean_anomaly_deg << ",";
    json << "\"mean_motion\":" << tle.mean_motion << ",";
    json << "\"bstar\":" << tle.bstar;
    json << "}";
    return json.str();
}

// Generate ground track as JSON array of {lat, lon, alt, time}
std::string wasm_ground_track(const std::string& name, const std::string& line1,
                               const std::string& line2,
                               double start_unix, double end_unix, double step_sec) {
    TLE tle = SGP4::parse_tle(name, line1, line2);
    auto track = SGP4::ground_track(tle,
                                     static_cast<int64_t>(start_unix),
                                     static_cast<int64_t>(end_unix),
                                     step_sec);
    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < track.size(); ++i) {
        if (i > 0) json << ",";
        json << "{\"lat\":" << track[i].position.lat
             << ",\"lon\":" << track[i].position.lon
             << ",\"alt_km\":" << track[i].altitude_km
             << ",\"time\":" << track[i].timestamp
             << ",\"ascending\":" << (track[i].ascending ? "true" : "false")
             << "}";
    }
    json << "]";
    return json.str();
}

// Correlate a single TLE against an exclusion zone, return JSON
std::string wasm_correlate(const std::string& zone_id,
                            double center_lat, double center_lon,
                            double radius_nm, bool is_circle,
                            double eff_start, double eff_end,
                            const std::string& tle_name,
                            const std::string& tle_line1,
                            const std::string& tle_line2) {
    ExclusionZone zone;
    zone.id = zone_id;
    zone.center = {center_lat, center_lon};
    zone.radius_nm = radius_nm;
    zone.is_circle = is_circle;
    zone.effective_start = static_cast<int64_t>(eff_start);
    zone.effective_end = static_cast<int64_t>(eff_end);
    zone.compute_bounds();

    TLE tle = SGP4::parse_tle(tle_name, tle_line1, tle_line2);
    Correlation corr = g_correlator.correlate_single(zone, tle);

    std::ostringstream json;
    json << "{";
    json << "\"zone_id\":\"" << corr.zone_id << "\",";
    json << "\"norad_id\":" << corr.norad_id << ",";
    json << "\"object_name\":\"" << corr.object_name << "\",";
    json << "\"confidence\":" << corr.confidence << ",";
    json << "\"min_distance_km\":" << corr.min_distance_km << ",";
    json << "\"predicted_crossing_time\":" << corr.predicted_crossing_time << ",";
    json << "\"crossing_lat\":" << corr.crossing_point.lat << ",";
    json << "\"crossing_lon\":" << corr.crossing_point.lon << ",";
    json << "\"ascending_pass\":" << (corr.ascending_pass ? "true" : "false") << ",";
    json << "\"inclination_match\":" << corr.inclination_match;
    json << "}";
    return json.str();
}

// ---- Emscripten Bindings ----------------------------------------------------

EMSCRIPTEN_BINDINGS(tle_correlator_wasm) {
    emscripten::function("parseTle",      &wasm_parse_tle);
    emscripten::function("groundTrack",   &wasm_ground_track);
    emscripten::function("correlate",     &wasm_correlate);
}
