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

// ---- Helper: split string by delimiter ----------------------------------------

static std::vector<std::string> split_string(const std::string& s, char delim) {
    std::vector<std::string> tokens;
    std::istringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, delim)) {
        tokens.push_back(tok);
    }
    return tokens;
}

// ---- Helper: parse TLE catalog (3-line format) --------------------------------

static std::vector<TLE> parse_tle_catalog(const std::string& text) {
    std::vector<TLE> catalog;
    auto lines = split_string(text, '\n');
    // Remove empty trailing lines
    while (!lines.empty() && lines.back().empty()) lines.pop_back();
    for (size_t i = 0; i + 2 < lines.size(); i += 3) {
        std::string name = lines[i];
        // Trim trailing whitespace/CR
        while (!name.empty() && (name.back() == '\r' || name.back() == ' ')) name.pop_back();
        std::string l1 = lines[i+1];
        while (!l1.empty() && (l1.back() == '\r' || l1.back() == ' ')) l1.pop_back();
        std::string l2 = lines[i+2];
        while (!l2.empty() && (l2.back() == '\r' || l2.back() == ' ')) l2.pop_back();
        catalog.push_back(SGP4::parse_tle(name, l1, l2));
    }
    return catalog;
}

// ---- New WASM API Functions ---------------------------------------------------

std::string wasm_batch_correlate(const std::string& zones_str, const std::string& catalog_text) {
    // Parse zones: each zone as "id,center_lat,center_lon,radius_nm,is_circle,eff_start,eff_end" separated by semicolons
    std::vector<ExclusionZone> zones;
    auto zone_strs = split_string(zones_str, ';');
    for (auto& zs : zone_strs) {
        if (zs.empty()) continue;
        auto fields = split_string(zs, ',');
        if (fields.size() < 7) continue;
        ExclusionZone zone;
        zone.id = fields[0];
        zone.center.lat = std::stod(fields[1]);
        zone.center.lon = std::stod(fields[2]);
        zone.radius_nm = std::stod(fields[3]);
        zone.is_circle = (fields[4] == "1" || fields[4] == "true");
        zone.effective_start = static_cast<int64_t>(std::stod(fields[5]));
        zone.effective_end = static_cast<int64_t>(std::stod(fields[6]));
        zone.compute_bounds();
        zones.push_back(zone);
    }

    auto catalog = parse_tle_catalog(catalog_text);
    auto results = g_correlator.batch_correlate(zones, catalog);

    std::ostringstream json;
    json << "[";
    for (size_t i = 0; i < results.size(); ++i) {
        if (i > 0) json << ",";
        auto& c = results[i];
        json << "{\"zone_id\":\"" << c.zone_id << "\","
             << "\"norad_id\":" << c.norad_id << ","
             << "\"object_name\":\"" << c.object_name << "\","
             << "\"confidence\":" << c.confidence << ","
             << "\"min_distance_km\":" << c.min_distance_km << ","
             << "\"predicted_crossing_time\":" << c.predicted_crossing_time << ","
             << "\"crossing_lat\":" << c.crossing_point.lat << ","
             << "\"crossing_lon\":" << c.crossing_point.lon << ","
             << "\"ascending_pass\":" << (c.ascending_pass ? "true" : "false") << ","
             << "\"inclination_match\":" << c.inclination_match << "}";
    }
    json << "]";
    return json.str();
}

std::string wasm_predict_launch_time(const std::string& zone_id,
                                      double center_lat, double center_lon,
                                      double radius_nm,
                                      double eff_start, double eff_end,
                                      const std::string& tle_name,
                                      const std::string& tle_line1,
                                      const std::string& tle_line2) {
    ExclusionZone zone;
    zone.id = zone_id;
    zone.center = {center_lat, center_lon};
    zone.radius_nm = radius_nm;
    zone.is_circle = true;
    zone.effective_start = static_cast<int64_t>(eff_start);
    zone.effective_end = static_cast<int64_t>(eff_end);
    zone.compute_bounds();

    TLE tle = SGP4::parse_tle(tle_name, tle_line1, tle_line2);
    auto pred = g_correlator.predict_launch_time(zone, tle);

    std::ostringstream json;
    json << "{\"launch_time\":" << pred.launch_time
         << ",\"confidence\":" << pred.confidence
         << ",\"norad_id\":" << pred.norad_id
         << ",\"rationale\":\"" << pred.rationale << "\"}";
    return json.str();
}

bool wasm_point_in_polygon(double lat, double lon, const std::string& vertices_str) {
    // vertices as "lat1,lon1;lat2,lon2;..."
    std::vector<LatLon> vertices;
    auto pairs = split_string(vertices_str, ';');
    for (auto& p : pairs) {
        if (p.empty()) continue;
        auto coords = split_string(p, ',');
        if (coords.size() < 2) continue;
        vertices.push_back({std::stod(coords[0]), std::stod(coords[1])});
    }
    return Geometry::point_in_polygon({lat, lon}, vertices);
}

bool wasm_point_in_circle(double lat, double lon,
                           double center_lat, double center_lon,
                           double radius_nm) {
    return Geometry::point_in_circle({lat, lon}, {center_lat, center_lon}, radius_nm);
}

double wasm_haversine_km(double lat1, double lon1, double lat2, double lon2) {
    return Geometry::haversine_km({lat1, lon1}, {lat2, lon2});
}

double wasm_haversine_nm(double lat1, double lon1, double lat2, double lon2) {
    return Geometry::haversine_nm({lat1, lon1}, {lat2, lon2});
}

double wasm_bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    return Geometry::bearing_deg({lat1, lon1}, {lat2, lon2});
}

double wasm_min_distance_to_zone_km(const std::string& track_str, const std::string& zone_str) {
    // track_str: "lat1,lon1;lat2,lon2;..." - ground track points
    // zone_str: "id,center_lat,center_lon,radius_nm,is_circle"
    std::vector<GroundTrackPoint> track;
    auto pairs = split_string(track_str, ';');
    for (auto& p : pairs) {
        if (p.empty()) continue;
        auto coords = split_string(p, ',');
        if (coords.size() < 2) continue;
        GroundTrackPoint gtp;
        gtp.position = {std::stod(coords[0]), std::stod(coords[1])};
        gtp.altitude_km = (coords.size() > 2) ? std::stod(coords[2]) : 0.0;
        track.push_back(gtp);
    }

    auto zfields = split_string(zone_str, ',');
    ExclusionZone zone;
    if (zfields.size() >= 5) {
        zone.id = zfields[0];
        zone.center = {std::stod(zfields[1]), std::stod(zfields[2])};
        zone.radius_nm = std::stod(zfields[3]);
        zone.is_circle = (zfields[4] == "1" || zfields[4] == "true");
        zone.compute_bounds();
    }

    return Geometry::min_distance_to_zone_km(track, zone);
}

int64_t wasm_tle_epoch_to_unix(int epoch_year, double epoch_day) {
    return SGP4::tle_epoch_to_unix(epoch_year, epoch_day);
}

double wasm_compute_confidence(double min_dist_km, double zone_radius_km, double incl_match) {
    return Correlator::compute_confidence(min_dist_km, zone_radius_km, incl_match);
}

// ---- Emscripten Bindings ----------------------------------------------------

EMSCRIPTEN_BINDINGS(tle_correlator_wasm) {
    emscripten::function("parseTle",              &wasm_parse_tle);
    emscripten::function("groundTrack",           &wasm_ground_track);
    emscripten::function("correlate",             &wasm_correlate);
    emscripten::function("batchCorrelate",        &wasm_batch_correlate);
    emscripten::function("predictLaunchTime",     &wasm_predict_launch_time);
    emscripten::function("pointInPolygon",        &wasm_point_in_polygon);
    emscripten::function("pointInCircle",         &wasm_point_in_circle);
    emscripten::function("haversineKm",           &wasm_haversine_km);
    emscripten::function("haversineNm",           &wasm_haversine_nm);
    emscripten::function("bearingDeg",            &wasm_bearing_deg);
    emscripten::function("minDistanceToZoneKm",   &wasm_min_distance_to_zone_km);
    emscripten::function("tleEpochToUnix",        &wasm_tle_epoch_to_unix);
    emscripten::function("computeConfidence",     &wasm_compute_confidence);
}
