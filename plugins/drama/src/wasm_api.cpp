/**
 * @file wasm_api.cpp
 * @brief Emscripten C-API exports for all DRAMA modules
 *
 * Provides JSON-in/JSON-out C functions suitable for WASM binding.
 * Each function takes a JSON string, parses the configuration,
 * runs the corresponding DRAMA module, and returns a JSON result string.
 *
 * Exported functions:
 *   _compute_lifetime_json        (OSCAR)
 *   _check_25yr_compliance_json   (OSCAR)
 *   _compute_flux_json            (ARES)
 *   _compute_collision_risk_json  (ARES)
 *   _compute_cross_section_json   (CROC)
 *   _compute_average_cross_section_json (CROC)
 *   _analyze_reentry_json         (SARA)
 *   _compute_casualty_risk_json   (SARA)
 *   _get_environment_json         (MASTER)
 *   _check_compliance_json        (Compliance)
 */

#include "cpp/include/types.h"
#include "cpp/include/constants.h"
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

// ── External DRAMA functions ────────────────────────────────────────────────
namespace drama {
    extern LifetimeResult compute_lifetime(const LifetimeConfig& config);
    extern bool check_25yr_compliance(const LifetimeConfig& config);
    extern FluxResult compute_flux(const FluxConfig& config);
    extern FluxResult compute_collision_risk(const FluxConfig& config, double cross_section_m2);
    extern CrossSectionResult compute_cross_section(const SpacecraftGeometry& geom,
                                                     const AttitudeDirection& attitude);
    extern CrossSectionResult compute_average_cross_section(const SpacecraftGeometry& geom,
                                                             int n_samples);
    extern ReentryResult analyze_reentry(const ReentryConfig& config);
    extern double compute_casualty_risk(const ReentryResult& result);
    extern EnvironmentResult get_environment(const EnvironmentConfig& config);
    extern ComplianceReport check_full_compliance(const ComplianceConfig& config);
}

using namespace drama;

// ── Minimal JSON helpers (no external dependency) ───────────────────────────

static double json_double(const char* json, const char* key, double def) {
    const char* p = strstr(json, key);
    if (!p) return def;
    p = strchr(p, ':');
    if (!p) return def;
    return atof(p + 1);
}

static int json_int(const char* json, const char* key, int def) {
    const char* p = strstr(json, key);
    if (!p) return def;
    p = strchr(p, ':');
    if (!p) return def;
    return atoi(p + 1);
}

static std::string json_string(const char* json, const char* key, const char* def) {
    const char* p = strstr(json, key);
    if (!p) return def;
    p = strchr(p, ':');
    if (!p) return def;
    p = strchr(p, '"');
    if (!p) return def;
    p++;
    const char* end = strchr(p, '"');
    if (!end) return def;
    return std::string(p, end - p);
}

static KeplerianOrbit parse_orbit(const char* json) {
    KeplerianOrbit orb;
    orb.semi_major_axis_m = json_double(json, "\"semi_major_axis_m\"", (R_EARTH_KM + 500.0) * 1000.0);
    orb.eccentricity = json_double(json, "\"eccentricity\"", 0.001);
    orb.inclination_rad = json_double(json, "\"inclination_deg\"", 51.6) * DEG_TO_RAD;
    orb.raan_rad = json_double(json, "\"raan_deg\"", 0.0) * DEG_TO_RAD;
    orb.arg_perigee_rad = json_double(json, "\"arg_perigee_deg\"", 0.0) * DEG_TO_RAD;
    orb.true_anomaly_rad = json_double(json, "\"true_anomaly_deg\"", 0.0) * DEG_TO_RAD;
    return orb;
}

static SatelliteParams parse_satellite(const char* json) {
    SatelliteParams sat;
    sat.mass_kg = json_double(json, "\"mass_kg\"", 100.0);
    sat.cross_section_m2 = json_double(json, "\"cross_section_m2\"", 1.0);
    sat.cd = json_double(json, "\"cd\"", 2.2);
    sat.cr = json_double(json, "\"cr\"", 1.3);
    return sat;
}

static EpochTime parse_epoch(const char* json) {
    EpochTime ep;
    ep.year = json_int(json, "\"year\"", 2024);
    ep.month = json_int(json, "\"month\"", 1);
    ep.day = json_int(json, "\"day\"", 1);
    ep.hour = json_int(json, "\"hour\"", 0);
    ep.minute = json_int(json, "\"minute\"", 0);
    ep.second = json_double(json, "\"second\"", 0.0);
    return ep;
}

static SolarActivityModel parse_solar_model(const char* json) {
    std::string model = json_string(json, "\"solar_model\"", "mean");
    if (model == "constant") return SolarActivityModel::CONSTANT;
    if (model == "low") return SolarActivityModel::LOW_ACTIVITY;
    if (model == "high") return SolarActivityModel::HIGH_ACTIVITY;
    return SolarActivityModel::MEAN_CYCLE;
}

// Allocate a string on the WASM heap so JS can read it
static char* alloc_result(const std::string& s) {
    char* buf = (char*)malloc(s.size() + 1);
    memcpy(buf, s.c_str(), s.size() + 1);
    return buf;
}

// ── Exported C API ──────────────────────────────────────────────────────────

extern "C" {

/**
 * OSCAR: compute orbital lifetime
 * Input JSON: { semi_major_axis_m, eccentricity, inclination_deg,
 *               mass_kg, cross_section_m2, cd, year, month, day,
 *               solar_model, max_lifetime_years, time_step_days }
 */
char* compute_lifetime_json(const char* json) {
    LifetimeConfig lc;
    lc.initial_orbit = parse_orbit(json);
    lc.satellite = parse_satellite(json);
    lc.solar_model = parse_solar_model(json);
    lc.solar_override = {
        json_double(json, "\"f107\"", F107_MEAN),
        json_double(json, "\"f107a\"", F107_MEAN),
        json_double(json, "\"ap\"", AP_MODERATE)
    };
    lc.epoch = parse_epoch(json);
    lc.max_lifetime_years = json_double(json, "\"max_lifetime_years\"", 200.0);
    lc.time_step_days = json_double(json, "\"time_step_days\"", 1.0);

    LifetimeResult result = compute_lifetime(lc);

    std::ostringstream oss;
    oss << "{\"lifetime_years\":" << result.lifetime_years
        << ",\"compliant_25yr\":" << (result.compliant_25yr ? "true" : "false")
        << ",\"decay_year\":" << result.decay_date.year
        << ",\"decay_month\":" << result.decay_date.month
        << ",\"decay_day\":" << result.decay_date.day
        << ",\"history_points\":" << result.history.size()
        << ",\"notes\":\"" << result.notes << "\""
        << ",\"history\":[";
    for (size_t i = 0; i < result.history.size(); i++) {
        if (i > 0) oss << ",";
        oss << "{\"t\":" << result.history[i].time_years
            << ",\"hp\":" << result.history[i].perigee_km
            << ",\"ha\":" << result.history[i].apogee_km
            << ",\"e\":" << result.history[i].eccentricity << "}";
    }
    oss << "]}";
    return alloc_result(oss.str());
}

/**
 * OSCAR: check 25-year compliance
 */
char* check_25yr_compliance_json(const char* json) {
    LifetimeConfig lc;
    lc.initial_orbit = parse_orbit(json);
    lc.satellite = parse_satellite(json);
    lc.solar_model = parse_solar_model(json);
    lc.solar_override = {F107_MEAN, F107_MEAN, AP_MODERATE};
    lc.epoch = parse_epoch(json);
    lc.max_lifetime_years = 30.0;
    lc.time_step_days = 1.0;

    bool compliant = check_25yr_compliance(lc);

    std::ostringstream oss;
    oss << "{\"compliant\":" << (compliant ? "true" : "false") << "}";
    return alloc_result(oss.str());
}

/**
 * ARES: compute debris/meteoroid flux
 */
char* compute_flux_json(const char* json) {
    FluxConfig fc;
    fc.orbit = parse_orbit(json);
    fc.epoch = parse_epoch(json);
    fc.duration_years = json_double(json, "\"duration_years\"", 5.0);
    fc.min_diameter_m = json_double(json, "\"min_diameter_m\"", 0.01);

    FluxResult result = compute_flux(fc);

    std::ostringstream oss;
    oss << "{\"total_debris_flux\":" << result.total_debris_flux
        << ",\"total_meteoroid_flux\":" << result.total_meteoroid_flux
        << ",\"collision_probability_per_year\":" << result.collision_probability_per_year
        << ",\"cumulative_collision_probability\":" << result.cumulative_collision_probability
        << ",\"debris_bins\":[";
    for (size_t i = 0; i < result.debris_flux_by_size.size(); i++) {
        if (i > 0) oss << ",";
        oss << "{\"d\":" << result.debris_flux_by_size[i].diameter_m
            << ",\"f\":" << result.debris_flux_by_size[i].flux_per_m2_yr << "}";
    }
    oss << "],\"meteoroid_bins\":[";
    for (size_t i = 0; i < result.meteoroid_flux_by_size.size(); i++) {
        if (i > 0) oss << ",";
        oss << "{\"d\":" << result.meteoroid_flux_by_size[i].diameter_m
            << ",\"f\":" << result.meteoroid_flux_by_size[i].flux_per_m2_yr << "}";
    }
    oss << "]}";
    return alloc_result(oss.str());
}

/**
 * ARES: compute collision risk with explicit cross-section
 */
char* compute_collision_risk_json(const char* json) {
    FluxConfig fc;
    fc.orbit = parse_orbit(json);
    fc.epoch = parse_epoch(json);
    fc.duration_years = json_double(json, "\"duration_years\"", 5.0);
    fc.min_diameter_m = json_double(json, "\"min_diameter_m\"", 0.01);

    double cross_section = json_double(json, "\"cross_section_m2\"", 1.0);
    FluxResult result = compute_collision_risk(fc, cross_section);

    std::ostringstream oss;
    oss << "{\"total_debris_flux\":" << result.total_debris_flux
        << ",\"total_meteoroid_flux\":" << result.total_meteoroid_flux
        << ",\"collision_probability_per_year\":" << result.collision_probability_per_year
        << ",\"cumulative_collision_probability\":" << result.cumulative_collision_probability
        << ",\"cross_section_m2\":" << cross_section << "}";
    return alloc_result(oss.str());
}

/**
 * CROC: compute cross-section at specific attitude
 */
char* compute_cross_section_json(const char* json) {
    SpacecraftGeometry geom;
    // Parse a single primitive from JSON for simplicity
    GeometricPrimitive prim;
    std::string ptype = json_string(json, "\"shape\"", "box");
    if (ptype == "sphere") prim.type = PrimitiveType::SPHERE;
    else if (ptype == "cylinder") prim.type = PrimitiveType::CYLINDER;
    else if (ptype == "panel") prim.type = PrimitiveType::FLAT_PANEL;
    else prim.type = PrimitiveType::BOX;

    prim.dim_x_m = json_double(json, "\"dim_x\"", 1.0);
    prim.dim_y_m = json_double(json, "\"dim_y\"", 1.0);
    prim.dim_z_m = json_double(json, "\"dim_z\"", 1.0);
    prim.offset_x_m = prim.offset_y_m = prim.offset_z_m = 0;
    geom.primitives.push_back(prim);

    int n_samples = json_int(json, "\"n_samples\"", 10000);
    CrossSectionResult result = compute_average_cross_section(geom, n_samples);

    std::ostringstream oss;
    oss << "{\"average_area_m2\":" << result.average_area_m2
        << ",\"max_area_m2\":" << result.max_area_m2
        << ",\"min_area_m2\":" << result.min_area_m2 << "}";
    return alloc_result(oss.str());
}

/**
 * CROC: compute average cross-section (random tumbling)
 */
char* compute_average_cross_section_json(const char* json) {
    // Same as compute_cross_section_json — the function already computes average
    return compute_cross_section_json(json);
}

/**
 * SARA: analyze re-entry survival
 */
char* analyze_reentry_json(const char* json) {
    ReentryConfig rc;
    rc.initial_orbit = parse_orbit(json);
    rc.total_mass_kg = json_double(json, "\"total_mass_kg\"", 100.0);
    rc.breakup_altitude_km = json_double(json, "\"breakup_altitude_km\"", 78.0);
    rc.ground_impact_velocity_threshold_ms = json_double(json, "\"impact_threshold_ms\"", 1.0);
    rc.epoch = parse_epoch(json);

    // Parse a single component
    SpacecraftComponent comp;
    comp.name = json_string(json, "\"component_name\"", "component");
    std::string mat = json_string(json, "\"material\"", "aluminum");
    if (mat == "steel") comp.material = MaterialType::STEEL;
    else if (mat == "titanium") comp.material = MaterialType::TITANIUM;
    else if (mat == "cfrp") comp.material = MaterialType::CFRP;
    else if (mat == "copper") comp.material = MaterialType::COPPER;
    else if (mat == "glass") comp.material = MaterialType::GLASS;
    else if (mat == "inconel") comp.material = MaterialType::INCONEL;
    else if (mat == "stainless") comp.material = MaterialType::STAINLESS_STEEL;
    else comp.material = MaterialType::ALUMINUM;

    comp.mass_kg = json_double(json, "\"component_mass_kg\"", rc.total_mass_kg);
    comp.characteristic_length_m = json_double(json, "\"characteristic_length_m\"", 0.1);
    comp.shape.type = PrimitiveType::SPHERE;
    comp.shape.dim_x_m = comp.characteristic_length_m;
    rc.components.push_back(comp);

    ReentryResult result = analyze_reentry(rc);

    std::ostringstream oss;
    oss << "{\"total_surviving_mass_kg\":" << result.total_surviving_mass_kg
        << ",\"casualty_area_m2\":" << result.casualty_area_m2
        << ",\"casualty_expectation\":" << result.casualty_expectation
        << ",\"compliant_1e4\":" << (result.compliant_1e4 ? "true" : "false")
        << ",\"ground_footprint_km2\":" << result.ground_footprint_km2
        << ",\"notes\":\"" << result.notes << "\""
        << ",\"fragments\":[";
    for (size_t i = 0; i < result.fragments.size(); i++) {
        if (i > 0) oss << ",";
        const auto& f = result.fragments[i];
        oss << "{\"name\":\"" << f.component_name << "\""
            << ",\"demised\":" << (f.demised ? "true" : "false")
            << ",\"mass_kg\":" << f.mass_kg
            << ",\"impact_velocity_ms\":" << f.impact_velocity_ms
            << ",\"impact_energy_j\":" << f.impact_energy_j
            << ",\"casualty_area_m2\":" << f.area_m2
            << ",\"peak_heating_kw_m2\":" << f.peak_heating_kw_m2
            << ",\"demise_altitude_km\":" << f.demise_altitude_km << "}";
    }
    oss << "]}";
    return alloc_result(oss.str());
}

/**
 * SARA: compute casualty risk from re-entry result
 */
char* compute_casualty_risk_json(const char* json) {
    // Run full reentry analysis and return just the casualty risk
    ReentryConfig rc;
    rc.initial_orbit = parse_orbit(json);
    rc.total_mass_kg = json_double(json, "\"total_mass_kg\"", 100.0);
    rc.breakup_altitude_km = json_double(json, "\"breakup_altitude_km\"", 78.0);
    rc.ground_impact_velocity_threshold_ms = 1.0;
    rc.epoch = parse_epoch(json);

    SpacecraftComponent comp;
    comp.name = "component";
    comp.material = MaterialType::ALUMINUM;
    comp.mass_kg = rc.total_mass_kg;
    comp.characteristic_length_m = json_double(json, "\"characteristic_length_m\"", 0.1);
    comp.shape.type = PrimitiveType::SPHERE;
    comp.shape.dim_x_m = comp.characteristic_length_m;
    rc.components.push_back(comp);

    ReentryResult result = analyze_reentry(rc);
    double risk = compute_casualty_risk(result);

    std::ostringstream oss;
    oss << "{\"casualty_risk\":" << risk
        << ",\"compliant_1e4\":" << (risk < CASUALTY_RISK_THRESHOLD ? "true" : "false") << "}";
    return alloc_result(oss.str());
}

/**
 * MASTER: get space debris environment
 */
char* get_environment_json(const char* json) {
    EnvironmentConfig ec;
    ec.epoch = parse_epoch(json);
    ec.altitude_min_km = json_double(json, "\"altitude_min_km\"", 200.0);
    ec.altitude_max_km = json_double(json, "\"altitude_max_km\"", 2000.0);
    ec.altitude_step_km = json_double(json, "\"altitude_step_km\"", 50.0);
    ec.inclination_min_deg = json_double(json, "\"inclination_min_deg\"", 0.0);
    ec.inclination_max_deg = json_double(json, "\"inclination_max_deg\"", 180.0);
    ec.size_min_m = json_double(json, "\"size_min_m\"", 0.01);
    ec.size_max_m = json_double(json, "\"size_max_m\"", 10.0);

    EnvironmentResult result = get_environment(ec);

    std::ostringstream oss;
    oss << "{\"total_debris_objects\":" << result.total_debris_objects
        << ",\"total_meteoroid_flux\":" << result.total_meteoroid_flux
        << ",\"debris_by_altitude\":[";
    for (size_t i = 0; i < result.debris_by_altitude.size(); i++) {
        if (i > 0) oss << ",";
        oss << "{\"alt\":" << result.debris_by_altitude[i].altitude_km
            << ",\"sd\":" << result.debris_by_altitude[i].spatial_density
            << ",\"flux\":" << result.debris_by_altitude[i].flux << "}";
    }
    oss << "],\"size_distribution\":[";
    for (size_t i = 0; i < result.debris_size_distribution.size(); i++) {
        if (i > 0) oss << ",";
        oss << "{\"d\":" << result.debris_size_distribution[i].diameter_m
            << ",\"n\":" << result.debris_size_distribution[i].cumulative_count << "}";
    }
    oss << "]}";
    return alloc_result(oss.str());
}

/**
 * Compliance: run full compliance check
 */
char* check_compliance_json(const char* json) {
    ComplianceConfig cc;
    cc.orbit = parse_orbit(json);
    cc.satellite = parse_satellite(json);
    cc.epoch = parse_epoch(json);
    cc.solar_model = parse_solar_model(json);
    cc.mission_duration_years = json_double(json, "\"mission_duration_years\"", 5.0);
    cc.post_mission_disposal_years = json_double(json, "\"post_mission_disposal_years\"", 0.0);

    // Default geometry: box from cross-section
    GeometricPrimitive body;
    body.type = PrimitiveType::BOX;
    double side = std::sqrt(cc.satellite.cross_section_m2);
    body.dim_x_m = side;
    body.dim_y_m = side;
    body.dim_z_m = side;
    body.offset_x_m = body.offset_y_m = body.offset_z_m = 0;
    cc.geometry.primitives.push_back(body);

    // Default component: aluminum structure
    SpacecraftComponent comp;
    comp.name = "structure";
    comp.material = MaterialType::ALUMINUM;
    comp.mass_kg = cc.satellite.mass_kg;
    comp.characteristic_length_m = side;
    comp.shape = body;
    cc.components.push_back(comp);

    ComplianceReport report = check_full_compliance(cc);

    std::ostringstream oss;
    oss << "{\"all_passed\":" << (report.all_passed ? "true" : "false")
        << ",\"lifetime_years\":" << report.lifetime.lifetime_years
        << ",\"compliant_25yr\":" << (report.lifetime.compliant_25yr ? "true" : "false")
        << ",\"casualty_expectation\":" << report.reentry.casualty_expectation
        << ",\"average_cross_section_m2\":" << report.cross_section.average_area_m2
        << ",\"collision_probability_per_year\":" << report.flux.collision_probability_per_year
        << ",\"guidelines\":[";
    for (size_t i = 0; i < report.results.size(); i++) {
        if (i > 0) oss << ",";
        const auto& r = report.results[i];
        oss << "{\"name\":\"" << r.name << "\""
            << ",\"passed\":" << (r.passed ? "true" : "false")
            << ",\"details\":\"" << r.details << "\""
            << ",\"findings\":[";
        for (size_t j = 0; j < r.findings.size(); j++) {
            if (j > 0) oss << ",";
            oss << "\"" << r.findings[j] << "\"";
        }
        oss << "]}";
    }
    oss << "]}";
    return alloc_result(oss.str());
}

} // extern "C"
