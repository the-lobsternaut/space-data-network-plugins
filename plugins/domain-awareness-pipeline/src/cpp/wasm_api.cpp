#ifdef __EMSCRIPTEN__
#include <emscripten/bind.h>
#endif

#include "da_pipeline/pipeline.h"
#include "da_pipeline/sgp4.h"
#include "da_pipeline/exclusion_zone.h"
#include "da_pipeline/ascent_estimator.h"
#include "da_pipeline/engagement.h"
#include <sstream>
#include <iomanip>

using namespace da_pipeline;

// ---------------------------------------------------------------------------
// JSON serialization helpers
// ---------------------------------------------------------------------------
static std::string escape_json(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 10);
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += c;
        }
    }
    return out;
}

static std::string notam_to_json(const NOTAM& n) {
    std::ostringstream ss;
    ss << std::setprecision(8);
    const char* cls[] = {"LAUNCH","REENTRY","HAZARD","AIRSPACE","ROUTINE"};
    ss << "{\"notam_id\":\"" << escape_json(n.notam_id) << "\""
       << ",\"location_id\":\"" << escape_json(n.location_id) << "\""
       << ",\"center_lat\":" << n.center_lat
       << ",\"center_lon\":" << n.center_lon
       << ",\"radius_nm\":" << n.radius_nm
       << ",\"altitude_floor_ft\":" << n.altitude_floor_ft
       << ",\"altitude_ceiling_ft\":" << n.altitude_ceiling_ft
       << ",\"effective_start\":" << n.effective_start
       << ",\"effective_end\":" << n.effective_end
       << ",\"classification\":\"" << cls[static_cast<int>(n.classification)] << "\""
       << "}";
    return ss.str();
}

static std::string ntm_to_json(const MaritimeNotice& m) {
    std::ostringstream ss;
    ss << std::setprecision(8);
    const char* hcl[] = {"ROCKET_DEBRIS","SPLASHDOWN","BOOSTER_RECOVERY",
                         "FAIRING_RECOVERY","GENERAL_HAZARD","UNKNOWN"};
    ss << "{\"notice_id\":\"" << escape_json(m.notice_id) << "\""
       << ",\"classification\":\"" << hcl[static_cast<int>(m.classification)] << "\""
       << ",\"effective_start\":" << m.effective_start
       << ",\"effective_end\":" << m.effective_end
       << ",\"confidence\":" << m.confidence
       << ",\"is_cancelled\":" << (m.is_cancelled ? "true" : "false")
       << "}";
    return ss.str();
}

static std::string tle_to_json(const TLE& t) {
    std::ostringstream ss;
    ss << std::setprecision(8);
    ss << "{\"name\":\"" << escape_json(t.name) << "\""
       << ",\"norad_id\":" << t.norad_id
       << ",\"inclination_deg\":" << t.inclination_deg
       << ",\"raan_deg\":" << t.raan_deg
       << ",\"eccentricity\":" << t.eccentricity
       << ",\"mean_motion\":" << t.mean_motion
       << "}";
    return ss.str();
}

static std::string correlation_to_json(const Correlation& c) {
    std::ostringstream ss;
    ss << std::setprecision(8);
    ss << "{\"zone_id\":\"" << escape_json(c.zone_id) << "\""
       << ",\"norad_id\":" << c.norad_id
       << ",\"object_name\":\"" << escape_json(c.object_name) << "\""
       << ",\"confidence\":" << c.confidence
       << ",\"min_distance_km\":" << c.min_distance_km
       << ",\"ascending_pass\":" << (c.ascending_pass ? "true" : "false")
       << "}";
    return ss.str();
}

// ---------------------------------------------------------------------------
// WASM API functions
// ---------------------------------------------------------------------------
static std::string runPipeline(const std::string& notam_text,
                               const std::string& ntm_text,
                               const std::string& tle_catalog) {
    DomainAwarenessPipeline pipeline;
    auto sa = pipeline.run_full_pipeline(notam_text, ntm_text, tle_catalog);

    std::ostringstream ss;
    ss << std::setprecision(8);

    const char* threat_str[] = {"LOW","MODERATE","HIGH","CRITICAL"};

    ss << "{\"generated_at\":" << sa.generated_at
       << ",\"total_active_zones\":" << sa.total_active_zones
       << ",\"launch_sites_active\":" << sa.launch_sites_active
       << ",\"tle_correlations_found\":" << sa.tle_correlations_found
       << ",\"asat_threats_identified\":" << sa.asat_threats_identified
       << ",\"overall_threat\":\"" << threat_str[static_cast<int>(sa.overall_threat)] << "\""
       << ",\"parsed_notams\":[";

    for (size_t i = 0; i < sa.parsed_notams.size(); i++) {
        if (i > 0) ss << ",";
        ss << notam_to_json(sa.parsed_notams[i]);
    }

    ss << "],\"parsed_ntms\":[";
    for (size_t i = 0; i < sa.parsed_ntms.size(); i++) {
        if (i > 0) ss << ",";
        ss << ntm_to_json(sa.parsed_ntms[i]);
    }

    ss << "],\"correlations\":[";
    for (size_t i = 0; i < sa.correlations.size(); i++) {
        if (i > 0) ss << ",";
        ss << correlation_to_json(sa.correlations[i]);
    }

    ss << "],\"summary\":\"" << escape_json(sa.summary_text) << "\"}";
    return ss.str();
}

static std::string parseNotams(const std::string& text) {
    NotamParser parser;
    auto notams = parser.parse_batch(text);
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < notams.size(); i++) {
        if (i > 0) ss << ",";
        ss << notam_to_json(notams[i]);
    }
    ss << "]";
    return ss.str();
}

static std::string parseNtms(const std::string& text) {
    NtmParser parser;
    auto ntms = parser.parse_batch(text);
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < ntms.size(); i++) {
        if (i > 0) ss << ",";
        ss << ntm_to_json(ntms[i]);
    }
    ss << "]";
    return ss.str();
}

static std::string parseTleCatalog(const std::string& text) {
    auto catalog = SGP4::parse_catalog(text);
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < catalog.size(); i++) {
        if (i > 0) ss << ",";
        ss << tle_to_json(catalog[i]);
    }
    ss << "]";
    return ss.str();
}

static std::string correlateZones(const std::string& zone_json, const std::string& tle_catalog) {
    // Parse TLE catalog
    auto catalog = SGP4::parse_catalog(tle_catalog);

    // Create a simple zone from the JSON (latitude, longitude, radius_nm)
    // Simplified parser: expects "lat,lon,radius_nm"
    ExclusionZone zone;
    zone.id = "QUERY";
    zone.type = GeometryType::CIRCLE;

    std::istringstream iss(zone_json);
    char delim;
    iss >> zone.circle.center.lat >> delim >> zone.circle.center.lon >> delim >> zone.circle.radius_nm;
    if (zone.circle.radius_nm <= 0) zone.circle.radius_nm = 100;
    zone.compute_bounds();

    // Use epoch of first TLE for time window
    if (!catalog.empty()) {
        Timestamp epoch = SGP4::tle_epoch_to_unix(catalog[0].epoch_year, catalog[0].epoch_day);
        zone.effective_start = epoch - 43200;
        zone.effective_end = epoch + 43200;
    }

    TLECorrelator corr;
    auto results = corr.correlate(zone, catalog);

    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < results.size(); i++) {
        if (i > 0) ss << ",";
        ss << correlation_to_json(results[i]);
    }
    ss << "]";
    return ss.str();
}

static std::string generateTrajectoryFamily(double site_lat, double site_lon, double azimuth_deg) {
    LaunchSite site;
    site.site_id = "QUERY";
    site.name = "Query Site";
    site.lat = site_lat;
    site.lon = site_lon;

    AscentEstimator estimator;
    auto family = estimator.generate_family(site, azimuth_deg);

    std::ostringstream ss;
    ss << std::setprecision(8);
    ss << "{\"family_id\":\"" << escape_json(family.family_id) << "\""
       << ",\"min_azimuth_deg\":" << family.min_azimuth_deg
       << ",\"max_azimuth_deg\":" << family.max_azimuth_deg
       << ",\"estimated_inclination_deg\":" << family.estimated_inclination_deg
       << ",\"nominal_trajectory_points\":" << family.nominal_trajectory.size()
       << ",\"dispersions\":" << family.dispersions.size()
       << ",\"nominal_burnout_alt_m\":";

    if (!family.nominal_trajectory.empty()) {
        ss << family.nominal_trajectory.back().altitude_m;
    } else {
        ss << 0;
    }
    ss << "}";
    return ss.str();
}

static std::string assessAsatThreat(double site_lat, double site_lon,
                                     const std::string& tle_catalog,
                                     double window_start, double window_end) {
    LaunchSite site;
    site.site_id = "QUERY";
    site.name = "Query Site";
    site.lat = site_lat;
    site.lon = site_lon;

    auto catalog = SGP4::parse_catalog(tle_catalog);
    auto params = EngagementAnalyzer::default_interceptor();

    EngagementAnalyzer analyzer;
    auto ta = analyzer.assess_threat(
        site, catalog, params,
        static_cast<Timestamp>(window_start),
        static_cast<Timestamp>(window_end));

    const char* lvl[] = {"LOW","MODERATE","HIGH","CRITICAL"};
    std::ostringstream ss;
    ss << std::setprecision(8);
    ss << "{\"site_id\":\"" << escape_json(ta.site_id) << "\""
       << ",\"threat_level\":\"" << lvl[static_cast<int>(ta.level)] << "\""
       << ",\"rationale\":\"" << escape_json(ta.rationale) << "\""
       << ",\"feasible_engagements\":" << ta.feasible_engagements.size()
       << ",\"threatened_norad_ids\":[";
    for (size_t i = 0; i < ta.threatened_norad_ids.size(); i++) {
        if (i > 0) ss << ",";
        ss << ta.threatened_norad_ids[i];
    }
    ss << "]}";
    return ss.str();
}

// ---------------------------------------------------------------------------
// Emscripten bindings
// ---------------------------------------------------------------------------
#ifdef __EMSCRIPTEN__
EMSCRIPTEN_BINDINGS(da_pipeline_wasm) {
    emscripten::function("runPipeline", &runPipeline);
    emscripten::function("parseNotams", &parseNotams);
    emscripten::function("parseNtms", &parseNtms);
    emscripten::function("parseTleCatalog", &parseTleCatalog);
    emscripten::function("correlateZones", &correlateZones);
    emscripten::function("generateTrajectoryFamily", &generateTrajectoryFamily);
    emscripten::function("assessAsatThreat", &assessAsatThreat);
}
#endif
