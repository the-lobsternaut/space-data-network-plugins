#include <iostream>
#include <cmath>
#include <cassert>
#include <string>
#include <vector>

#include "da_pipeline/types.h"
#include "da_pipeline/notam_parser.h"
#include "da_pipeline/ntm_parser.h"
#include "da_pipeline/exclusion_zone.h"
#include "da_pipeline/sgp4.h"
#include "da_pipeline/tle_correlator.h"
#include "da_pipeline/trajectory.h"
#include "da_pipeline/ascent_estimator.h"
#include "da_pipeline/engagement.h"
#include "da_pipeline/pipeline.h"

using namespace da_pipeline;

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------
static int tests_run = 0, tests_passed = 0;

#define TEST(name) \
    void name(); \
    struct name##_reg { \
        name##_reg() { \
            tests_run++; \
            std::cout << "  " #name "... "; \
            try { name(); tests_passed++; std::cout << "PASS\n"; } \
            catch (const std::exception& e) { std::cout << "FAIL: " << e.what() << "\n"; } \
            catch (...) { std::cout << "FAIL\n"; } \
        } \
    } name##_instance; \
    void name()

#define ASSERT_TRUE(x) \
    do { if (!(x)) throw std::runtime_error("ASSERT_TRUE failed: " #x); } while(0)

#define ASSERT_FALSE(x) \
    do { if ((x)) throw std::runtime_error("ASSERT_FALSE failed: " #x); } while(0)

#define ASSERT_NEAR(a, b, tol) \
    do { if (std::abs((a)-(b)) > (tol)) { \
        throw std::runtime_error( \
            std::string("ASSERT_NEAR failed: ") + std::to_string(a) + " vs " + std::to_string(b) + \
            " (tol=" + std::to_string(tol) + ")"); \
    } } while(0)

#define ASSERT_EQ(a, b) \
    do { if ((a) != (b)) { \
        throw std::runtime_error("ASSERT_EQ failed"); \
    } } while(0)

// ---------------------------------------------------------------------------
// Test data
// ---------------------------------------------------------------------------
static const char* ISS_TLE_NAME = "ISS (ZARYA)";
static const char* ISS_TLE_LINE1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
static const char* ISS_TLE_LINE2 = "2 25544  51.6400 100.0000 0007000  30.0000 330.0000 15.50000000    09";

static const char* SPACEX_NOTAM =
    "!FDC 4/3200 ZJX FL..AIRSPACE SPACE OPERATIONS AREA\n"
    "283000N/0803500W OR 28-30-00N/080-35-00W\n"
    "EFFECTIVE 2401150000-2401152359\n"
    "SFC-UNL\n"
    "30NM RADIUS OF 283000N/0803500W\n"
    "SPACE LAUNCH OPERATIONS SPACEX FALCON 9\n"
    "14 CFR 91.143";

static const char* HYDROLANT_NTM =
    "HYDROLANT 1234/2024\n"
    "WESTERN ATLANTIC\n"
    "SPACE OPERATIONS - ROCKET DEBRIS\n"
    "151200Z MAR 2024 TO 151800Z MAR 2024\n"
    "28-30.0N 076-00.0W\n"
    "28-30.0N 074-00.0W\n"
    "27-00.0N 074-00.0W\n"
    "27-00.0N 076-00.0W\n"
    "CANCEL NONE\n"
    "AUTHORITY: CCSFS\n"
    "NNNN";

static const char* ISS_CATALOG =
    "ISS (ZARYA)\n"
    "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002\n"
    "2 25544  51.6400 100.0000 0007000  30.0000 330.0000 15.50000000    09\n";

// ============================================================================
// 1. NOTAM PARSING TESTS
// ============================================================================

TEST(test_parse_faa_notam) {
    NotamParser parser;
    auto result = parser.parse(SPACEX_NOTAM);
    ASSERT_TRUE(result.has_value());
    auto& n = *result;
    // Should extract coordinates
    ASSERT_NEAR(n.center_lat, 28.5, 0.1);
    ASSERT_NEAR(n.center_lon, -80.583, 0.1);
    // Should extract radius
    ASSERT_NEAR(n.radius_nm, 30.0, 0.1);
    // Should classify as LAUNCH
    ASSERT_TRUE(n.classification == NotamClassification::LAUNCH);
}

TEST(test_parse_icao_notam) {
    NotamParser parser;
    std::string icao =
        "A0123/24 NOTAMN\n"
        "Q) KZJX/QXXXX/IV/NBO/AE/000/999/2830N08035W030\n"
        "B) 2401150000\n"
        "C) 2401152359\n"
        "E) SPACE LAUNCH OPERATIONS ROCKET LAUNCH";
    auto result = parser.parse_icao(icao);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->notam_id, std::string("A0123/24"));
    ASSERT_TRUE(result->classification == NotamClassification::LAUNCH);
    ASSERT_TRUE(result->effective_start > 0);
    ASSERT_TRUE(result->effective_end > result->effective_start);
}

TEST(test_classify_launch) {
    NotamParser parser;
    NOTAM n;
    n.notam_text = "SPACE LAUNCH OPERATIONS SPACEX FALCON 9";
    auto cls = parser.classify(n);
    ASSERT_TRUE(cls == NotamClassification::LAUNCH);

    NOTAM n2;
    n2.notam_text = "ULA ATLAS V SPACE OPERATIONS";
    ASSERT_TRUE(parser.classify(n2) == NotamClassification::LAUNCH);

    NOTAM n3;
    n3.notam_text = "14 CFR 91.143 ROCKET LAUNCH";
    ASSERT_TRUE(parser.classify(n3) == NotamClassification::LAUNCH);
}

TEST(test_classify_hazard) {
    NotamParser parser;
    NOTAM n;
    n.notam_text = "ROCKET DEBRIS HAZARDOUS OPERATIONS BOOSTER RECOVERY ZONE";
    auto cls = parser.classify(n);
    ASSERT_TRUE(cls == NotamClassification::HAZARD);
}

TEST(test_notam_to_zone) {
    NotamParser parser;
    auto result = parser.parse(SPACEX_NOTAM);
    ASSERT_TRUE(result.has_value());
    auto zone = parser.notam_to_zone(*result);
    ASSERT_TRUE(zone.type == GeometryType::CIRCLE);
    ASSERT_NEAR(zone.circle.radius_nm, 30.0, 0.1);
    ASSERT_NEAR(zone.circle.center.lat, 28.5, 0.1);
    // Bounds should be computed
    ASSERT_TRUE(zone.min_lat < zone.max_lat);
    ASSERT_TRUE(zone.min_lon < zone.max_lon);
}

// ============================================================================
// 2. NTM PARSING TESTS
// ============================================================================

TEST(test_parse_hydrolant) {
    NtmParser parser;
    auto result = parser.parse(HYDROLANT_NTM);
    ASSERT_TRUE(result.has_value());
    auto& m = *result;
    ASSERT_EQ(m.notice_id, std::string("1234/2024"));
    ASSERT_TRUE(m.area == BroadcastArea::HYDROLANT);
    ASSERT_TRUE(m.classification == HazardClassification::ROCKET_DEBRIS);
    ASSERT_TRUE(m.zone.type == GeometryType::POLYGON);
    ASSERT_EQ(m.zone.polygon.vertices.size(), 4u);
    ASSERT_NEAR(m.zone.polygon.vertices[0].lat, 28.5, 0.1);
}

TEST(test_parse_hydropac_circle) {
    NtmParser parser;
    std::string ntm =
        "HYDROPAC 5678/2024\n"
        "PACIFIC OCEAN\n"
        "SPACE OPERATIONS - FAIRING RECOVERY\n"
        "201400Z APR 2024 TO 202000Z APR 2024\n"
        "WITHIN 30 NM OF 32-00.0N 120-30.0W\n"
        "NNNN";
    auto result = parser.parse(ntm);
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->area == BroadcastArea::HYDROPAC);
    ASSERT_TRUE(result->zone.type == GeometryType::CIRCLE);
    ASSERT_NEAR(result->zone.circle.center.lat, 32.0, 0.1);
    ASSERT_NEAR(result->zone.circle.center.lon, -120.5, 0.1);
    ASSERT_NEAR(result->zone.circle.radius_nm, 30.0, 0.1);
}

TEST(test_parse_ntm_batch) {
    NtmParser parser;
    std::string batch = std::string(HYDROLANT_NTM) + "\n" +
        "HYDROLANT 5555/2024\n"
        "WESTERN ATLANTIC\n"
        "SPACE OPERATIONS - SPLASHDOWN\n"
        "201400Z MAR 2024 TO 202000Z MAR 2024\n"
        "30-00.0N 070-00.0W\n"
        "30-00.0N 068-00.0W\n"
        "29-00.0N 068-00.0W\n"
        "29-00.0N 070-00.0W\n"
        "NNNN";
    auto results = parser.parse_batch(batch);
    ASSERT_TRUE(results.size() >= 2);
}

TEST(test_coordinate_parsing) {
    // NTM coordinate format: DD-MM.MN
    ASSERT_NEAR(NtmParser::parse_lat("28-30.0N"), 28.5, 0.001);
    ASSERT_NEAR(NtmParser::parse_lat("33-15.0S"), -33.25, 0.001);
    ASSERT_NEAR(NtmParser::parse_lon("080-20.0W"), -80.333, 0.01);
    ASSERT_NEAR(NtmParser::parse_lon("110-45.0E"), 110.75, 0.001);
}

// ============================================================================
// 3. EXCLUSION ZONE TESTS
// ============================================================================

TEST(test_point_in_polygon) {
    ExclusionZoneAnalyzer ez;
    ExclusionZone zone;
    zone.type = GeometryType::POLYGON;
    zone.polygon.vertices = {
        {0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}
    };

    ASSERT_TRUE(ez.point_in_zone({5.0, 5.0}, zone));
    ASSERT_FALSE(ez.point_in_zone({15.0, 5.0}, zone));
    ASSERT_FALSE(ez.point_in_zone({-1.0, 5.0}, zone));
}

TEST(test_point_in_circle) {
    ExclusionZoneAnalyzer ez;
    ExclusionZone zone;
    zone.type = GeometryType::CIRCLE;
    zone.circle.center = {28.5, -80.5};
    zone.circle.radius_nm = 30.0;

    // Point at center — definitely inside
    ASSERT_TRUE(ez.point_in_zone({28.5, -80.5}, zone));
    // Point far away
    ASSERT_FALSE(ez.point_in_zone({40.0, -80.5}, zone));
}

TEST(test_zone_grouping) {
    ExclusionZoneAnalyzer ez;

    // Create a NOTAM zone near Cape Canaveral
    ExclusionZone nz;
    nz.id = "NOTAM-1";
    nz.type = GeometryType::CIRCLE;
    nz.circle.center = {28.5, -80.5};
    nz.circle.radius_nm = 30.0;
    nz.compute_bounds();
    nz.effective_start = 1700000000;
    nz.effective_end = 1700100000;

    // Create an NTM zone downrange
    ExclusionZone mz;
    mz.id = "NTM-1";
    mz.type = GeometryType::POLYGON;
    mz.polygon.vertices = {
        {28.5, -76.0}, {28.5, -74.0}, {27.0, -74.0}, {27.0, -76.0}
    };
    mz.compute_bounds();
    mz.effective_start = 1700000000;
    mz.effective_end = 1700100000;

    auto groups = ez.group_zones({nz}, {mz}, {});
    // Should find at least one group (Cape Canaveral or KSC)
    ASSERT_TRUE(!groups.empty());

    // The group should contain our zones
    bool found_cape = false;
    for (auto& g : groups) {
        if (g.associated_site.site_id == "CCAFS" || g.associated_site.site_id == "KSC") {
            if (!g.notam_zones.empty() && !g.ntm_zones.empty()) {
                found_cape = true;
            }
        }
    }
    ASSERT_TRUE(found_cape);
}

TEST(test_azimuth_estimation) {
    ExclusionZoneAnalyzer ez;

    ExclusionZoneGroup group;
    group.associated_site = {"CCAFS", "Cape Canaveral AFS", 28.4889, -80.5778, {90}};

    // Add a downrange zone to the east
    ExclusionZone z;
    z.type = GeometryType::CIRCLE;
    z.circle.center = {28.5, -70.0};  // ~10 degrees east
    z.circle.radius_nm = 50;
    group.ntm_zones.push_back(z);

    double az = ez.estimate_azimuth(group);
    // Bearing from Cape to point due east should be roughly 89-91 degrees
    ASSERT_NEAR(az, 89.5, 5.0);
}

TEST(test_activity_classification) {
    ExclusionZoneAnalyzer ez;

    ExclusionZoneGroup g1;
    // Both NOTAM and NTM zones → launch preparation
    ExclusionZone nz; nz.type = GeometryType::CIRCLE;
    ExclusionZone mz; mz.type = GeometryType::CIRCLE;
    g1.notam_zones = {nz};
    g1.ntm_zones = {mz};
    ASSERT_TRUE(ez.classify_activity(g1) == ActivityType::LAUNCH_PREPARATION);

    ExclusionZoneGroup g2;
    g2.notam_zones = {nz, nz};
    g2.ntm_zones = {mz, mz};
    ASSERT_TRUE(ez.classify_activity(g2) == ActivityType::LAUNCH_DETECTED);
}

// ============================================================================
// 4. SGP4/TLE TESTS
// ============================================================================

TEST(test_parse_tle) {
    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    ASSERT_EQ(tle.name, std::string("ISS (ZARYA)"));
    ASSERT_EQ(tle.norad_id, 25544);
    ASSERT_NEAR(tle.inclination_deg, 51.64, 0.01);
    ASSERT_NEAR(tle.eccentricity, 0.0007, 0.0001);
    ASSERT_NEAR(tle.mean_motion, 15.5, 0.1);
    ASSERT_EQ(tle.epoch_year, 24);
    ASSERT_NEAR(tle.epoch_day, 1.5, 0.1);
}

TEST(test_parse_catalog) {
    std::string catalog = std::string(ISS_TLE_NAME) + "\n" +
                          ISS_TLE_LINE1 + "\n" +
                          ISS_TLE_LINE2 + "\n" +
                          "COSMOS 2251 DEB\n"
                          "1 34454U 93036SX  24001.00000000  .00000100  00000-0  10000-3 0  9999\n"
                          "2 34454  74.0400 200.0000 0010000  50.0000 310.0000 14.40000000 99999\n";
    auto tles = SGP4::parse_catalog(catalog);
    ASSERT_EQ(tles.size(), 2u);
    ASSERT_EQ(tles[0].norad_id, 25544);
    ASSERT_EQ(tles[1].norad_id, 34454);
}

TEST(test_propagate_at_epoch) {
    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    auto elems = SGP4::tle_to_elements(tle);

    Vec3 pos, vel;
    SGP4::propagate(elems, 0.0, pos, vel); // At epoch

    double r = pos.norm();
    // ISS should be at ~6370+420 = ~6790 km from Earth center
    ASSERT_NEAR(r, 6790, 50);

    double v = vel.norm();
    // ISS velocity ~7.66 km/s
    ASSERT_NEAR(v, 7.66, 0.3);
}

TEST(test_ground_track) {
    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);

    // One full orbit (~92 min)
    auto track = SGP4::ground_track(tle, epoch, epoch + 5520, 60.0);
    ASSERT_TRUE(track.size() > 80); // At least 80 points (5520/60 = 92)

    // All latitudes should be within [-52, 52] for 51.64 deg inclination
    for (auto& pt : track) {
        ASSERT_TRUE(pt.position.lat >= -55 && pt.position.lat <= 55);
        ASSERT_TRUE(pt.altitude_km > 350 && pt.altitude_km < 500);
    }
}

// ============================================================================
// 5. CORRELATION TESTS
// ============================================================================

TEST(test_correlate_zone_tle) {
    // Create a zone at mid-latitude that ISS should cross
    ExclusionZone zone;
    zone.id = "TEST-ZONE";
    zone.type = GeometryType::CIRCLE;
    zone.circle.center = {28.5, -80.5}; // Cape Canaveral area
    zone.circle.radius_nm = 200;        // Large zone
    zone.compute_bounds();

    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
    zone.effective_start = epoch;
    zone.effective_end = epoch + 86400; // 24 hours

    TLECorrelator correlator;
    auto corr = correlator.correlate_single(zone, tle, 30.0);

    // ISS at 51.6 deg should definitely cross 28.5N latitude
    ASSERT_TRUE(corr.confidence > 0.1);
    ASSERT_EQ(corr.norad_id, 25544);
    // Min distance should be small for such a large zone
    ASSERT_TRUE(corr.min_distance_km < 500);
}

TEST(test_no_correlation_high_lat) {
    // Zone at high latitude beyond ISS inclination
    ExclusionZone zone;
    zone.id = "HIGH-LAT";
    zone.type = GeometryType::CIRCLE;
    zone.circle.center = {80.0, 0.0}; // 80 N — beyond 51.6 deg inclination
    zone.circle.radius_nm = 50;
    zone.compute_bounds();

    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
    zone.effective_start = epoch;
    zone.effective_end = epoch + 86400;

    TLECorrelator correlator;
    auto corr = correlator.correlate_single(zone, tle, 60.0);

    // ISS never reaches 80N — confidence should be very low
    ASSERT_TRUE(corr.min_distance_km > 2000);
    ASSERT_TRUE(corr.confidence < 0.3);
}

TEST(test_batch_correlate) {
    ExclusionZone z1, z2;
    z1.id = "Z1"; z1.type = GeometryType::CIRCLE;
    z1.circle.center = {28.5, -80.5}; z1.circle.radius_nm = 100;
    z1.compute_bounds();

    z2.id = "Z2"; z2.type = GeometryType::CIRCLE;
    z2.circle.center = {45.0, -75.0}; z2.circle.radius_nm = 100;
    z2.compute_bounds();

    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
    z1.effective_start = z2.effective_start = epoch;
    z1.effective_end = z2.effective_end = epoch + 86400;

    TLECorrelator correlator;
    auto results = correlator.batch_correlate({z1, z2}, {tle});
    // Both zones should produce correlations for ISS
    ASSERT_TRUE(results.size() >= 2);
}

// ============================================================================
// 6. TRAJECTORY TESTS
// ============================================================================

TEST(test_simulate_single_stage) {
    AscentModelParams params;
    params.launch_lat = 28.5;
    params.launch_lon = -80.5;
    params.launch_alt = 0;
    params.launch_azimuth = 90;
    params.initial_pitch = 90;

    AscentModelParams::StageParams s1;
    s1.thrust = 1000000;  // 1 MN
    s1.isp = 300;
    s1.mass_initial = 50000;
    s1.mass_final = 10000;
    s1.burn_time = 120;
    s1.pitch_rate = 0.5;
    params.stages = {s1};

    TrajectorySimulator sim;
    auto traj = sim.simulate(params, 130, 0.5);
    ASSERT_TRUE(!traj.empty());

    // Should reach significant altitude
    double max_alt = 0;
    for (auto& sv : traj) {
        if (sv.altitude_m > max_alt) max_alt = sv.altitude_m;
    }
    ASSERT_TRUE(max_alt > 50000); // > 50 km
}

TEST(test_simulate_two_stage) {
    AscentModelParams params = AscentEstimator::default_vehicle(28.5, -80.5, 90.0);

    TrajectorySimulator sim;
    auto traj = sim.simulate(params, 600, 1.0);
    ASSERT_TRUE(!traj.empty());
    ASSERT_TRUE(traj.size() > 100);

    // Two-stage F9-like should reach very high altitude
    double max_alt = 0;
    double max_speed = 0;
    for (auto& sv : traj) {
        if (sv.altitude_m > max_alt) max_alt = sv.altitude_m;
        if (sv.speed_mps > max_speed) max_speed = sv.speed_mps;
    }
    // Should reach well above 100 km
    ASSERT_TRUE(max_alt > 100000);
    // Should reach several km/s
    ASSERT_TRUE(max_speed > 2000);
}

TEST(test_geodetic_ecef_roundtrip) {
    double lat = 28.5, lon = -80.5, alt = 1000.0;
    Vec3 ecef = TrajectorySimulator::geodetic_to_ecef(lat, lon, alt);
    LatLonAlt result = TrajectorySimulator::ecef_to_geodetic(ecef);

    ASSERT_NEAR(result.lat, lat, 0.001);
    ASSERT_NEAR(result.lon, lon, 0.001);
    ASSERT_NEAR(result.alt, alt, 1.0);
}

TEST(test_trajectory_family) {
    LaunchSite site = {"CCAFS", "Cape Canaveral AFS", 28.4889, -80.5778, {90}};
    AscentEstimator estimator;
    auto family = estimator.generate_family(site, 90.0, 10.0);

    ASSERT_TRUE(!family.nominal_trajectory.empty());
    ASSERT_NEAR(family.min_azimuth_deg, 80.0, 0.1);
    ASSERT_NEAR(family.max_azimuth_deg, 100.0, 0.1);
    ASSERT_EQ(family.dispersions.size(), 5u);
    // Each dispersion should have trajectory points
    for (auto& d : family.dispersions) {
        ASSERT_TRUE(!d.empty());
    }
    // Estimated inclination for 90-deg azimuth from 28.5N
    // cos(i) = sin(az)*cos(lat) = sin(90)*cos(28.5) = cos(28.5) ≈ 0.879
    // i ≈ 28.5 deg
    ASSERT_NEAR(family.estimated_inclination_deg, 28.5, 2.0);
}

// ============================================================================
// 7. ENGAGEMENT TESTS
// ============================================================================

TEST(test_default_interceptor) {
    auto params = EngagementAnalyzer::default_interceptor();
    ASSERT_EQ(params.num_stages, 3);
    ASSERT_EQ(params.stage_thrust_kn.size(), 3u);
    ASSERT_EQ(params.stage_burn_time_sec.size(), 3u);
    ASSERT_TRUE(params.kill_vehicle_mass_kg > 0);
    ASSERT_TRUE(params.max_lateral_accel_g > 0);

    // Check max engagement altitude
    EngagementAnalyzer analyzer;
    double max_alt = analyzer.max_engagement_altitude(params);
    ASSERT_TRUE(max_alt > 100);  // Should reach > 100 km
}

TEST(test_evaluate_engagement) {
    LaunchSite site = {"CCAFS", "Cape Canaveral AFS", 28.4889, -80.5778, {90}};
    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    auto params = EngagementAnalyzer::default_interceptor();

    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);

    EngagementAnalyzer analyzer;
    // Try multiple launch times to find a feasible one
    bool found_feasible = false;
    for (Timestamp t = epoch; t < epoch + 86400; t += 300) {
        auto result = analyzer.evaluate_single(site, tle, params, t);
        if (result.feasibility == Feasibility::FEASIBLE) {
            found_feasible = true;
            ASSERT_TRUE(result.total_delta_v_kms > 0);
            ASSERT_TRUE(result.time_of_flight_sec > 0);
            ASSERT_EQ(result.target_norad_id, 25544);
            break;
        }
    }
    // ISS passes over Cape Canaveral regularly, so we should find a feasible engagement
    ASSERT_TRUE(found_feasible);
}

TEST(test_threat_assessment) {
    LaunchSite site = {"CCAFS", "Cape Canaveral AFS", 28.4889, -80.5778, {90}};
    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);

    auto params = EngagementAnalyzer::default_interceptor();
    EngagementAnalyzer analyzer;

    auto ta = analyzer.assess_threat(site, {tle}, params, epoch, epoch + 86400);
    ASSERT_EQ(ta.site_id, std::string("CCAFS"));
    // Should have a rationale string
    ASSERT_TRUE(!ta.rationale.empty());
    // Threat level should be at least LOW
    ASSERT_TRUE(ta.level >= ThreatLevel::LOW);
}

// ============================================================================
// 8. PIPELINE INTEGRATION TESTS
// ============================================================================

TEST(test_full_pipeline) {
    DomainAwarenessPipeline pipeline;

    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
    pipeline.set_analysis_window(epoch, epoch + 86400);

    auto sa = pipeline.run_full_pipeline(SPACEX_NOTAM, HYDROLANT_NTM, ISS_CATALOG);

    // Should have parsed inputs
    ASSERT_TRUE(!sa.parsed_notams.empty());
    ASSERT_TRUE(!sa.parsed_ntms.empty());
    ASSERT_TRUE(!sa.tle_catalog.empty());
    ASSERT_EQ(sa.tle_catalog.size(), 1u);
    ASSERT_EQ(sa.tle_catalog[0].norad_id, 25544);
}

TEST(test_pipeline_report) {
    DomainAwarenessPipeline pipeline;
    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
    pipeline.set_analysis_window(epoch, epoch + 86400);

    auto sa = pipeline.run_full_pipeline(SPACEX_NOTAM, HYDROLANT_NTM, ISS_CATALOG);

    ASSERT_TRUE(sa.generated_at > 0);
    ASSERT_TRUE(!sa.summary_text.empty());
    // Summary should mention domain awareness
    ASSERT_TRUE(sa.summary_text.find("Domain Awareness") != std::string::npos);
}

TEST(test_left_of_launch) {
    DomainAwarenessPipeline pipeline;
    auto tle = SGP4::parse_tle(ISS_TLE_NAME, ISS_TLE_LINE1, ISS_TLE_LINE2);
    Timestamp epoch = SGP4::tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
    pipeline.set_analysis_window(epoch, epoch + 86400);

    pipeline.ingest_notams(SPACEX_NOTAM);
    pipeline.ingest_ntms(HYDROLANT_NTM);
    pipeline.ingest_tle_catalog(ISS_CATALOG);
    pipeline.analyze_exclusion_zones();
    pipeline.generate_trajectory_families();

    auto sa = pipeline.generate_report();

    // If zone groups were found near Cape Canaveral with launch activity,
    // trajectory families should be generated
    if (!sa.zone_groups.empty()) {
        bool has_launch_site = false;
        for (auto& g : sa.zone_groups) {
            if (g.activity == ActivityType::LAUNCH_PREPARATION ||
                g.activity == ActivityType::LAUNCH_DETECTED) {
                has_launch_site = true;
            }
        }
        if (has_launch_site) {
            ASSERT_TRUE(!sa.trajectory_families.empty());
        }
    }
}

TEST(test_right_of_launch) {
    DomainAwarenessPipeline pipeline;

    // Create a synthetic tracklet
    Tracklet tracklet;
    tracklet.tracklet_id = "TRK-001";
    tracklet.sensor_id = "RADAR-1";
    tracklet.sensor_location = {28.5, -80.5, 0};
    tracklet.epoch_unix = 1704067200;

    // Simulate some ascending observations
    Vec3 launch_pos = TrajectorySimulator::geodetic_to_ecef(28.5, -80.5, 0);
    for (int i = 0; i < 10; i++) {
        Observation obs;
        obs.time = 60.0 + i * 10.0; // Start at T+60s, every 10s
        double alt = 5000 + i * 15000; // Ascending
        double east_offset = i * 2000;
        obs.position = TrajectorySimulator::geodetic_to_ecef(28.5, -80.5 + east_offset / 111000.0, alt);
        obs.has_position = true;
        obs.weight = 1.0;
        tracklet.observations.push_back(obs);
    }

    pipeline.ingest_tracklet(tracklet);
    auto sa = pipeline.generate_report();
    ASSERT_TRUE(!sa.reconstructions.empty());
    auto& recon = sa.reconstructions[0];
    // Should have estimated a launch site near Cape Canaveral
    ASSERT_NEAR(recon.estimated_launch_site.lat, 28.5, 2.0);
}

// ============================================================================
// Additional tests for math correctness
// ============================================================================

TEST(test_haversine_known_distance) {
    ExclusionZoneAnalyzer ez;
    // New York (40.7128, -74.0060) to Los Angeles (34.0522, -118.2437)
    // Known distance: ~3944 km
    LatLon ny = {40.7128, -74.0060};
    LatLon la = {34.0522, -118.2437};
    double dist = ez.haversine_km(ny, la);
    ASSERT_NEAR(dist, 3944, 50);
}

TEST(test_bearing_known) {
    ExclusionZoneAnalyzer ez;
    // From equator/prime meridian going due north
    LatLon a = {0.0, 0.0};
    LatLon b = {10.0, 0.0};
    double brng = ez.bearing_deg(a, b);
    ASSERT_NEAR(brng, 0.0, 1.0); // Due north

    // Due east
    LatLon c = {0.0, 10.0};
    double brng_e = ez.bearing_deg(a, c);
    ASSERT_NEAR(brng_e, 90.0, 1.0);
}

TEST(test_julian_date) {
    // J2000.0: 2000 Jan 1 12:00 TT = JD 2451545.0
    double jd = SGP4::julian_date(2000, 1, 1, 12, 0, 0.0);
    ASSERT_NEAR(jd, 2451545.0, 0.001);
}

TEST(test_vec3_operations) {
    Vec3 a = {1, 2, 3};
    Vec3 b = {4, 5, 6};

    Vec3 c = a + b;
    ASSERT_NEAR(c.x, 5, 1e-10);
    ASSERT_NEAR(c.y, 7, 1e-10);
    ASSERT_NEAR(c.z, 9, 1e-10);

    double d = a.dot(b);
    ASSERT_NEAR(d, 32, 1e-10);

    Vec3 cr = a.cross(b);
    ASSERT_NEAR(cr.x, -3, 1e-10);
    ASSERT_NEAR(cr.y, 6, 1e-10);
    ASSERT_NEAR(cr.z, -3, 1e-10);

    ASSERT_NEAR(a.norm(), std::sqrt(14.0), 1e-10);
}

TEST(test_atm_density) {
    // Sea level should be ~1.225 kg/m^3
    double rho0 = TrajectorySimulator::atm_density(0);
    ASSERT_NEAR(rho0, 1.225, 0.01);

    // Density should decrease with altitude
    double rho10 = TrajectorySimulator::atm_density(10000);
    double rho50 = TrajectorySimulator::atm_density(50000);
    ASSERT_TRUE(rho10 < rho0);
    ASSERT_TRUE(rho50 < rho10);

    // Above 1000 km should be essentially zero
    double rho_high = TrajectorySimulator::atm_density(1100000);
    ASSERT_NEAR(rho_high, 0, 1e-10);
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "\n=== Domain Awareness Pipeline Tests ===\n\n";
    // Tests are auto-registered and run by static initialization
    std::cout << "\n" << tests_passed << "/" << tests_run << " tests passed\n";
    return tests_passed == tests_run ? 0 : 1;
}
