#include "tle_correlator/sgp4.h"
#include "tle_correlator/geometry.h"
#include "tle_correlator/correlator.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using namespace tle_correlator;

// ---- Test helpers -----------------------------------------------------------

static int tests_run    = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { \
        std::cout << "  TEST: " << #name << "... "; \
        tests_run++; \
    } while (0)

#define PASS() \
    do { \
        tests_passed++; \
        std::cout << "PASS" << std::endl; \
    } while (0)

#define ASSERT_TRUE(expr) \
    do { \
        if (!(expr)) { \
            std::cout << "FAIL at line " << __LINE__ << ": " << #expr << std::endl; \
            return; \
        } \
    } while (0)

#define ASSERT_EQ(a, b) \
    do { \
        if ((a) != (b)) { \
            std::cout << "FAIL at line " << __LINE__ << ": " << #a << " != " << #b \
                      << " (" << (a) << " vs " << (b) << ")" << std::endl; \
            return; \
        } \
    } while (0)

#define ASSERT_NEAR(a, b, tol) \
    do { \
        if (std::abs((a) - (b)) > (tol)) { \
            std::cout << "FAIL at line " << __LINE__ << ": " << (a) << " != " << (b) \
                      << " (tol=" << (tol) << ")" << std::endl; \
            return; \
        } \
    } while (0)

// ---- Test TLE Data ----------------------------------------------------------

static const std::string TLE_NAME  = "ISS (ZARYA)";
static const std::string TLE_LINE1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
static const std::string TLE_LINE2 = "2 25544  51.6400 100.0000 0001234  90.0000 270.0000 15.49560000400000";

// Epoch: 2024-01-01 12:00:00 UTC -> Unix ~1704110400
static int64_t iss_epoch_unix() {
    return SGP4::tle_epoch_to_unix(24, 1.5);
}

// ---- SGP4 Tests -------------------------------------------------------------

void test_parse_tle() {
    TEST(parse_tle);

    TLE tle = SGP4::parse_tle(TLE_NAME, TLE_LINE1, TLE_LINE2);

    ASSERT_EQ(tle.name, "ISS (ZARYA)");
    ASSERT_EQ(tle.norad_id, 25544);
    ASSERT_EQ(tle.epoch_year, 24);
    ASSERT_NEAR(tle.epoch_day, 1.5, 0.001);
    ASSERT_NEAR(tle.inclination_deg, 51.64, 0.01);
    ASSERT_NEAR(tle.raan_deg, 100.0, 0.01);
    ASSERT_NEAR(tle.eccentricity, 0.0001234, 0.0001);
    ASSERT_NEAR(tle.arg_perigee_deg, 90.0, 0.01);
    ASSERT_NEAR(tle.mean_anomaly_deg, 270.0, 0.01);
    ASSERT_NEAR(tle.mean_motion, 15.4956, 0.01);
    ASSERT_TRUE(std::string(tle.intl_designator).find("98067A") != std::string::npos);

    PASS();
}

void test_tle_to_elements() {
    TEST(tle_to_elements);

    TLE tle = SGP4::parse_tle(TLE_NAME, TLE_LINE1, TLE_LINE2);
    OrbitalElements elems = SGP4::tle_to_elements(tle);

    // Semi-major axis: ISS is at ~6780 km -> ~1.063 Earth radii
    double a_km = elems.a * R_EARTH_KM;
    ASSERT_TRUE(a_km > 6700.0 && a_km < 6900.0);

    // Inclination in radians: 51.64 deg ~ 0.9013 rad
    ASSERT_NEAR(elems.i, 51.64 * DEG2RAD, 0.01);

    // Eccentricity
    ASSERT_NEAR(elems.e, 0.0001234, 0.0001);

    // Mean motion in rad/min: 15.4956 rev/day -> ~0.0676 rad/min
    double expected_n = 15.4956 * 2.0 * PI / MIN_PER_DAY;
    ASSERT_NEAR(elems.n, expected_n, 0.001);

    PASS();
}

void test_propagate() {
    TEST(propagate);

    TLE tle = SGP4::parse_tle(TLE_NAME, TLE_LINE1, TLE_LINE2);
    OrbitalElements elems = SGP4::tle_to_elements(tle);

    // Propagate at epoch (tsince = 0)
    Vec3 pos, vel;
    SGP4::propagate(elems, 0.0, pos, vel);

    double r = pos.norm();
    // ISS altitude ~410 km + Earth radius 6378 = ~6788 km
    ASSERT_TRUE(r > 6500.0 && r < 7000.0);

    // Velocity should be ~7.7 km/s
    double v = vel.norm();
    ASSERT_TRUE(v > 7.0 && v < 8.5);

    // Propagate 90 minutes (one orbit) - should still be in valid range
    Vec3 pos2, vel2;
    SGP4::propagate(elems, 90.0, pos2, vel2);
    double r2 = pos2.norm();
    ASSERT_TRUE(r2 > 6500.0 && r2 < 7000.0);

    // Positions should be different
    double dx = pos2.x - pos.x;
    double dy = pos2.y - pos.y;
    double dz = pos2.z - pos.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    ASSERT_TRUE(dist > 10.0); // Should have moved

    PASS();
}

void test_eci_to_geodetic() {
    TEST(eci_to_geodetic);

    // Object at x=7000, y=0, z=0 with GMST=0 -> lat~0, lon~0
    Vec3 equator = {7000.0, 0.0, 0.0};
    LatLon pos = SGP4::eci_to_geodetic(equator, 0.0);
    ASSERT_NEAR(pos.lat, 0.0, 1.0);
    ASSERT_NEAR(pos.lon, 0.0, 1.0);

    // Object at north pole: x=0, y=0, z=7000 -> lat~90
    Vec3 north = {0.0, 0.0, 7000.0};
    LatLon north_pos = SGP4::eci_to_geodetic(north, 0.0);
    ASSERT_NEAR(north_pos.lat, 90.0, 1.0);

    // With GMST = pi/2, longitude shifts by -90 degrees
    double gmst_90 = PI / 2.0;
    LatLon rotated = SGP4::eci_to_geodetic(equator, gmst_90);
    ASSERT_NEAR(rotated.lat, 0.0, 1.0);
    ASSERT_NEAR(rotated.lon, -90.0, 1.0);

    PASS();
}

void test_ground_track() {
    TEST(ground_track);

    TLE tle = SGP4::parse_tle(TLE_NAME, TLE_LINE1, TLE_LINE2);
    int64_t epoch = iss_epoch_unix();

    // Generate 1 orbit (~93 min = 5580 sec)
    auto track = SGP4::ground_track(tle, epoch, epoch + 5580, 60.0);

    // Should have ~93 points
    ASSERT_TRUE(track.size() > 80);
    ASSERT_TRUE(track.size() < 110);

    // Latitude should stay within inclination bounds
    for (const auto& gtp : track) {
        ASSERT_TRUE(gtp.position.lat >= -55.0 && gtp.position.lat <= 55.0);
        ASSERT_TRUE(gtp.altitude_km > 300.0 && gtp.altitude_km < 500.0);
    }

    // Find max latitude - should be close to inclination (51.6 deg)
    double max_lat = -90.0;
    for (const auto& gtp : track) {
        max_lat = std::max(max_lat, std::abs(gtp.position.lat));
    }
    // Over one orbit, should reach near max latitude
    ASSERT_TRUE(max_lat > 30.0);

    PASS();
}

// ---- Geometry Tests ---------------------------------------------------------

void test_point_in_polygon() {
    TEST(point_in_polygon);

    // Rectangle around Cape Canaveral
    std::vector<LatLon> poly = {
        {28.0, -81.0},
        {28.0, -80.0},
        {29.0, -80.0},
        {29.0, -81.0}
    };

    // Inside
    ASSERT_TRUE(Geometry::point_in_polygon({28.5, -80.5}, poly));

    // Outside (west)
    ASSERT_TRUE(!Geometry::point_in_polygon({28.5, -82.0}, poly));

    // Outside (north)
    ASSERT_TRUE(!Geometry::point_in_polygon({30.0, -80.5}, poly));

    // Outside (far away)
    ASSERT_TRUE(!Geometry::point_in_polygon({40.7, -74.0}, poly));

    // On the edge (south side) - may or may not be inside depending on algorithm
    // Just verify it doesn't crash
    Geometry::point_in_polygon({28.0, -80.5}, poly);

    PASS();
}

void test_point_in_circle() {
    TEST(point_in_circle);

    LatLon center = {28.5, -80.5};
    double radius_nm = 30.0;

    // Center is inside
    ASSERT_TRUE(Geometry::point_in_circle(center, center, radius_nm));

    // Nearby point (within 30 NM)
    ASSERT_TRUE(Geometry::point_in_circle({28.6, -80.4}, center, radius_nm));

    // Far point (New York)
    ASSERT_TRUE(!Geometry::point_in_circle({40.7, -74.0}, center, radius_nm));

    PASS();
}

void test_haversine() {
    TEST(haversine);

    // New York to London: ~5570 km / ~3007 NM
    LatLon ny = {40.7128, -74.0060};
    LatLon london = {51.5074, -0.1278};

    double dist_km = Geometry::haversine_km(ny, london);
    ASSERT_TRUE(dist_km > 5500.0 && dist_km < 5700.0);

    double dist_nm = Geometry::haversine_nm(ny, london);
    ASSERT_TRUE(dist_nm > 2900.0 && dist_nm < 3100.0);

    // Distance to self should be 0
    ASSERT_NEAR(Geometry::haversine_km(ny, ny), 0.0, 0.01);

    PASS();
}

void test_bearing() {
    TEST(bearing);

    // Due north: same longitude, different latitude
    LatLon a = {0.0, 0.0};
    LatLon b = {10.0, 0.0};
    double brng = Geometry::bearing_deg(a, b);
    ASSERT_NEAR(brng, 0.0, 1.0);

    // Due east: same latitude, different longitude
    LatLon c = {0.0, 10.0};
    double brng_e = Geometry::bearing_deg(a, c);
    ASSERT_NEAR(brng_e, 90.0, 1.0);

    // Due south
    LatLon d = {-10.0, 0.0};
    double brng_s = Geometry::bearing_deg(a, d);
    ASSERT_NEAR(brng_s, 180.0, 1.0);

    // Due west
    LatLon e = {0.0, -10.0};
    double brng_w = Geometry::bearing_deg(a, e);
    ASSERT_NEAR(brng_w, 270.0, 1.0);

    PASS();
}

// ---- Correlator Tests -------------------------------------------------------

void test_correlate_iss() {
    TEST(correlate_iss);

    TLE tle = SGP4::parse_tle(TLE_NAME, TLE_LINE1, TLE_LINE2);
    int64_t epoch = iss_epoch_unix();

    // Create a large circular exclusion zone at mid-latitude where ISS must pass
    // Use a wide zone to catch ISS ground track over 1 day
    ExclusionZone zone;
    zone.id = "MID_LAT_ZONE";
    zone.center = {30.0, -50.0};
    zone.radius_nm = 500.0;
    zone.is_circle = true;
    zone.effective_start = epoch;
    zone.effective_end = epoch + 86400; // 1 day
    zone.compute_bounds();

    Correlator correlator;
    auto results = correlator.correlate(zone, {tle}, 60.0);

    // ISS at 51.6 incl should correlate with a zone at 30N
    ASSERT_TRUE(!results.empty());
    ASSERT_EQ(results[0].zone_id, "MID_LAT_ZONE");
    ASSERT_EQ(results[0].norad_id, 25544);
    ASSERT_TRUE(results[0].confidence > 0.1);
    ASSERT_TRUE(results[0].inclination_match > 0.5);

    PASS();
}

void test_no_correlation() {
    TEST(no_correlation);

    TLE tle = SGP4::parse_tle(TLE_NAME, TLE_LINE1, TLE_LINE2);
    int64_t epoch = iss_epoch_unix();

    // Zone at 70N - ISS (51.6 incl) cannot reach it
    ExclusionZone zone;
    zone.id = "ARCTIC_ZONE";
    zone.center = {75.0, 0.0};
    zone.radius_nm = 30.0;
    zone.is_circle = true;
    zone.effective_start = epoch;
    zone.effective_end = epoch + 86400;
    zone.compute_bounds();

    Correlator correlator;
    auto results = correlator.correlate(zone, {tle}, 60.0);

    // Should be empty (75N is well beyond 51.6 incl + 5 deg margin)
    ASSERT_TRUE(results.empty());

    PASS();
}

void test_confidence_scoring() {
    TEST(confidence_scoring);

    // Confidence should decrease with distance
    double c1 = Correlator::compute_confidence(0.0, 100.0, 0.9);
    double c2 = Correlator::compute_confidence(50.0, 100.0, 0.9);
    double c3 = Correlator::compute_confidence(200.0, 100.0, 0.9);
    double c4 = Correlator::compute_confidence(1000.0, 100.0, 0.9);

    ASSERT_TRUE(c1 > c2);
    ASSERT_TRUE(c2 > c3);
    ASSERT_TRUE(c3 > c4);

    // Zero distance should give high confidence
    ASSERT_TRUE(c1 > 0.8);

    // Large distance should give low confidence
    ASSERT_TRUE(c4 < 0.5);

    // Higher inclination match should give higher confidence
    double c_high = Correlator::compute_confidence(50.0, 100.0, 1.0);
    double c_low = Correlator::compute_confidence(50.0, 100.0, 0.2);
    ASSERT_TRUE(c_high > c_low);

    PASS();
}

void test_batch_correlate() {
    TEST(batch_correlate);

    TLE iss = SGP4::parse_tle(TLE_NAME, TLE_LINE1, TLE_LINE2);

    // Second TLE: a different object
    std::string sl_name = "STARLINK-1007";
    std::string sl_l1 = "1 44713U 19074A   24001.50000000  .00002300  00000-0  16000-4 0  9994";
    std::string sl_l2 = "2 44713  53.0500 120.3400 0001500  80.0000 280.1000 15.06400000200000";
    TLE starlink = SGP4::parse_tle(sl_name, sl_l1, sl_l2);

    int64_t epoch = iss_epoch_unix();

    // Two zones
    ExclusionZone zone1;
    zone1.id = "ZONE_A";
    zone1.center = {30.0, -80.0};
    zone1.radius_nm = 300.0;
    zone1.is_circle = true;
    zone1.effective_start = epoch;
    zone1.effective_end = epoch + 86400;
    zone1.compute_bounds();

    ExclusionZone zone2;
    zone2.id = "ZONE_B";
    zone2.center = {40.0, 10.0};
    zone2.radius_nm = 300.0;
    zone2.is_circle = true;
    zone2.effective_start = epoch;
    zone2.effective_end = epoch + 86400;
    zone2.compute_bounds();

    Correlator correlator;
    auto results = correlator.batch_correlate({zone1, zone2}, {iss, starlink});

    // Should have some correlations (both objects have ~52 deg inclination)
    ASSERT_TRUE(results.size() >= 2);

    // Results should be sorted by confidence
    for (size_t i = 1; i < results.size(); ++i) {
        ASSERT_TRUE(results[i - 1].confidence >= results[i].confidence);
    }

    PASS();
}

void test_launch_time_prediction() {
    TEST(launch_time_prediction);

    TLE tle = SGP4::parse_tle(TLE_NAME, TLE_LINE1, TLE_LINE2);
    int64_t epoch = iss_epoch_unix();

    // Cape Canaveral zone
    ExclusionZone zone;
    zone.id = "CAPE";
    zone.center = {28.5, -80.6};
    zone.radius_nm = 30.0;
    zone.is_circle = true;
    zone.effective_start = epoch;
    zone.effective_end = epoch + 86400;
    zone.compute_bounds();

    Correlator correlator;
    auto pred = correlator.predict_launch_time(zone, tle);

    // Should find an ascending node crossing
    ASSERT_TRUE(pred.launch_time > 0);
    ASSERT_TRUE(pred.launch_time >= epoch);
    ASSERT_TRUE(pred.launch_time <= epoch + 86400);
    ASSERT_TRUE(pred.confidence >= 0.0 && pred.confidence <= 1.0);
    ASSERT_TRUE(!pred.rationale.empty());
    ASSERT_EQ(pred.norad_id, 25544);

    PASS();
}

// ---- Main -------------------------------------------------------------------

int main() {
    std::cout << "=== Exclusion Zone TLE Correlator Plugin Tests ===" << std::endl;
    std::cout << std::endl;

    std::cout << "[SGP4]" << std::endl;
    test_parse_tle();
    test_tle_to_elements();
    test_propagate();
    test_eci_to_geodetic();
    test_ground_track();

    std::cout << std::endl;
    std::cout << "[Geometry]" << std::endl;
    test_point_in_polygon();
    test_point_in_circle();
    test_haversine();
    test_bearing();

    std::cout << std::endl;
    std::cout << "[Correlator]" << std::endl;
    test_correlate_iss();
    test_no_correlation();
    test_confidence_scoring();
    test_batch_correlate();
    test_launch_time_prediction();

    std::cout << std::endl;
    std::cout << "=== Results: " << tests_passed << "/" << tests_run
              << " passed ===" << std::endl;

    return (tests_passed == tests_run) ? 0 : 1;
}
