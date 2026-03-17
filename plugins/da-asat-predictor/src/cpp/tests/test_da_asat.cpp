#include "da_asat/types.h"
#include "da_asat/propagator.h"
#include "da_asat/interceptor.h"
#include "da_asat/engagement.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using namespace da_asat;

// ─── Test helpers ───────────────────────────────────────────────────────────

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
            std::cout << "FAIL at line " << __LINE__ << ": " << #a << " != " << #b << std::endl; \
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

// ─── Test TLE data ─────────────────────────────────────────────────────────

static const std::string ISS_NAME  = "ISS (ZARYA)";
static const std::string ISS_LINE1 = "1 25544U 98067A   24015.50000000  .00016717  00000-0  30000-4 0  9991";
static const std::string ISS_LINE2 = "2 25544  51.6400 247.4627 0006703 130.5360 325.0288 15.49815384868000";

// Plesetsk Cosmodrome (Russia, ~63 N)
static LaunchSite plesetsk() {
    return {"plesetsk", "Plesetsk Cosmodrome", 62.93, 40.58, 0.13};
}

// Xichang Satellite Launch Center (China, ~28 N)
static LaunchSite xichang() {
    return {"xichang", "Xichang SLC", 28.25, 102.03, 1.8};
}

// ─── Test: parse_tle ───────────────────────────────────────────────────────

void test_parse_tle() {
    TEST(parse_tle);

    auto result = Propagator::parse_tle(ISS_NAME, ISS_LINE1, ISS_LINE2);
    ASSERT_TRUE(result.has_value());

    auto& tle = result.value();
    ASSERT_EQ(tle.norad_id, 25544);
    ASSERT_EQ(tle.name, "ISS (ZARYA)");
    ASSERT_NEAR(tle.inclination, 51.64, 0.01);
    ASSERT_NEAR(tle.raan, 247.4627, 0.001);
    ASSERT_NEAR(tle.eccentricity, 0.0006703, 1e-7);
    ASSERT_NEAR(tle.arg_perigee, 130.536, 0.01);
    ASSERT_NEAR(tle.mean_anomaly, 325.0288, 0.01);
    ASSERT_NEAR(tle.mean_motion, 15.49815384, 0.001);

    PASS();
}

// ─── Test: propagate_target ────────────────────────────────────────────────

void test_propagate_target() {
    TEST(propagate_target);

    auto tle = Propagator::parse_tle(ISS_NAME, ISS_LINE1, ISS_LINE2);
    ASSERT_TRUE(tle.has_value());

    // Propagate to epoch
    Timestamp epoch = static_cast<Timestamp>(
        (24.0 * 365.25 + 15.5 - 0.5) * 86400.0 + 946684800.0); // approx Jan 15 2024
    // Use a more direct approach: just use TLE epoch conversion
    double epoch_year = tle->epoch_year;
    double epoch_day = tle->epoch_day;

    // Propagate 0 seconds from epoch (at epoch)
    auto state = Propagator::propagate(*tle, epoch);
    ASSERT_TRUE(state.has_value());

    // ISS altitude should be ~400-430 km
    double r = std::sqrt(state->position.x * state->position.x +
                          state->position.y * state->position.y +
                          state->position.z * state->position.z);
    double alt = r - 6378.137;

    ASSERT_TRUE(alt > 350.0);
    ASSERT_TRUE(alt < 500.0);

    // Velocity should be ~7.5-7.8 km/s
    double v = std::sqrt(state->velocity.x * state->velocity.x +
                          state->velocity.y * state->velocity.y +
                          state->velocity.z * state->velocity.z);
    ASSERT_TRUE(v > 7.0);
    ASSERT_TRUE(v < 8.0);

    PASS();
}

// ─── Test: orbital_period ──────────────────────────────────────────────────

void test_orbital_period() {
    TEST(orbital_period);

    auto tle = Propagator::parse_tle(ISS_NAME, ISS_LINE1, ISS_LINE2);
    ASSERT_TRUE(tle.has_value());

    double period = Propagator::orbital_period_sec(*tle);

    // ISS orbital period ~92 minutes = ~5520 seconds
    ASSERT_NEAR(period, 5560.0, 100.0);

    PASS();
}

// ─── Test: default_interceptor ─────────────────────────────────────────────

void test_default_interceptor() {
    TEST(default_interceptor);

    auto params = Interceptor::default_interceptor();

    ASSERT_EQ(params.num_stages, 3);
    ASSERT_EQ(static_cast<int>(params.stage_thrust_kn.size()), 3);
    ASSERT_EQ(static_cast<int>(params.stage_burn_time_sec.size()), 3);
    ASSERT_EQ(static_cast<int>(params.stage_mass_kg.size()), 3);
    ASSERT_EQ(static_cast<int>(params.stage_propellant_kg.size()), 3);

    // Stage 1: 500 kN, 60s, 20000 kg total, 15000 kg propellant
    ASSERT_NEAR(params.stage_thrust_kn[0], 500.0, 0.1);
    ASSERT_NEAR(params.stage_burn_time_sec[0], 60.0, 0.1);
    ASSERT_NEAR(params.stage_mass_kg[0], 20000.0, 0.1);
    ASSERT_NEAR(params.stage_propellant_kg[0], 15000.0, 0.1);

    // Kill vehicle
    ASSERT_NEAR(params.kill_vehicle_mass_kg, 50.0, 0.1);
    ASSERT_NEAR(params.max_lateral_accel_g, 15.0, 0.1);

    // Propellant must be less than total mass for each stage
    for (int i = 0; i < params.num_stages; ++i) {
        ASSERT_TRUE(params.stage_propellant_kg[i] < params.stage_mass_kg[i]);
    }

    PASS();
}

// ─── Test: atmospheric_density ─────────────────────────────────────────────

void test_atmospheric_density() {
    TEST(atmospheric_density);

    // Test atmospheric model indirectly via trajectory simulation.
    // A launch at low altitude should experience drag (slower trajectory)
    // vs. a trajectory that starts above the atmosphere.
    // We verify by comparing delta-v requirements at different altitudes.

    Interceptor interceptor;
    auto params = Interceptor::default_interceptor();

    // Compute delta-v to a point at 400 km altitude
    // From sea level site vs. from high-altitude site
    LaunchSite sea_level = {"test_sl", "Sea Level", 28.0, -80.0, 0.0};
    LaunchSite high_alt  = {"test_hi", "High Alt",  28.0, -80.0, 100.0}; // 100 km

    Vec3 target_pos = {6778.0, 0.0, 0.0}; // ~400 km alt on x-axis

    double dv_sl = interceptor.compute_delta_v(sea_level, target_pos, 300.0);
    double dv_hi = interceptor.compute_delta_v(high_alt, target_pos, 300.0);

    // Delta-v from sea level should be higher (more gravity loss from lower altitude)
    ASSERT_TRUE(dv_sl > dv_hi);

    // Both should be in reasonable range (2-10 km/s)
    ASSERT_TRUE(dv_sl > 1.0);
    ASSERT_TRUE(dv_sl < 15.0);
    ASSERT_TRUE(dv_hi > 0.5);
    ASSERT_TRUE(dv_hi < 15.0);

    // The default interceptor default_interceptor() exercises the atmospheric
    // model during simulate_trajectory — verify it produces points
    auto site = sea_level;
    OrbitalState dummy_state;
    dummy_state.position = {6778.0, 0.0, 0.0};
    dummy_state.velocity = {0.0, 7.67, 0.0};
    dummy_state.epoch = 1705312800;

    std::vector<OrbitalState> target_states(601, dummy_state);
    for (size_t i = 0; i < target_states.size(); ++i) {
        target_states[i].epoch = dummy_state.epoch + static_cast<Timestamp>(i);
    }

    auto traj = interceptor.simulate_trajectory(site, 45.0, 80.0, params,
                                                  target_states, 1.0, 150.0);
    // Should produce trajectory points
    ASSERT_TRUE(traj.size() > 10);

    // First point should be near sea level
    ASSERT_NEAR(traj[0].altitude_km, 0.0, 10.0);

    // Some later point should be at higher altitude (rocket goes up)
    bool went_up = false;
    for (const auto& pt : traj) {
        if (pt.altitude_km > 50.0) { went_up = true; break; }
    }
    ASSERT_TRUE(went_up);

    PASS();
}

// ─── Test: compute_delta_v ─────────────────────────────────────────────────

void test_compute_delta_v() {
    TEST(compute_delta_v);

    Interceptor interceptor;
    auto site = plesetsk();

    // Target at ~400 km altitude, 300 seconds TOF
    // Position at 400 km above equator (ECI)
    Vec3 intercept_pos = {6778.0, 0.0, 0.0}; // R_earth + 400 km on x-axis

    double dv = interceptor.compute_delta_v(site, intercept_pos, 300.0);

    // Delta-V to reach 400 km in 300s should be roughly 3-7 km/s
    ASSERT_TRUE(dv > 2.0);
    ASSERT_TRUE(dv < 10.0);

    PASS();
}

// ─── Test: check_feasibility ───────────────────────────────────────────────

void test_check_feasibility() {
    TEST(check_feasibility);

    auto tle = Propagator::parse_tle(ISS_NAME, ISS_LINE1, ISS_LINE2);
    ASSERT_TRUE(tle.has_value());

    auto params = Interceptor::default_interceptor();
    EngagementCalculator calc;

    // Propagate ISS to some time
    Timestamp t = 1705312800; // ~Jan 15, 2024 10:00 UTC
    auto state = Propagator::propagate(*tle, t);
    ASSERT_TRUE(state.has_value());

    // The ISS at ~420 km should be within the default interceptor's capability
    // (it may or may not be geometrically visible from Plesetsk at this specific time)
    double r = std::sqrt(state->position.x * state->position.x +
                          state->position.y * state->position.y +
                          state->position.z * state->position.z);
    double alt = r - 6378.137;
    ASSERT_TRUE(alt > 350.0);
    ASSERT_TRUE(alt < 500.0);

    // The default interceptor should be capable of reaching ISS altitude
    double max_alt = calc.max_engagement_altitude(params);
    ASSERT_TRUE(max_alt > alt);

    PASS();
}

// ─── Test: evaluate_single_engagement ──────────────────────────────────────

void test_evaluate_single_engagement() {
    TEST(evaluate_single_engagement);

    auto tle = Propagator::parse_tle(ISS_NAME, ISS_LINE1, ISS_LINE2);
    ASSERT_TRUE(tle.has_value());

    auto params = Interceptor::default_interceptor();
    auto site = plesetsk();
    EngagementCalculator calc;

    // Try multiple launch times over an orbit period to find a feasible one
    Timestamp base_time = 1705312800; // ~Jan 15, 2024 10:00 UTC
    bool found_feasible = false;

    for (int i = 0; i < 100; ++i) {
        Timestamp t = base_time + i * 60; // Step by 1 minute
        auto result = calc.evaluate_single(site, *tle, params, t);

        if (result.feasibility == Feasibility::FEASIBLE) {
            found_feasible = true;

            // Verify reasonable launch parameters
            ASSERT_TRUE(result.launch_azimuth_deg >= 0.0);
            ASSERT_TRUE(result.launch_azimuth_deg <= 360.0);
            ASSERT_TRUE(result.total_delta_v_kms > 1.0);
            ASSERT_TRUE(result.total_delta_v_kms < 15.0);
            ASSERT_TRUE(result.time_of_flight_sec > 60.0);
            ASSERT_TRUE(result.time_of_flight_sec < 600.0);
            ASSERT_TRUE(result.intercept.altitude_km > 100.0);
            break;
        }
    }

    // ISS from Plesetsk (62.9N) should have feasible windows
    // since ISS inclination (51.6) means it passes reasonably close
    ASSERT_TRUE(found_feasible);

    PASS();
}

// ─── Test: find_engagement_windows ─────────────────────────────────────────

void test_find_engagement_windows() {
    TEST(find_engagement_windows);

    auto tle = Propagator::parse_tle(ISS_NAME, ISS_LINE1, ISS_LINE2);
    ASSERT_TRUE(tle.has_value());

    auto params = Interceptor::default_interceptor();
    auto site = xichang(); // ~28N, closer to ISS inclination
    EngagementCalculator calc;

    // Search over 6 hours (should find at least one pass)
    Timestamp window_start = 1705312800;
    Timestamp window_end   = window_start + 6 * 3600;

    auto windows = calc.find_engagement_windows(site, *tle, params,
                                                  window_start, window_end);

    // Over 6 hours, ISS should have multiple passes visible from Xichang
    // (ISS period ~92 min, so ~4 orbits in 6 hours)
    // Not all will be geometrically feasible, but at least some should be
    ASSERT_TRUE(windows.size() >= 1);

    // Each window should have valid parameters
    for (const auto& w : windows) {
        ASSERT_TRUE(w.window_end >= w.window_start);
        ASSERT_TRUE(w.min_delta_v_kms > 0.0);
        ASSERT_TRUE(w.best_engagement.feasibility == Feasibility::FEASIBLE);
    }

    PASS();
}

// ─── Test: engagement_zone_boundary ────────────────────────────────────────

void test_engagement_zone_boundary() {
    TEST(engagement_zone_boundary);

    auto params = Interceptor::default_interceptor();
    auto site = plesetsk();
    EngagementCalculator calc;

    auto boundary = calc.compute_engagement_zone_boundary(site, params, 400.0);

    // Should produce a polygon (72 points at 5-degree steps)
    ASSERT_EQ(static_cast<int>(boundary.size()), 72);

    // All points should be within reasonable distance from launch site
    for (const auto& pt : boundary) {
        ASSERT_TRUE(pt.lat >= -90.0 && pt.lat <= 90.0);
        ASSERT_TRUE(pt.lon >= -180.0 && pt.lon <= 360.0);

        // Distance from site should be > 0 and < ~3000 km
        double dlat = (pt.lat - site.lat) * 3.14159265 / 180.0;
        double dlon = (pt.lon - site.lon) * 3.14159265 / 180.0;
        double a = std::sin(dlat/2) * std::sin(dlat/2) +
                   std::cos(site.lat * 3.14159265/180.0) *
                   std::cos(pt.lat * 3.14159265/180.0) *
                   std::sin(dlon/2) * std::sin(dlon/2);
        double c = 2 * std::asin(std::sqrt(a));
        double dist_km = 6378.137 * c;

        ASSERT_TRUE(dist_km > 0.0);
        ASSERT_TRUE(dist_km < 5000.0);
    }

    PASS();
}

// ─── Test: max_engagement_altitude ─────────────────────────────────────────

void test_max_engagement_altitude() {
    TEST(max_engagement_altitude);

    auto params = Interceptor::default_interceptor();
    EngagementCalculator calc;

    double max_alt = calc.max_engagement_altitude(params);

    // Default 3-stage interceptor should reach ~1000-2000 km
    ASSERT_TRUE(max_alt > 800.0);
    ASSERT_TRUE(max_alt < 5000.0);

    PASS();
}

// ─── Test: infeasible_engagement ───────────────────────────────────────────

void test_infeasible_engagement() {
    TEST(infeasible_engagement);

    // Create a high-altitude TLE (MEO-like, ~20000 km)
    // Modify ISS TLE to have much lower mean motion → higher orbit
    std::string high_line1 = "1 99999U 24001A   24015.50000000  .00000000  00000-0  00000-0 0  9991";
    std::string high_line2 = "2 99999  55.0000 100.0000 0010000  90.0000 270.0000  2.00568000    01";

    auto tle = Propagator::parse_tle("HIGH-SAT", high_line1, high_line2);
    ASSERT_TRUE(tle.has_value());

    // Verify this is a high orbit
    double period = Propagator::orbital_period_sec(*tle);
    ASSERT_TRUE(period > 40000.0); // > 11 hours → MEO

    auto params = Interceptor::default_interceptor();
    auto site = plesetsk();
    EngagementCalculator calc;

    // Evaluate engagement — should be infeasible (too high)
    Timestamp t = 1705312800;
    auto result = calc.evaluate_single(site, *tle, params, t);

    // Should not be FEASIBLE — either ALTITUDE_TOO_HIGH or INSUFFICIENT_ENERGY
    ASSERT_TRUE(result.feasibility != Feasibility::FEASIBLE);

    PASS();
}

// ─── Test: proportional_navigation ─────────────────────────────────────────

void test_proportional_navigation() {
    TEST(proportional_navigation);

    Interceptor interceptor;

    // LOS pointing along x-axis
    Vec3 los = {100.0, 0.0, 0.0};
    // LOS rate: target moving in y direction
    Vec3 los_rate = {0.0, 1.0, 0.0};
    double closing_speed = 10.0;

    Vec3 accel = interceptor.compute_pn_acceleration(los, los_rate, closing_speed, 4.0);

    // PN should produce acceleration perpendicular to LOS
    // a = N * Vc * omega_LOS
    // omega_LOS = (LOS x LOS_rate) / |LOS|^2
    // (100,0,0) x (0,1,0) = (0,0,100)
    // omega = (0,0,100) / 10000 = (0,0,0.01)
    // a = 4 * 10 * (0,0,0.01) = (0,0,0.4)
    ASSERT_NEAR(accel.x, 0.0, 0.01);
    ASSERT_NEAR(accel.y, 0.0, 0.01);
    ASSERT_NEAR(accel.z, 0.4, 0.01);

    PASS();
}

// ─── Test: state_to_geodetic ───────────────────────────────────────────────

void test_state_to_geodetic() {
    TEST(state_to_geodetic);

    auto tle = Propagator::parse_tle(ISS_NAME, ISS_LINE1, ISS_LINE2);
    ASSERT_TRUE(tle.has_value());

    Timestamp t = 1705312800;
    auto state = Propagator::propagate(*tle, t);
    ASSERT_TRUE(state.has_value());

    auto geo = Propagator::state_to_geodetic(*state);

    // ISS altitude ~400-430 km
    ASSERT_TRUE(geo.alt_km > 350.0);
    ASSERT_TRUE(geo.alt_km < 500.0);

    // Latitude should be within ISS inclination band (-51.64 to +51.64)
    ASSERT_TRUE(geo.lat >= -52.0 && geo.lat <= 52.0);

    // Longitude should be valid
    ASSERT_TRUE(geo.lon >= -180.0 && geo.lon <= 180.0);

    PASS();
}

// ─── Main ──────────────────────────────────────────────────────────────────

int main() {
    std::cout << "DA-ASAT Engagement Zone Predictor — Test Suite" << std::endl;
    std::cout << "================================================" << std::endl;

    test_parse_tle();
    test_propagate_target();
    test_orbital_period();
    test_default_interceptor();
    test_atmospheric_density();
    test_compute_delta_v();
    test_check_feasibility();
    test_evaluate_single_engagement();
    test_find_engagement_windows();
    test_engagement_zone_boundary();
    test_max_engagement_altitude();
    test_infeasible_engagement();
    test_proportional_navigation();
    test_state_to_geodetic();

    std::cout << "================================================" << std::endl;
    std::cout << "Results: " << tests_passed << "/" << tests_run << " passed" << std::endl;

    return (tests_passed == tests_run) ? 0 : 1;
}
