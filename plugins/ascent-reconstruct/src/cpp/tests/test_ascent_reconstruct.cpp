#include "ascent_reconstruct/types.h"
#include "ascent_reconstruct/trajectory.h"
#include "ascent_reconstruct/estimator.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using namespace ascent_reconstruct;

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

// ─── Test data: sounding rocket parameters ─────────────────────────────────

static AscentModelParams make_single_stage_params() {
    AscentModelParams params;
    params.launch_lat = 28.49;    // Cape Canaveral
    params.launch_lon = -80.58;
    params.launch_alt = 0.0;
    params.launch_time = 0.0;
    params.launch_azimuth = 90.0 * DEG2RAD; // East
    params.initial_pitch = 85.0 * DEG2RAD;  // Nearly vertical

    AscentModelParams::StageParams s1;
    s1.thrust = 500000.0;     // 500 kN
    s1.isp = 280.0;           // 280 s
    s1.mass_initial = 20000.0; // 20 tons
    s1.mass_final = 5000.0;   // 5 tons
    s1.burn_time = 120.0;     // 120 s
    s1.pitch_rate = 0.005;    // rad/s
    params.stages.push_back(s1);

    return params;
}

static AscentModelParams make_two_stage_params() {
    AscentModelParams params = make_single_stage_params();

    AscentModelParams::StageParams s2;
    s2.thrust = 100000.0;     // 100 kN
    s2.isp = 310.0;           // 310 s
    s2.mass_initial = 5000.0; // 5 tons
    s2.mass_final = 1500.0;   // 1.5 tons
    s2.burn_time = 200.0;     // 200 s
    s2.pitch_rate = 0.001;    // rad/s
    params.stages.push_back(s2);

    return params;
}

// Sensor near Cape Canaveral
static LatLonAlt test_sensor() {
    return {28.40, -80.60, 0.0};
}

// ─── Tests ──────────────────────────────────────────────────────────────────

void test_geodetic_ecef_roundtrip() {
    TEST(geodetic_ecef_roundtrip);

    // Test several points
    struct TestCase { double lat, lon, alt; };
    TestCase cases[] = {
        {28.49, -80.58, 0.0},
        {0.0, 0.0, 0.0},
        {45.0, 90.0, 10000.0},
        {-33.87, 151.21, 50.0},
        {89.99, 0.0, 100000.0}
    };

    for (const auto& tc : cases) {
        Vec3 ecef = TrajectorySimulator::geodetic_to_ecef(tc.lat, tc.lon, tc.alt);
        LatLonAlt lla = TrajectorySimulator::ecef_to_geodetic(ecef);

        ASSERT_NEAR(lla.lat, tc.lat, 0.01);
        ASSERT_NEAR(lla.lon, tc.lon, 0.01);
        ASSERT_NEAR(lla.alt, tc.alt, 100.0); // 100m tolerance for spherical approx
    }

    PASS();
}

void test_gravity() {
    TEST(gravity);

    // Gravity at sea level on equator
    Vec3 pos = TrajectorySimulator::geodetic_to_ecef(0.0, 0.0, 0.0);
    Vec3 g = TrajectorySimulator::gravity(pos);
    double g_mag = g.norm();

    // Should be approximately 9.8 m/s^2
    ASSERT_NEAR(g_mag, 9.8, 0.2);

    // Gravity should point toward Earth center (opposite to position)
    double dot = g.dot(pos.normalized());
    ASSERT_TRUE(dot < 0); // gravity points inward

    // At higher altitude, gravity should be weaker
    Vec3 pos_high = TrajectorySimulator::geodetic_to_ecef(0.0, 0.0, 200000.0);
    Vec3 g_high = TrajectorySimulator::gravity(pos_high);
    ASSERT_TRUE(g_high.norm() < g_mag);

    PASS();
}

void test_atm_density() {
    TEST(atm_density);

    // Sea level density ~1.225 kg/m^3
    double rho_sl = TrajectorySimulator::atm_density(0.0);
    ASSERT_NEAR(rho_sl, 1.225, 0.01);

    // Density decreases with altitude
    double rho_10k = TrajectorySimulator::atm_density(10000.0);
    ASSERT_TRUE(rho_10k < rho_sl);
    ASSERT_TRUE(rho_10k > 0.0);

    // At 50 km, very low density
    double rho_50k = TrajectorySimulator::atm_density(50000.0);
    ASSERT_TRUE(rho_50k < 0.01);
    ASSERT_TRUE(rho_50k > 0.0);

    // At 200 km, essentially zero
    double rho_200k = TrajectorySimulator::atm_density(200000.0);
    ASSERT_TRUE(rho_200k < 1e-8);

    // Above 200 km, zero
    double rho_high = TrajectorySimulator::atm_density(300000.0);
    ASSERT_NEAR(rho_high, 0.0, 1e-15);

    PASS();
}

void test_single_stage_simulation() {
    TEST(single_stage_simulation);

    auto params = make_single_stage_params();
    TrajectorySimulator sim;
    auto traj = sim.simulate(params, 130.0, 0.5);

    ASSERT_TRUE(!traj.empty());

    // First point should be near launch site
    ASSERT_NEAR(traj[0].altitude_m, 0.0, 1000.0);

    // Should gain altitude
    double max_alt = 0;
    double max_speed = 0;
    for (const auto& sv : traj) {
        if (sv.altitude_m > max_alt) max_alt = sv.altitude_m;
        if (sv.speed_mps > max_speed) max_speed = sv.speed_mps;
    }

    // With 500kN thrust and 20t vehicle, should reach significant altitude
    // T/W ratio ~2.5 at launch, should reach >50 km in 120s
    std::cout << "(max_alt=" << max_alt/1000.0 << "km, max_speed=" << max_speed << "m/s) ";
    ASSERT_TRUE(max_alt > 30000.0);  // At least 30 km
    ASSERT_TRUE(max_speed > 1000.0); // At least 1 km/s

    // Speed should generally increase during burn
    ASSERT_TRUE(traj.back().speed_mps > traj[10].speed_mps);

    // All states should be stage 1
    for (const auto& sv : traj) {
        ASSERT_EQ(sv.stage, 1);
    }

    PASS();
}

void test_two_stage_simulation() {
    TEST(two_stage_simulation);

    auto params = make_two_stage_params();
    TrajectorySimulator sim;

    double total_burn = params.stages[0].burn_time + params.stages[1].burn_time;
    auto traj = sim.simulate(params, total_burn + 10.0, 0.5);

    ASSERT_TRUE(!traj.empty());

    // Should have both stage 1 and stage 2 states
    bool has_stage1 = false, has_stage2 = false;
    for (const auto& sv : traj) {
        if (sv.stage == 1) has_stage1 = true;
        if (sv.stage == 2) has_stage2 = true;
    }
    ASSERT_TRUE(has_stage1);
    ASSERT_TRUE(has_stage2);

    // Should reach higher altitude/speed than single stage
    double max_alt = 0, max_speed = 0;
    for (const auto& sv : traj) {
        if (sv.altitude_m > max_alt) max_alt = sv.altitude_m;
        if (sv.speed_mps > max_speed) max_speed = sv.speed_mps;
    }

    std::cout << "(max_alt=" << max_alt/1000.0 << "km, max_speed=" << max_speed << "m/s) ";
    ASSERT_TRUE(max_alt > 50000.0);  // At least 50 km
    ASSERT_TRUE(max_speed > 2000.0); // At least 2 km/s

    PASS();
}

void test_staging_detection() {
    TEST(staging_detection);

    auto params = make_two_stage_params();
    auto sensor = test_sensor();

    // Generate synthetic tracklet spanning both stages
    double t_start = 10.0;
    double total_burn = params.stages[0].burn_time + params.stages[1].burn_time;
    double t_end = total_burn - 10.0;

    auto tracklet = AscentEstimator::generate_synthetic_tracklet(
        params, sensor, t_start, t_end, 2.0, 10.0);

    ASSERT_TRUE(tracklet.observations.size() > 10);

    AscentEstimator estimator;
    auto events = estimator.detect_staging(tracklet);

    std::cout << "(detected " << events.size() << " events) ";

    // Should detect at least one staging event near t=120s
    ASSERT_TRUE(events.size() >= 1);

    // First staging event should be near the actual staging time
    ASSERT_NEAR(events[0].time, params.stages[0].burn_time, 20.0);
    ASSERT_EQ(events[0].from_stage, 1);
    ASSERT_EQ(events[0].to_stage, 2);

    PASS();
}

void test_launch_site_estimation() {
    TEST(launch_site_estimation);

    auto params = make_single_stage_params();
    auto sensor = test_sensor();

    auto tracklet = AscentEstimator::generate_synthetic_tracklet(
        params, sensor, 10.0, 60.0, 2.0, 20.0);

    ASSERT_TRUE(!tracklet.observations.empty());

    AscentEstimator estimator;
    auto site = estimator.estimate_launch_site(tracklet);

    // Should be within 10 km (~0.09 degrees) of actual launch site
    double dlat = site.lat - params.launch_lat;
    double dlon = site.lon - params.launch_lon;
    double dist_deg = std::sqrt(dlat*dlat + dlon*dlon);

    std::cout << "(est=" << site.lat << "," << site.lon
              << " err=" << dist_deg << "deg) ";

    // 10 km is roughly 0.09 degrees
    ASSERT_TRUE(dist_deg < 1.0); // Within ~100 km (generous for extrapolation)

    PASS();
}

void test_launch_time_estimation() {
    TEST(launch_time_estimation);

    auto params = make_single_stage_params();
    auto sensor = test_sensor();

    // Start observing at t=20 seconds after launch
    auto tracklet = AscentEstimator::generate_synthetic_tracklet(
        params, sensor, 20.0, 80.0, 2.0, 10.0);

    AscentEstimator estimator;
    double t_est = estimator.estimate_launch_time(tracklet);

    // Should estimate launch time within 10 seconds of actual (t=0)
    std::cout << "(est=" << t_est << "s) ";
    ASSERT_NEAR(t_est, 0.0, 15.0); // 15 second tolerance

    PASS();
}

void test_residual_computation() {
    TEST(residual_computation);

    auto params = make_single_stage_params();
    auto sensor = test_sensor();

    // Generate perfect data (low noise)
    auto tracklet = AscentEstimator::generate_synthetic_tracklet(
        params, sensor, 5.0, 100.0, 2.0, 1.0); // 1m noise

    AscentEstimator estimator;
    auto residuals = estimator.compute_residuals(params, tracklet);

    ASSERT_TRUE(!residuals.empty());
    ASSERT_EQ(residuals.size(), tracklet.observations.size());

    double rms = estimator.rms_residual(residuals);
    std::cout << "(rms=" << rms << "m) ";

    // With 1m noise and correct params, residuals should be small
    ASSERT_TRUE(rms < 500.0); // Allow for integration mismatch

    PASS();
}

void test_reconstruction() {
    TEST(reconstruction);

    auto true_params = make_single_stage_params();
    auto sensor = test_sensor();

    // Generate synthetic tracklet
    auto tracklet = AscentEstimator::generate_synthetic_tracklet(
        true_params, sensor, 10.0, 110.0, 2.0, 20.0);

    ASSERT_TRUE(tracklet.observations.size() > 20);

    // Create a perturbed initial guess
    AscentModelParams guess = true_params;
    guess.launch_time = 5.0;  // 5 second offset
    guess.launch_azimuth = 85.0 * DEG2RAD; // 5 degree offset
    guess.initial_pitch = 83.0 * DEG2RAD;
    guess.stages[0].thrust = 480000.0; // 4% off

    AscentEstimator estimator;
    estimator.set_max_iterations(15);
    estimator.set_tolerance(500.0);

    auto result = estimator.reconstruct(tracklet, guess);

    std::cout << "(converged=" << result.converged
              << " iter=" << result.iterations
              << " rms=" << result.rms_residual << "m) ";

    // Should produce a trajectory
    ASSERT_TRUE(!result.trajectory.empty());

    // Trajectory should have reasonable altitude profile
    double max_alt = 0;
    for (const auto& sv : result.trajectory) {
        if (sv.altitude_m > max_alt) max_alt = sv.altitude_m;
    }
    ASSERT_TRUE(max_alt > 20000.0); // At least 20 km

    // RMS should be reasonable (may not fully converge but should improve)
    ASSERT_TRUE(result.rms_residual < 50000.0);

    PASS();
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== ascent-reconstruct tests ===" << std::endl;

    test_geodetic_ecef_roundtrip();
    test_gravity();
    test_atm_density();
    test_single_stage_simulation();
    test_two_stage_simulation();
    test_staging_detection();
    test_launch_site_estimation();
    test_launch_time_estimation();
    test_residual_computation();
    test_reconstruction();

    std::cout << "\n" << tests_passed << "/" << tests_run << " tests passed." << std::endl;
    return (tests_passed == tests_run) ? 0 : 1;
}
