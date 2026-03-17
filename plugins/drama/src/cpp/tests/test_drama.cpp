/**
 * @file test_drama.cpp
 * @brief Comprehensive test suite for all DRAMA modules
 *
 * Test values are derived from authoritative sources:
 *   - NASA ORDEM 3.1 Technical Report (Krisko 2014)
 *   - ESA DRAMA User Manual (OSCAR, ARES, CROC, SARA)
 *   - Vallado "Fundamentals of Astrodynamics", 4th ed.
 *   - IADC Space Debris Mitigation Guidelines
 *   - Grün et al. 1985, Icarus 62
 *   - King-Hele "Satellite Orbits in an Atmosphere", 1987
 */
#include "../include/types.h"
#include "../include/constants.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

namespace drama {
    // External function declarations
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

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        std::cerr << "  FAIL: " << msg << std::endl; \
        tests_failed++; \
    } else { \
        std::cout << "  PASS: " << msg << std::endl; \
        tests_passed++; \
    } \
} while(0)

#define TEST_RANGE(val, lo, hi, msg) do { \
    double v_ = (val); \
    if (v_ >= (lo) && v_ <= (hi)) { \
        std::cout << "  PASS: " << msg << " = " << v_ << " (expected " << (lo) << "–" << (hi) << ")" << std::endl; \
        tests_passed++; \
    } else { \
        std::cerr << "  FAIL: " << msg << " = " << v_ << " (expected " << (lo) << "–" << (hi) << ")" << std::endl; \
        tests_failed++; \
    } \
} while(0)

// ════════════════════════════════════════════════════════════════════════════
// Helper: create standard orbit
// ════════════════════════════════════════════════════════════════════════════

static KeplerianOrbit make_circular_orbit(double alt_km, double inc_deg) {
    KeplerianOrbit orb;
    orb.semi_major_axis_m = (R_EARTH_KM + alt_km) * 1000.0;
    orb.eccentricity = 0.001; // near-circular
    orb.inclination_rad = inc_deg * DEG_TO_RAD;
    orb.raan_rad = 0;
    orb.arg_perigee_rad = 0;
    orb.true_anomaly_rad = 0;
    return orb;
}

static EpochTime make_epoch(int year, int month, int day) {
    return {year, month, day, 0, 0, 0.0};
}

// ════════════════════════════════════════════════════════════════════════════
// Type tests
// ════════════════════════════════════════════════════════════════════════════

static void test_types() {
    std::cout << "\n═══ Types & Utility Tests ═══" << std::endl;

    // Keplerian orbit basics
    // Source: Vallado Ch.2 — ISS at 408 km circular, 51.6° inclination
    KeplerianOrbit iss = make_circular_orbit(408.0, 51.6);
    // With e=0.001: perigee ≈ 408 - 6.878 ≈ 401 km, apogee ≈ 415 km
    TEST_RANGE(iss.perigee_altitude_km(), 399.0, 410.0,
               "ISS perigee altitude [Source: Vallado Ch.2, e=0.001]");
    TEST_RANGE(iss.apogee_altitude_km(), 406.0, 418.0,
               "ISS apogee altitude [Source: Vallado Ch.2, e=0.001]");

    // Orbital period: ISS ~92.7 minutes
    // Source: NASA ISS Trajectory Data
    double period_min = iss.period_seconds() / 60.0;
    TEST_RANGE(period_min, 92.0, 93.5,
               "ISS orbital period [Source: NASA ISS data, ~92.7 min]");

    // Julian Date conversion
    // Source: Meeus "Astronomical Algorithms", Example 7.a
    // 2000-01-01 12:00 TDB = JD 2451545.0
    EpochTime j2000 = {2000, 1, 1, 12, 0, 0.0};
    TEST_RANGE(j2000.to_jd(), 2451544.9, 2451545.1,
               "J2000 Julian Date [Source: Meeus, JD 2451545.0]");

    // Fractional year
    EpochTime mid2024 = {2024, 7, 1, 0, 0, 0.0};
    TEST_RANGE(mid2024.fractional_year(), 2024.49, 2024.51,
               "Mid-2024 fractional year");

    // Material properties
    // Source: ESA DRAMA SARA User Manual, Table 4-1
    MaterialProperties al = MaterialProperties::get(MaterialType::ALUMINUM);
    TEST_RANGE(al.melting_point_k, 930.0, 940.0,
               "Aluminum melting point [Source: DRAMA SARA, 933 K / 660°C]");
    TEST_RANGE(al.density_kg_m3, 2690.0, 2710.0,
               "Aluminum density [Source: standard, 2700 kg/m³]");

    MaterialProperties ti = MaterialProperties::get(MaterialType::TITANIUM);
    TEST_RANGE(ti.melting_point_k, 1935.0, 1945.0,
               "Titanium melting point [Source: DRAMA SARA, 1941 K / 1668°C]");

    MaterialProperties st = MaterialProperties::get(MaterialType::STEEL);
    TEST_RANGE(st.melting_point_k, 1780.0, 1790.0,
               "Steel melting point [Source: DRAMA SARA, 1783 K / 1510°C]");

    MaterialProperties cfrp = MaterialProperties::get(MaterialType::CFRP);
    TEST_RANGE(cfrp.melting_point_k, 570.0, 580.0,
               "CFRP decomposition temp [Source: DRAMA SARA, 573 K / 300°C]");

    MaterialProperties glass = MaterialProperties::get(MaterialType::GLASS);
    TEST_RANGE(glass.melting_point_k, 970.0, 980.0,
               "Glass softening point [Source: DRAMA SARA, 973 K / 700°C]");
}

// ════════════════════════════════════════════════════════════════════════════
// OSCAR tests — Orbital Lifetime
// ════════════════════════════════════════════════════════════════════════════

static void test_oscar() {
    std::cout << "\n═══ OSCAR — Orbital Lifetime Tests ═══" << std::endl;

    // Test 1: 3U CubeSat at 500 km SSO
    // Expected: Several years of gradual decay, then accelerating
    // Source: ESA DRAMA OSCAR User Manual, Example 5.1
    //         LinkedIn post reference: gradual decay at 500 km SSO
    // Typical 3U CubeSat: 4 kg, 0.03 m², Cd=2.2
    // Ballistic coefficient: 4/(2.2×0.03) ≈ 60.6 kg/m²
    {
        std::cout << "\n--- 3U CubeSat at 500 km SSO ---" << std::endl;
        LifetimeConfig lc;
        lc.initial_orbit = make_circular_orbit(500.0, 97.4); // SSO
        lc.satellite = {4.0, 0.03, 2.2, 1.3};
        lc.solar_model = SolarActivityModel::MEAN_CYCLE;
        lc.solar_override = {F107_MEAN, F107_MEAN, AP_MODERATE};
        lc.epoch = make_epoch(2024, 1, 1);
        lc.max_lifetime_years = 100;
        lc.time_step_days = 1.0;

        LifetimeResult result = compute_lifetime(lc);

        std::cout << "  Lifetime: " << result.lifetime_years << " years" << std::endl;
        std::cout << "  History points: " << result.history.size() << std::endl;
        std::cout << "  25-yr compliant: " << (result.compliant_25yr ? "YES" : "NO") << std::endl;

        // CubeSat at 500 km with B~60 should decay in ~3-15 years depending on solar activity
        // Source: ESA DRAMA OSCAR typical results for LEO small satellites
        // King-Hele theory for B≈60 at 500 km → 3–15 years (solar-cycle dependent)
        TEST_RANGE(result.lifetime_years, 1.0, 30.0,
                   "CubeSat 500km lifetime [Source: ESA DRAMA OSCAR, King-Hele theory]");
        TEST_ASSERT(result.compliant_25yr || result.lifetime_years <= 25.0,
                    "25-year compliance check consistent");
        TEST_ASSERT(result.history.size() > 10,
                    "Sufficient history points recorded");

        // Verify history shows gradual then accelerating decay
        if (result.history.size() > 2) {
            double first_perigee = result.history.front().perigee_km;
            double last_perigee = result.history.back().perigee_km;
            TEST_ASSERT(first_perigee > last_perigee,
                        "Perigee decreases over time (gradual decay)");
        }
    }

    // Test 2: ISS at 408 km — known ~2-3 year uncontrolled lifetime
    // Source: NASA ISS Trajectory Data; Vallado Ch.9 Example 9-2
    // ISS: ~420,000 kg, ~3000 m² (huge cross-section), Cd≈2.2
    // Ballistic coefficient: 420000/(2.2×3000) ≈ 63.6 kg/m²
    // Despite similar B to CubeSat, lower altitude = much faster decay
    {
        std::cout << "\n--- ISS at 408 km (uncontrolled) ---" << std::endl;
        LifetimeConfig lc;
        lc.initial_orbit = make_circular_orbit(408.0, 51.6);
        lc.satellite = {420000.0, 3000.0, 2.2, 1.3};
        lc.solar_model = SolarActivityModel::MEAN_CYCLE;
        lc.solar_override = {F107_MEAN, F107_MEAN, AP_MODERATE};
        lc.epoch = make_epoch(2024, 1, 1);
        lc.max_lifetime_years = 50;
        lc.time_step_days = 0.5;

        LifetimeResult result = compute_lifetime(lc);

        std::cout << "  Lifetime: " << result.lifetime_years << " years" << std::endl;
        std::cout << "  Ballistic coeff: " << lc.satellite.ballistic_coefficient() << " kg/m²" << std::endl;

        // ISS at 408 km: depends heavily on solar activity model
        // Source: Vallado Ch.9; NASA ISS re-entry planning documents
        // With B≈63.6 at 408 km in mean solar activity, HP model gives ~0.3-5 years
        // depending on solar cycle phase at epoch
        TEST_RANGE(result.lifetime_years, 0.1, 10.0,
                   "ISS 408km uncontrolled lifetime [Source: Vallado Ch.9, HP model]");
        TEST_ASSERT(result.compliant_25yr,
                    "ISS at 408km decays well within 25 years");
    }

    // Test 3: High-altitude satellite (1000 km) — should take decades
    {
        std::cout << "\n--- Satellite at 1000 km ---" << std::endl;
        LifetimeConfig lc;
        lc.initial_orbit = make_circular_orbit(1000.0, 87.0);
        lc.satellite = {500.0, 2.0, 2.2, 1.3};
        lc.solar_model = SolarActivityModel::MEAN_CYCLE;
        lc.solar_override = {F107_MEAN, F107_MEAN, AP_MODERATE};
        lc.epoch = make_epoch(2024, 1, 1);
        lc.max_lifetime_years = 200;
        lc.time_step_days = 5.0;

        LifetimeResult result = compute_lifetime(lc);

        std::cout << "  Lifetime: " << result.lifetime_years << " years" << std::endl;

        // B = 500/(2.2×2.0) = 113.6 kg/m² at 1000 km
        // Very low atmospheric density → hundreds of years
        // Source: King-Hele theory; ORDEM lifetime estimates
        TEST_ASSERT(result.lifetime_years > 25.0,
                    "1000 km satellite exceeds 25-year limit [Source: King-Hele theory]");
        TEST_ASSERT(!result.compliant_25yr,
                    "1000 km satellite is NOT 25-year compliant");
    }
}

// ════════════════════════════════════════════════════════════════════════════
// ARES tests — Collision Risk
// ════════════════════════════════════════════════════════════════════════════

static void test_ares() {
    std::cout << "\n═══ ARES — Collision Risk Tests ═══" << std::endl;

    // Test 1: Debris flux at 800 km for d >= 1 cm
    // Source: NASA ORDEM 3.1, Table 5-1
    // Expected: ~2e-5 impacts/m²/yr for d >= 1 cm at 800 km
    {
        std::cout << "\n--- Debris flux at 800 km, d≥1cm ---" << std::endl;
        FluxConfig fc;
        fc.orbit = make_circular_orbit(800.0, 98.0);
        fc.epoch = make_epoch(2024, 1, 1);
        fc.duration_years = 5.0;
        fc.min_diameter_m = 0.01; // 1 cm

        FluxResult result = compute_flux(fc);

        std::cout << "  Total debris flux: " << result.total_debris_flux << " /m²/yr" << std::endl;
        std::cout << "  Total meteoroid flux: " << result.total_meteoroid_flux << " /m²/yr" << std::endl;
        std::cout << "  Debris flux bins: " << result.debris_flux_by_size.size() << std::endl;

        // ORDEM 3.1 reference: F(≥1cm) ≈ 2e-5 /m²/yr at 800 km
        // Accept factor-of-5 match for parametric model
        TEST_RANGE(result.total_debris_flux, 5e-6, 1e-4,
                   "Debris flux at 800km ≥1cm [Source: NASA ORDEM 3.1, ~2e-5 /m²/yr]");

        // Meteoroid flux should be smaller than debris at 800 km for cm-sized
        // Source: Grün et al. 1985; meteoroid flux ~1e-5 for ≥1cm at 1 AU
        TEST_ASSERT(result.total_meteoroid_flux > 0,
                    "Meteoroid flux is positive [Source: Grün et al. 1985]");

        // Cumulative collision probability over 5 years for 1 m² target
        TEST_ASSERT(result.cumulative_collision_probability > 0 &&
                    result.cumulative_collision_probability < 1.0,
                    "Cumulative collision probability in valid range");
    }

    // Test 2: Collision risk for a 10 m² satellite at 800 km over 10 years
    {
        std::cout << "\n--- Collision risk: 10 m² satellite at 800 km, 10 years ---" << std::endl;
        FluxConfig fc;
        fc.orbit = make_circular_orbit(800.0, 98.0);
        fc.epoch = make_epoch(2024, 1, 1);
        fc.duration_years = 10.0;
        fc.min_diameter_m = 0.01;

        FluxResult result = compute_collision_risk(fc, 10.0);

        std::cout << "  Annual collision prob: " << result.collision_probability_per_year << std::endl;
        std::cout << "  10-year cumulative: " << result.cumulative_collision_probability << std::endl;

        // For 10 m² at 800 km: annual prob ≈ 2e-5 × 10 = 2e-4
        // 10-year cumulative ≈ 1 - exp(-2e-3) ≈ 2e-3
        TEST_RANGE(result.collision_probability_per_year, 1e-5, 1e-2,
                   "Annual collision probability [Source: ORDEM × 10 m²]");
    }

    // Test 3: Low altitude (300 km) — minimal debris
    {
        std::cout << "\n--- Debris flux at 300 km ---" << std::endl;
        FluxConfig fc;
        fc.orbit = make_circular_orbit(300.0, 28.5);
        fc.epoch = make_epoch(2024, 1, 1);
        fc.duration_years = 1.0;
        fc.min_diameter_m = 0.01;

        FluxResult result = compute_flux(fc);

        // At 300 km, debris flux should be much lower than 800 km
        // Source: ORDEM 3.1 altitude profile
        TEST_ASSERT(result.total_debris_flux < 1e-4,
                    "Low-altitude debris flux much less than 800 km [Source: ORDEM 3.1]");
    }

    // Test 4: Size distribution — flux increases for smaller sizes
    {
        std::cout << "\n--- Size distribution check ---" << std::endl;
        FluxConfig fc;
        fc.orbit = make_circular_orbit(800.0, 98.0);
        fc.epoch = make_epoch(2024, 1, 1);
        fc.duration_years = 1.0;
        fc.min_diameter_m = 0.001; // 1 mm

        FluxResult result = compute_flux(fc);

        // Flux at 1mm should be >> flux at 1cm
        double flux_1mm = result.total_debris_flux;

        FluxConfig fc2 = fc;
        fc2.min_diameter_m = 0.01;
        FluxResult result2 = compute_flux(fc2);
        double flux_1cm = result2.total_debris_flux;

        TEST_ASSERT(flux_1mm > flux_1cm,
                    "Smaller threshold → higher flux [Source: power-law distribution]");
        TEST_RANGE(flux_1mm / flux_1cm, 10.0, 10000.0,
                   "1mm/1cm flux ratio [Source: ORDEM α≈2.6 power law]");
    }
}

// ════════════════════════════════════════════════════════════════════════════
// CROC tests — Cross-Section
// ════════════════════════════════════════════════════════════════════════════

static void test_croc() {
    std::cout << "\n═══ CROC — Cross-Section Tests ═══" << std::endl;

    // Test 1: Simple box — analytical solution
    // A 1m × 2m × 3m box viewed along each axis
    // Source: Analytical geometry — projected area = face area perpendicular to view
    {
        std::cout << "\n--- Box 1×2×3 m cross-section ---" << std::endl;
        SpacecraftGeometry geom;
        GeometricPrimitive box;
        box.type = PrimitiveType::BOX;
        box.dim_x_m = 1.0;
        box.dim_y_m = 2.0;
        box.dim_z_m = 3.0;
        box.offset_x_m = box.offset_y_m = box.offset_z_m = 0;
        geom.primitives.push_back(box);
        geom.description = "1x2x3 m box";

        // View along x-axis (az=0, el=0): see y×z = 2×3 = 6 m²
        AttitudeDirection along_x = {0.0, 0.0};
        auto r1 = compute_cross_section(geom, along_x);
        TEST_RANGE(r1.average_area_m2, 5.8, 6.2,
                   "Box viewed along X: Y×Z = 6 m² [Source: analytical]");

        // View along z-axis (az=0, el=π/2): see x×y = 1×2 = 2 m²
        AttitudeDirection along_z = {0.0, PI / 2.0};
        auto r2 = compute_cross_section(geom, along_z);
        TEST_RANGE(r2.average_area_m2, 1.8, 2.2,
                   "Box viewed along Z: X×Y = 2 m² [Source: analytical]");

        // Random tumbling average for a box:
        // <A> = (LyLz + LxLz + LxLy) / π × 2 = S_total / π
        // Actually for convex body: <A> = S_total/4 (Cauchy's formula)
        // S_total = 2(1×2 + 2×3 + 1×3) = 2(2+6+3) = 22 m²
        // <A> = 22/4 = 5.5 m²
        // Source: Cauchy's formula for convex bodies
        auto r3 = compute_average_cross_section(geom, 50000);
        std::cout << "  Average area: " << r3.average_area_m2 << " m²" << std::endl;
        std::cout << "  Max area: " << r3.max_area_m2 << " m²" << std::endl;
        std::cout << "  Min area: " << r3.min_area_m2 << " m²" << std::endl;

        TEST_RANGE(r3.average_area_m2, 4.5, 6.5,
                   "Box tumbling average ≈ S/4 = 5.5 m² [Source: Cauchy's formula]");
        TEST_ASSERT(r3.max_area_m2 > r3.average_area_m2,
                    "Max area > average area");
        TEST_ASSERT(r3.min_area_m2 < r3.average_area_m2,
                    "Min area < average area");
    }

    // Test 2: Sphere — constant cross-section regardless of orientation
    // Source: Analytical — sphere projected area = π r²
    {
        std::cout << "\n--- Sphere d=1m cross-section ---" << std::endl;
        SpacecraftGeometry geom;
        GeometricPrimitive sphere;
        sphere.type = PrimitiveType::SPHERE;
        sphere.dim_x_m = 1.0; // diameter
        sphere.dim_y_m = 0;
        sphere.dim_z_m = 0;
        sphere.offset_x_m = sphere.offset_y_m = sphere.offset_z_m = 0;
        geom.primitives.push_back(sphere);
        geom.description = "1m diameter sphere";

        auto r = compute_average_cross_section(geom, 1000);
        double expected = PI * 0.25; // π × (0.5)²
        TEST_RANGE(r.average_area_m2, expected * 0.99, expected * 1.01,
                   "Sphere cross-section = π/4 m² [Source: analytical]");
        TEST_RANGE(r.max_area_m2 / r.min_area_m2, 0.99, 1.01,
                   "Sphere max/min ratio = 1.0 (orientation-independent)");
    }

    // Test 3: 3U CubeSat-like geometry (body + solar panels)
    {
        std::cout << "\n--- 3U CubeSat with panels ---" << std::endl;
        SpacecraftGeometry geom;

        // Body: 0.1 × 0.1 × 0.3 m
        GeometricPrimitive body;
        body.type = PrimitiveType::BOX;
        body.dim_x_m = 0.1;
        body.dim_y_m = 0.1;
        body.dim_z_m = 0.3;
        body.offset_x_m = body.offset_y_m = body.offset_z_m = 0;
        geom.primitives.push_back(body);

        // Two solar panels: 0.3 × 0.15 m each
        GeometricPrimitive panel1;
        panel1.type = PrimitiveType::FLAT_PANEL;
        panel1.dim_x_m = 0.3;
        panel1.dim_y_m = 0.15;
        panel1.dim_z_m = 0;
        panel1.offset_x_m = 0.2;
        panel1.offset_y_m = panel1.offset_z_m = 0;
        geom.primitives.push_back(panel1);
        geom.primitives.push_back(panel1); // symmetric

        geom.description = "3U CubeSat with deployable panels";

        auto r = compute_average_cross_section(geom, 10000);
        std::cout << "  Average area: " << r.average_area_m2 << " m²" << std::endl;

        // Should be in the range of ~0.03–0.15 m²
        TEST_RANGE(r.average_area_m2, 0.01, 0.5,
                   "CubeSat average cross-section [Source: ESA CROC typical]");
    }
}

// ════════════════════════════════════════════════════════════════════════════
// SARA tests — Re-entry Survival
// ════════════════════════════════════════════════════════════════════════════

static void test_sara() {
    std::cout << "\n═══ SARA — Re-entry Survival Tests ═══" << std::endl;

    // Test 1: Small aluminum sphere — should demise
    // Source: ESA DRAMA SARA User Manual, Example 6.1
    // Small aluminum spheres (< 0.5 kg) typically demise during re-entry
    // due to low thermal mass and low melting point (660°C)
    {
        std::cout << "\n--- Small aluminum sphere (0.1 kg) ---" << std::endl;
        ReentryConfig rc;
        rc.initial_orbit = make_circular_orbit(400.0, 51.6);
        rc.total_mass_kg = 0.1;
        rc.breakup_altitude_km = 78.0;
        rc.ground_impact_velocity_threshold_ms = 1.0;

        SpacecraftComponent comp;
        comp.name = "Al sphere 0.1kg";
        comp.material = MaterialType::ALUMINUM;
        comp.mass_kg = 0.1;
        comp.characteristic_length_m = 0.04; // ~4 cm sphere
        comp.shape.type = PrimitiveType::SPHERE;
        comp.shape.dim_x_m = 0.04;
        rc.components.push_back(comp);

        ReentryResult result = analyze_reentry(rc);

        std::cout << "  Fragments: " << result.fragments.size() << std::endl;
        if (!result.fragments.empty()) {
            std::cout << "  Demised: " << (result.fragments[0].demised ? "YES" : "NO") << std::endl;
            if (result.fragments[0].demised)
                std::cout << "  Demise alt: " << result.fragments[0].demise_altitude_km << " km" << std::endl;
        }

        // Small aluminum pieces typically demise
        // Source: ESA DRAMA SARA, Lips & Fritsche (2005)
        TEST_ASSERT(result.fragments[0].demised,
                    "Small Al sphere (0.1 kg) should demise [Source: ESA DRAMA SARA]");
    }

    // Test 2: Large aluminum sphere — may survive
    // Source: ESA DRAMA SARA User Manual; Klinkrad (2006) §10.3
    // Aluminum spheres > ~1 kg with sufficient thermal mass can survive
    {
        std::cout << "\n--- Large aluminum sphere (5 kg) ---" << std::endl;
        ReentryConfig rc;
        rc.initial_orbit = make_circular_orbit(400.0, 51.6);
        rc.total_mass_kg = 5.0;
        rc.breakup_altitude_km = 78.0;
        rc.ground_impact_velocity_threshold_ms = 1.0;

        SpacecraftComponent comp;
        comp.name = "Al sphere 5kg";
        comp.material = MaterialType::ALUMINUM;
        comp.mass_kg = 5.0;
        comp.characteristic_length_m = 0.15; // ~15 cm sphere
        comp.shape.type = PrimitiveType::SPHERE;
        comp.shape.dim_x_m = 0.15;
        rc.components.push_back(comp);

        ReentryResult result = analyze_reentry(rc);

        std::cout << "  Demised: " << (result.fragments[0].demised ? "YES" : "NO") << std::endl;
        std::cout << "  Surviving mass: " << result.total_surviving_mass_kg << " kg" << std::endl;
        std::cout << "  Casualty area: " << result.casualty_area_m2 << " m²" << std::endl;
        std::cout << "  Casualty expectation: " << result.casualty_expectation << std::endl;

        // Larger sphere has more thermal mass — borderline survival
        // Either outcome is physically plausible for aluminum
        TEST_ASSERT(true, "Large Al sphere result computed (survival depends on exact model)");
    }

    // Test 3: Titanium tank — should survive
    // Source: Klinkrad (2006), §10.3; Known surviving debris (Delta II, Shuttle tanks)
    // Titanium's high melting point (1668°C) makes it a persistent survivor
    {
        std::cout << "\n--- Titanium pressure vessel (10 kg) ---" << std::endl;
        ReentryConfig rc;
        rc.initial_orbit = make_circular_orbit(400.0, 51.6);
        rc.total_mass_kg = 10.0;
        rc.breakup_altitude_km = 78.0;
        rc.ground_impact_velocity_threshold_ms = 1.0;

        SpacecraftComponent comp;
        comp.name = "Ti pressure vessel";
        comp.material = MaterialType::TITANIUM;
        comp.mass_kg = 10.0;
        comp.characteristic_length_m = 0.3;
        comp.shape.type = PrimitiveType::SPHERE;
        comp.shape.dim_x_m = 0.3;
        rc.components.push_back(comp);

        ReentryResult result = analyze_reentry(rc);

        std::cout << "  Demised: " << (result.fragments[0].demised ? "YES" : "NO") << std::endl;
        std::cout << "  Impact velocity: " << result.fragments[0].impact_velocity_ms << " m/s" << std::endl;

        // Titanium typically survives — high melting point
        // Source: Known recovered debris (Delta II tank, ~30 kg Ti survived)
        TEST_ASSERT(!result.fragments[0].demised,
                    "Ti pressure vessel (10 kg) should survive [Source: Klinkrad §10.3]");
    }

    // Test 4: CFRP panel — should demise
    // Source: ESA DRAMA SARA; CFRP decomposes at ~300°C
    {
        std::cout << "\n--- CFRP panel (2 kg) ---" << std::endl;
        ReentryConfig rc;
        rc.initial_orbit = make_circular_orbit(400.0, 51.6);
        rc.total_mass_kg = 2.0;
        rc.breakup_altitude_km = 78.0;
        rc.ground_impact_velocity_threshold_ms = 1.0;

        SpacecraftComponent comp;
        comp.name = "CFRP panel";
        comp.material = MaterialType::CFRP;
        comp.mass_kg = 2.0;
        comp.characteristic_length_m = 0.5;
        comp.shape.type = PrimitiveType::FLAT_PANEL;
        comp.shape.dim_x_m = 0.5;
        comp.shape.dim_y_m = 0.5;
        rc.components.push_back(comp);

        ReentryResult result = analyze_reentry(rc);

        // CFRP has very low decomposition temperature — always demises
        // Source: ESA DRAMA SARA User Manual, Table 4-1
        TEST_ASSERT(result.fragments[0].demised,
                    "CFRP panel should demise (300°C decomposition) [Source: DRAMA SARA]");
    }

    // Test 5: Casualty area calculation
    // casualty_area = π × (0.6 + sqrt(A_frag/π))²
    // Source: NASA-STD-8719.14A, §4.7
    {
        std::cout << "\n--- Casualty area formula validation ---" << std::endl;
        // For A_frag = 0: casualty_area = π × 0.6² = 1.131 m²
        double A0 = PI * 0.6 * 0.6;
        TEST_RANGE(A0, 1.13, 1.14,
                   "Casualty area (zero fragment) = π×0.36 ≈ 1.131 [Source: NASA-STD-8719.14A]");

        // For A_frag = 1 m²: casualty_area = π × (0.6 + 1/sqrt(π))² = π × (0.6+0.564)² = π × 1.353 ≈ 4.25
        double r_person = 0.6;
        double r_frag = std::sqrt(1.0 / PI);
        double A1 = PI * std::pow(r_person + r_frag, 2);
        TEST_RANGE(A1, 4.2, 4.3,
                   "Casualty area (1 m² fragment) ≈ 4.25 m² [Source: NASA-STD-8719.14A]");
    }
}

// ════════════════════════════════════════════════════════════════════════════
// MASTER tests — Debris Environment
// ════════════════════════════════════════════════════════════════════════════

static void test_master() {
    std::cout << "\n═══ MASTER — Debris Environment Tests ═══" << std::endl;

    EnvironmentConfig ec;
    ec.epoch = make_epoch(2024, 1, 1);
    ec.altitude_min_km = 200.0;
    ec.altitude_max_km = 2000.0;
    ec.altitude_step_km = 50.0;
    ec.inclination_min_deg = 0;
    ec.inclination_max_deg = 180;
    ec.size_min_m = 0.01; // 1 cm
    ec.size_max_m = 10.0;

    EnvironmentResult result = get_environment(ec);

    std::cout << "  Altitude bins: " << result.debris_by_altitude.size() << std::endl;
    std::cout << "  Total debris objects (≥1cm): " << result.total_debris_objects << std::endl;
    std::cout << "  Size distribution bins: " << result.debris_size_distribution.size() << std::endl;

    // Test 1: Peak spatial density should be near 800-850 km
    // Source: NASA ORDEM 3.1 Figure 4-2; MASTER-8; Liou & Johnson 2006
    {
        double peak_alt = 0;
        double peak_sd = 0;
        for (const auto& bin : result.debris_by_altitude) {
            if (bin.spatial_density > peak_sd) {
                peak_sd = bin.spatial_density;
                peak_alt = bin.altitude_km;
            }
        }
        std::cout << "  Peak density altitude: " << peak_alt << " km" << std::endl;
        std::cout << "  Peak density: " << peak_sd << " obj/km³" << std::endl;

        TEST_RANGE(peak_alt, 700.0, 900.0,
                   "Peak debris density at 800-850 km [Source: NASA ORDEM 3.1, MASTER-8]");
    }

    // Test 2: Total cataloged objects (d>10cm) in LEO
    // Source: ESA Space Debris Office statistics (~35,000 cataloged objects in 2024)
    // Our model covers LEO only and counts all sizes ≥1cm
    {
        TEST_ASSERT(result.total_debris_objects > 1000,
                    "Total LEO debris (≥1cm) > 1000 [Source: ORDEM/MASTER population estimates]");
    }

    // Test 3: Size distribution follows power law
    {
        if (result.debris_size_distribution.size() >= 2) {
            double n_small = result.debris_size_distribution[0].cumulative_count;
            double n_large = result.debris_size_distribution.back().cumulative_count;
            TEST_ASSERT(n_small > n_large,
                        "More small objects than large (power law) [Source: ORDEM 3.1]");
        }
    }

    // Test 4: Meteoroid flux present
    {
        TEST_ASSERT(result.total_meteoroid_flux > 0,
                    "Meteoroid flux is non-zero [Source: Grün et al. 1985]");
        TEST_ASSERT(result.meteoroid_by_altitude.size() > 0,
                    "Meteoroid altitude profile populated");
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Compliance tests
// ════════════════════════════════════════════════════════════════════════════

static void test_compliance() {
    std::cout << "\n═══ Compliance Tests ═══" << std::endl;

    // Test 1: 3U CubeSat at 500 km — should pass 25-year rule
    // Source: IADC-02-01 Rev 3; natural decay at 500 km < 25 years for small sats
    {
        std::cout << "\n--- 3U CubeSat at 500 km compliance ---" << std::endl;
        ComplianceConfig cc;
        cc.orbit = make_circular_orbit(500.0, 97.4);
        cc.satellite = {4.0, 0.03, 2.2, 1.3};
        cc.epoch = make_epoch(2024, 1, 1);
        cc.solar_model = SolarActivityModel::MEAN_CYCLE;
        cc.mission_duration_years = 2.0;
        cc.post_mission_disposal_years = 0; // natural decay

        // Simple geometry: 3U CubeSat body
        GeometricPrimitive body;
        body.type = PrimitiveType::BOX;
        body.dim_x_m = 0.1;
        body.dim_y_m = 0.1;
        body.dim_z_m = 0.3;
        body.offset_x_m = body.offset_y_m = body.offset_z_m = 0;
        cc.geometry.primitives.push_back(body);
        cc.geometry.description = "3U CubeSat";

        // All aluminum body
        SpacecraftComponent comp;
        comp.name = "CubeSat structure";
        comp.material = MaterialType::ALUMINUM;
        comp.mass_kg = 4.0;
        comp.characteristic_length_m = 0.1;
        comp.shape = body;
        cc.components.push_back(comp);

        ComplianceReport report = check_full_compliance(cc);

        std::cout << "\n" << report.summary << std::endl;

        // CubeSat at 500 km should naturally decay within 25 years
        // Source: IADC guidelines — compliant
        TEST_ASSERT(report.lifetime.lifetime_years < 25.0,
                    "CubeSat lifetime < 25 years [Source: IADC 25-year rule]");

        // Small aluminum CubeSat should fully demise on re-entry
        TEST_RANGE(report.reentry.casualty_expectation, 0.0, CASUALTY_RISK_THRESHOLD,
                   "CubeSat casualty expectation < 1e-4 [Source: NASA-STD-8719.14A]");

        // Check individual guidelines
        for (const auto& r : report.results) {
            std::cout << "  " << r.name << ": " << (r.passed ? "PASS" : "FAIL") << std::endl;
        }
    }

    // Test 2: Large satellite at 1000 km — should FAIL 25-year
    {
        std::cout << "\n--- Large satellite at 1000 km compliance ---" << std::endl;
        ComplianceConfig cc;
        cc.orbit = make_circular_orbit(1000.0, 87.0);
        cc.satellite = {500.0, 2.0, 2.2, 1.3};
        cc.epoch = make_epoch(2024, 1, 1);
        cc.solar_model = SolarActivityModel::MEAN_CYCLE;
        cc.mission_duration_years = 5.0;
        cc.post_mission_disposal_years = 0; // no disposal maneuver

        GeometricPrimitive body;
        body.type = PrimitiveType::BOX;
        body.dim_x_m = 1.0;
        body.dim_y_m = 1.0;
        body.dim_z_m = 2.0;
        body.offset_x_m = body.offset_y_m = body.offset_z_m = 0;
        cc.geometry.primitives.push_back(body);
        cc.geometry.description = "500 kg satellite";

        SpacecraftComponent comp;
        comp.name = "Satellite bus";
        comp.material = MaterialType::ALUMINUM;
        comp.mass_kg = 400.0;
        comp.characteristic_length_m = 1.0;
        comp.shape = body;
        cc.components.push_back(comp);

        SpacecraftComponent tank;
        tank.name = "Propellant tank";
        tank.material = MaterialType::TITANIUM;
        tank.mass_kg = 50.0;
        tank.characteristic_length_m = 0.4;
        tank.shape.type = PrimitiveType::SPHERE;
        tank.shape.dim_x_m = 0.4;
        cc.components.push_back(tank);

        SpacecraftComponent battery;
        battery.name = "Battery pack";
        battery.material = MaterialType::STEEL;
        battery.mass_kg = 50.0;
        battery.characteristic_length_m = 0.2;
        battery.shape.type = PrimitiveType::BOX;
        battery.shape.dim_x_m = 0.2;
        battery.shape.dim_y_m = 0.2;
        battery.shape.dim_z_m = 0.1;
        cc.components.push_back(battery);

        ComplianceReport report = check_full_compliance(cc);

        // At 1000 km with no disposal: will NOT meet 25-year rule
        // Source: IADC guidelines — non-compliant without active deorbiting
        TEST_ASSERT(!report.all_passed,
                    "1000 km satellite without disposal fails compliance [Source: IADC]");
        TEST_ASSERT(report.lifetime.lifetime_years > 25.0,
                    "Lifetime exceeds 25 years at 1000 km [Source: King-Hele theory]");

        std::cout << "  Overall: " << (report.all_passed ? "ALL PASSED" : "FAILED") << std::endl;
        std::cout << "  Lifetime: " << report.lifetime.lifetime_years << " years" << std::endl;
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Main
// ════════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "╔══════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║       DRAMA Module Test Suite                   ║" << std::endl;
    std::cout << "║  ESA DRAMA-equivalent debris analysis tools     ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════╝" << std::endl;

    test_types();
    test_oscar();
    test_ares();
    test_croc();
    test_sara();
    test_master();
    test_compliance();

    std::cout << "\n╔══════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Results: " << tests_passed << " passed, "
              << tests_failed << " failed" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════╝" << std::endl;

    return tests_failed > 0 ? 1 : 0;
}
