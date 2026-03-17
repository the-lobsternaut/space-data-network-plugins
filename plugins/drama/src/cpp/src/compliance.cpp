/**
 * @file compliance.cpp
 * @brief Regulatory Compliance Assessment
 *
 * Checks spacecraft mission against debris mitigation guidelines:
 *   - IADC Space Debris Mitigation Guidelines (2020 revision)
 *   - ISO 24113:2019 Space Debris Mitigation Requirements
 *   - FCC 25-year rule (47 CFR §25.114, updated 2022 to 5-year proposal)
 *   - French Space Operations Act (FSOA / Loi Relative aux Opérations Spatiales)
 *   - ESA Space Debris Mitigation Policy (ESA/ADMIN/IPOL(2014)2)
 *
 * References:
 *   - IADC-02-01, Rev 3 (2021)
 *   - ISO 24113:2019, Space systems — Space debris mitigation requirements
 *   - 47 CFR §25.114(d)(14) — FCC orbital debris mitigation
 *   - French Space Operations Act (LOI n° 2008-518, 3 June 2008)
 *   - ESA DRAMA Compliance Tool User Manual
 */
#include "../include/types.h"
#include "../include/constants.h"
#include <cmath>
#include <sstream>
#include <algorithm>

namespace drama {

// Forward declarations (implemented in other modules)
extern LifetimeResult compute_lifetime(const LifetimeConfig& config);
extern FluxResult compute_flux(const FluxConfig& config);
extern FluxResult compute_collision_risk(const FluxConfig& config, double cross_section_m2);
extern CrossSectionResult compute_average_cross_section(const SpacecraftGeometry& geom, int n_samples);
extern ReentryResult analyze_reentry(const ReentryConfig& config);

// ════════════════════════════════════════════════════════════════════════════
// Individual guideline checks
// ════════════════════════════════════════════════════════════════════════════

/**
 * IADC Space Debris Mitigation Guidelines check.
 *
 * Key requirements (IADC-02-01 Rev 3):
 *   1. Limit debris released during normal operations
 *   2. Minimize breakup potential (passivation)
 *   3. Post-mission disposal:
 *      a. LEO (< 2000 km): orbital lifetime ≤ 25 years after EOL
 *      b. GEO: re-orbit to graveyard (GEO + 235 km minimum)
 *   4. Minimize collision risk during active mission
 *   5. Casualty risk from re-entry < 1:10,000
 */
static GuidelineResult check_iadc(const ComplianceConfig& config,
                                   const LifetimeResult& lifetime,
                                   const ReentryResult& reentry) {
    GuidelineResult gr;
    gr.guideline = ComplianceGuideline::IADC_DEBRIS_MITIGATION;
    gr.name = "IADC Space Debris Mitigation Guidelines (IADC-02-01 Rev 3)";
    gr.passed = true;

    double perigee_km = config.orbit.perigee_altitude_km();
    double apogee_km = config.orbit.apogee_altitude_km();

    // Check 1: Post-mission disposal (25-year rule for LEO)
    if (perigee_km < LEO_PROTECTED_ALTITUDE_KM) {
        // LEO regime: must deorbit within 25 years of EOL
        double post_mission_lifetime = lifetime.lifetime_years;
        // Account for planned PMD maneuver
        double effective_lifetime = post_mission_lifetime;
        if (config.post_mission_disposal_years > 0) {
            effective_lifetime = config.post_mission_disposal_years;
        }

        if (effective_lifetime > POST_MISSION_LIFETIME_YEARS) {
            gr.passed = false;
            std::ostringstream oss;
            oss << "FAIL: Post-mission orbital lifetime " << effective_lifetime
                << " years exceeds 25-year limit";
            gr.findings.push_back(oss.str());
        } else {
            std::ostringstream oss;
            oss << "PASS: Post-mission orbital lifetime " << effective_lifetime
                << " years within 25-year limit";
            gr.findings.push_back(oss.str());
        }
    }

    // Check 2: GEO protected region
    double geo_alt = 35786.0; // km
    if (std::abs(perigee_km - geo_alt) < 500.0 || std::abs(apogee_km - geo_alt) < 500.0) {
        // Near GEO: must re-orbit to graveyard
        if (apogee_km < GEO_GRAVEYARD_MIN_KM) {
            gr.passed = false;
            gr.findings.push_back("FAIL: GEO spacecraft must re-orbit to graveyard orbit "
                                  "(GEO + 235 km minimum)");
        } else {
            gr.findings.push_back("PASS: Disposal orbit above GEO graveyard altitude");
        }
    }

    // Check 3: Passivation
    // Can't fully check without propulsion system details; note recommendation
    gr.findings.push_back("NOTE: Passivation of energy sources at EOL required "
                          "(deplete propellant, discharge batteries, vent pressure vessels)");

    // Check 4: Re-entry casualty risk
    if (reentry.casualty_expectation >= CASUALTY_RISK_THRESHOLD) {
        gr.passed = false;
        std::ostringstream oss;
        oss << "FAIL: Casualty expectation " << reentry.casualty_expectation
            << " exceeds 1:10,000 threshold";
        gr.findings.push_back(oss.str());
    } else {
        std::ostringstream oss;
        oss << "PASS: Casualty expectation " << reentry.casualty_expectation
            << " below 1:10,000 threshold";
        gr.findings.push_back(oss.str());
    }

    std::ostringstream detail;
    detail << "Assessed against IADC-02-01 Rev 3 (2021). ";
    detail << "Orbital lifetime: " << lifetime.lifetime_years << " years. ";
    detail << "Casualty expectation: " << reentry.casualty_expectation;
    gr.details = detail.str();

    return gr;
}

/**
 * ISO 24113:2019 check.
 *
 * Key requirements:
 *   1. Same 25-year post-mission disposal as IADC
 *   2. Probability of breakup < 0.001 per year in LEO
 *   3. Casualty risk < 1e-4 per re-entry event
 *   4. No intentional breakups that generate long-lived debris
 *   5. Debris release during deployment ≤ acceptable levels
 */
static GuidelineResult check_iso24113(const ComplianceConfig& config,
                                       const LifetimeResult& lifetime,
                                       const ReentryResult& reentry,
                                       const FluxResult& flux) {
    GuidelineResult gr;
    gr.guideline = ComplianceGuideline::ISO_24113;
    gr.name = "ISO 24113:2019 Space Debris Mitigation Requirements";
    gr.passed = true;

    // 25-year disposal
    double effective_lifetime = lifetime.lifetime_years;
    if (config.post_mission_disposal_years > 0) {
        effective_lifetime = config.post_mission_disposal_years;
    }

    if (config.orbit.perigee_altitude_km() < LEO_PROTECTED_ALTITUDE_KM) {
        if (effective_lifetime > POST_MISSION_LIFETIME_YEARS) {
            gr.passed = false;
            std::ostringstream oss;
            oss << "FAIL [§6.3.1]: Orbital lifetime " << effective_lifetime
                << " yr exceeds 25-year PMD requirement";
            gr.findings.push_back(oss.str());
        } else {
            gr.findings.push_back("PASS [§6.3.1]: 25-year PMD requirement satisfied");
        }
    }

    // Casualty risk
    if (reentry.casualty_expectation >= CASUALTY_RISK_THRESHOLD) {
        gr.passed = false;
        std::ostringstream oss;
        oss << "FAIL [§6.3.3]: Casualty expectation " << reentry.casualty_expectation
            << " exceeds 1e-4";
        gr.findings.push_back(oss.str());
    } else {
        gr.findings.push_back("PASS [§6.3.3]: Casualty expectation below 1e-4");
    }

    // Collision avoidance capability
    double annual_collision_prob = flux.collision_probability_per_year;
    if (annual_collision_prob > 1e-4) {
        gr.findings.push_back("WARNING [§6.2.2]: Annual collision probability > 1e-4. "
                              "Active collision avoidance recommended");
    } else {
        gr.findings.push_back("PASS [§6.2.2]: Collision probability within acceptable range");
    }

    gr.details = "Assessed against ISO 24113:2019. Full compliance requires "
                 "additional documentation per §5 (Debris Mitigation Plan).";

    return gr;
}

/**
 * FCC 25-year rule (47 CFR §25.114).
 *
 * As of 2022 NPRM, the FCC proposed reducing to 5 years, but 25 years
 * remains the current enforceable rule.
 *
 * Requirements:
 *   1. Post-mission disposal within 25 years (LEO)
 *   2. Disclosure of collision risk assessment
 *   3. Casualty risk assessment for controlled/uncontrolled re-entry
 */
static GuidelineResult check_fcc(const ComplianceConfig& config,
                                  const LifetimeResult& lifetime,
                                  const ReentryResult& reentry) {
    GuidelineResult gr;
    gr.guideline = ComplianceGuideline::FCC_25_YEAR;
    gr.name = "FCC 25-Year Orbital Debris Rule (47 CFR §25.114)";
    gr.passed = true;

    double effective_lifetime = lifetime.lifetime_years;
    if (config.post_mission_disposal_years > 0) {
        effective_lifetime = config.post_mission_disposal_years;
    }

    if (config.orbit.perigee_altitude_km() < LEO_PROTECTED_ALTITUDE_KM) {
        if (effective_lifetime > POST_MISSION_LIFETIME_YEARS) {
            gr.passed = false;
            std::ostringstream oss;
            oss << "FAIL: Post-mission orbital lifetime " << effective_lifetime
                << " years exceeds FCC 25-year limit";
            gr.findings.push_back(oss.str());
        } else {
            std::ostringstream oss;
            oss << "PASS: Orbital lifetime " << effective_lifetime
                << " years complies with 25-year rule";
            gr.findings.push_back(oss.str());
        }
    }

    // Casualty risk
    if (reentry.casualty_expectation >= CASUALTY_RISK_THRESHOLD) {
        gr.passed = false;
        std::ostringstream oss;
        oss << "FAIL: Casualty risk " << reentry.casualty_expectation
            << " exceeds 1:10,000 threshold";
        gr.findings.push_back(oss.str());
    } else {
        gr.findings.push_back("PASS: Casualty risk within acceptable limits");
    }

    gr.details = "Assessed against 47 CFR §25.114(d)(14). "
                 "Note: FCC proposed reducing to 5-year rule in 2022 NPRM (FCC 22-74).";

    return gr;
}

/**
 * French Space Operations Act (FSOA / LOS) check.
 *
 * Loi n° 2008-518 du 3 juin 2008 relative aux opérations spatiales.
 * Implemented through Technical Regulations (Arrêté du 31 mars 2011).
 *
 * Key requirements:
 *   1. 25-year post-mission disposal (LEO)
 *   2. Passivation at EOL
 *   3. Casualty risk < 1e-4 for uncontrolled re-entry
 *   4. Collision risk mitigation
 *   5. Limitation of mission-related debris
 */
static GuidelineResult check_fsoa(const ComplianceConfig& config,
                                   const LifetimeResult& lifetime,
                                   const ReentryResult& reentry) {
    GuidelineResult gr;
    gr.guideline = ComplianceGuideline::FRENCH_SPACE_OPS_ACT;
    gr.name = "French Space Operations Act (Loi n° 2008-518, FSOA/LOS)";
    gr.passed = true;

    double effective_lifetime = lifetime.lifetime_years;
    if (config.post_mission_disposal_years > 0) {
        effective_lifetime = config.post_mission_disposal_years;
    }

    if (config.orbit.perigee_altitude_km() < LEO_PROTECTED_ALTITUDE_KM) {
        if (effective_lifetime > POST_MISSION_LIFETIME_YEARS) {
            gr.passed = false;
            std::ostringstream oss;
            oss << "FAIL [Art. R.331-6]: Post-mission lifetime " << effective_lifetime
                << " years exceeds 25-year limit";
            gr.findings.push_back(oss.str());
        } else {
            gr.findings.push_back("PASS [Art. R.331-6]: 25-year disposal compliant");
        }
    }

    if (reentry.casualty_expectation >= CASUALTY_RISK_THRESHOLD) {
        gr.passed = false;
        std::ostringstream oss;
        oss << "FAIL [Art. R.331-8]: Casualty expectation " << reentry.casualty_expectation
            << " exceeds 1e-4";
        gr.findings.push_back(oss.str());
    } else {
        gr.findings.push_back("PASS [Art. R.331-8]: Casualty risk compliant");
    }

    gr.findings.push_back("NOTE: FSOA requires passivation plan documentation "
                          "(Art. R.331-5)");

    gr.details = "Assessed against Arrêté du 31 mars 2011 implementing "
                 "Loi n° 2008-518. CNES is the technical authority.";

    return gr;
}

// ════════════════════════════════════════════════════════════════════════════
// Public API
// ════════════════════════════════════════════════════════════════════════════

ComplianceReport check_full_compliance(const ComplianceConfig& config) {
    ComplianceReport report;
    report.all_passed = true;

    // 1. Compute orbital lifetime (OSCAR)
    LifetimeConfig lc;
    lc.initial_orbit = config.orbit;
    lc.satellite = config.satellite;
    lc.solar_model = config.solar_model;
    lc.solar_override = {F107_MEAN, F107_MEAN, AP_MODERATE};
    lc.epoch = config.epoch;
    lc.max_lifetime_years = 200.0;
    lc.time_step_days = 1.0;
    report.lifetime = compute_lifetime(lc);

    // 2. Compute cross-section (CROC)
    report.cross_section = compute_average_cross_section(config.geometry, 5000);

    // 3. Compute collision flux (ARES)
    FluxConfig fc;
    fc.orbit = config.orbit;
    fc.epoch = config.epoch;
    fc.duration_years = config.mission_duration_years > 0 ? config.mission_duration_years : 5.0;
    fc.min_diameter_m = 0.01; // 1 cm lethal threshold
    report.flux = compute_collision_risk(fc, report.cross_section.average_area_m2);

    // 4. Compute re-entry survival (SARA)
    ReentryConfig rc;
    rc.initial_orbit = config.orbit;
    rc.total_mass_kg = config.satellite.mass_kg;
    rc.components = config.components;
    rc.geometry = config.geometry;
    rc.epoch = config.epoch;
    rc.breakup_altitude_km = 78.0;
    rc.ground_impact_velocity_threshold_ms = 1.0;
    report.reentry = analyze_reentry(rc);

    // 5. Run each guideline check
    GuidelineResult iadc = check_iadc(config, report.lifetime, report.reentry);
    GuidelineResult iso = check_iso24113(config, report.lifetime, report.reentry, report.flux);
    GuidelineResult fcc = check_fcc(config, report.lifetime, report.reentry);
    GuidelineResult fsoa = check_fsoa(config, report.lifetime, report.reentry);

    report.results.push_back(iadc);
    report.results.push_back(iso);
    report.results.push_back(fcc);
    report.results.push_back(fsoa);

    // Overall pass/fail
    for (const auto& r : report.results) {
        if (!r.passed) {
            report.all_passed = false;
            break;
        }
    }

    // Summary
    std::ostringstream summary;
    summary << "Compliance Assessment Summary\n";
    summary << "═══════════════════════════════\n";
    summary << "Orbit: " << config.orbit.perigee_altitude_km() << " × "
            << config.orbit.apogee_altitude_km() << " km, "
            << (config.orbit.inclination_rad * RAD_TO_DEG) << "° inclination\n";
    summary << "Mass: " << config.satellite.mass_kg << " kg, "
            << "Area: " << report.cross_section.average_area_m2 << " m²\n";
    summary << "Orbital lifetime: " << report.lifetime.lifetime_years << " years\n";
    summary << "Casualty expectation: " << report.reentry.casualty_expectation << "\n\n";

    for (const auto& r : report.results) {
        summary << (r.passed ? "✓ PASS" : "✗ FAIL") << " — " << r.name << "\n";
        for (const auto& f : r.findings) {
            summary << "    " << f << "\n";
        }
        summary << "\n";
    }

    summary << "Overall: " << (report.all_passed ? "ALL PASSED" : "ONE OR MORE FAILED") << "\n";
    report.summary = summary.str();

    return report;
}

} // namespace drama
