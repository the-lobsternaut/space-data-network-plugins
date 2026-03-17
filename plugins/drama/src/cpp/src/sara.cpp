/**
 * @file sara.cpp
 * @brief SARA — Survival Analysis for Re-entry Applications
 *
 * Lumped-mass ablation model for spacecraft re-entry survival analysis.
 * Determines which components survive re-entry and computes casualty risk.
 *
 * References:
 *   - Lips, T. & Fritsche, B. "A comparison of commonly used re-entry
 *     analysis tools", Acta Astronautica 57, 312-323 (2005)
 *   - Detra, R.W., Kemp, N.H. & Riddell, F.R. "Addendum to Heat Transfer
 *     to Satellite Vehicles Re-entering the Atmosphere", Jet Propulsion
 *     28(10), 1958
 *   - Sutton, K. & Graves, R.A. "A General Stagnation-Point Convective
 *     Heating Equation for Arbitrary Gas Mixtures", NASA TR R-376, 1971
 *   - ESA DRAMA SARA User Manual, Issue 3.0
 *   - Klinkrad, H. "Space Debris", Springer, 2006, Chapter 10
 */
#include "../include/types.h"
#include "../include/constants.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <vector>

namespace drama {

// ════════════════════════════════════════════════════════════════════════════
// Atmospheric model for re-entry (US Standard Atmosphere 1976 simplified)
// ════════════════════════════════════════════════════════════════════════════

/**
 * Simplified atmospheric density for re-entry trajectory.
 * Exponential model fitted to US Standard Atmosphere 1976.
 * Valid from 60-120 km.
 */
static double reentry_density(double alt_km) {
    // Piecewise exponential fit to USSA76
    // Reference: Vallado (2013), Table 8-4
    if (alt_km > 120.0) {
        double rho_120 = 2.49e-8; // kg/m³
        double H = 11.5; // km
        return rho_120 * std::exp(-(alt_km - 120.0) / H);
    } else if (alt_km > 100.0) {
        double rho_100 = 5.60e-7;
        double H = 5.9;
        return rho_100 * std::exp(-(alt_km - 100.0) / H);
    } else if (alt_km > 80.0) {
        double rho_80 = 1.85e-5;
        double H = 6.6;
        return rho_80 * std::exp(-(alt_km - 80.0) / H);
    } else if (alt_km > 60.0) {
        double rho_60 = 3.10e-4;
        double H = 7.7;
        return rho_60 * std::exp(-(alt_km - 60.0) / H);
    } else {
        double rho_0 = 1.225;
        double H = 8.5;
        return rho_0 * std::exp(-alt_km / H);
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Stagnation point heating
// ════════════════════════════════════════════════════════════════════════════

/**
 * Stagnation-point convective heating rate using Detra-Kemp-Riddell correlation.
 *
 *   q̇ = C × sqrt(ρ/r_n) × V³
 *
 * where:
 *   C = 1.7415e-4 [W·s³/(kg^0.5·m)]  — Detra-Kemp-Riddell constant
 *   ρ = freestream density [kg/m³]
 *   r_n = nose radius [m]
 *   V = velocity [m/s]
 *
 * Reference: Detra, Kemp & Riddell (1958), modified by Sutton-Graves
 *
 * @param rho     Atmospheric density [kg/m³]
 * @param v       Velocity [m/s]
 * @param r_nose  Effective nose radius [m]
 * @return Stagnation heating rate [W/m²]
 */
static double stagnation_heating_dkr(double rho, double v, double r_nose) {
    // Sutton-Graves constant for air: C = 1.7415e-4
    // Reference: NASA TR R-376, Table 2 (N₂/O₂ mixture)
    constexpr double C_SG = 1.7415e-4;
    return C_SG * std::sqrt(rho / r_nose) * v * v * v;
}

/**
 * Effective nose radius for a given shape and size.
 * For blunt shapes, r_n ≈ characteristic_length / 2.
 * For flat plates, r_n → ∞ (zero stagnation heating, use flat plate correlation).
 * We use equivalent sphere radius.
 */
static double effective_nose_radius(double char_length_m) {
    return std::max(0.001, char_length_m / 2.0);
}

// ════════════════════════════════════════════════════════════════════════════
// Lumped mass ablation model
// ════════════════════════════════════════════════════════════════════════════

/**
 * Simulate re-entry of a single component using lumped-mass ablation model.
 *
 * The model integrates the thermal energy balance:
 *   m·Cp·dT/dt = q̇_in·A - ε·σ·T⁴·A
 *
 * where:
 *   q̇_in = stagnation point heating (convective + radiative)
 *   A = wetted surface area (≈ cross-sectional area)
 *   ε = emissivity
 *   σ = Stefan-Boltzmann constant
 *   T = component temperature
 *
 * When T reaches the melting point:
 *   - If heat_of_fusion > 0: additional energy Q_melt = m × L_f must be
 *     absorbed before the component fully demises
 *   - Demise = complete melting/ablation
 *
 * The trajectory is a simplified ballistic entry from breakup altitude:
 *   dh/dt = -V_vertical
 *   V = sqrt(V_h² + V_v²)
 *   dV/dt = -D/m + g·sin(γ)    (drag deceleration + gravity)
 *
 * Reference: ESA DRAMA SARA User Manual §4.2, Lips & Fritsche (2005)
 *
 * @param comp     Component properties
 * @param breakup_alt_km  Altitude of parent spacecraft breakup [km]
 * @return Surviving fragment description
 */
static SurvivingFragment simulate_component_reentry(
        const SpacecraftComponent& comp,
        double breakup_alt_km) {

    MaterialProperties mat = MaterialProperties::get(comp.material);
    SurvivingFragment frag;
    frag.component_name = comp.name;
    frag.material = comp.material;
    frag.mass_kg = comp.mass_kg;
    frag.demised = false;
    frag.demise_altitude_km = 0;
    frag.peak_heating_kw_m2 = 0;
    frag.impact_velocity_ms = 0;
    frag.impact_energy_j = 0;
    frag.area_m2 = 0;

    // Characteristic dimensions
    double char_len = comp.characteristic_length_m;
    if (char_len <= 0) char_len = 0.1;
    double r_nose = effective_nose_radius(char_len);

    // Cross-sectional area (sphere equivalent)
    double A_cross = PI * (char_len / 2.0) * (char_len / 2.0);

    // Initial conditions at breakup
    double alt = breakup_alt_km;         // km
    double V_horiz = 7500.0;             // m/s (typical orbital velocity)
    double V_vert = 0.0;                 // m/s (initially horizontal)
    double gamma = -1.0 * DEG_TO_RAD;    // flight path angle (shallow entry)
    double T_body = 300.0;               // K (initial temperature)
    double mass = comp.mass_kg;
    double Cd = 2.2;                     // drag coefficient

    // Melting energy budget
    double Q_melt_total = mass * mat.heat_of_fusion_j_kg;
    double Q_melt_absorbed = 0;
    bool melting = false;

    double dt = 0.1; // time step [s]
    double peak_heating = 0;

    for (double t = 0; t < 2000.0; t += dt) {
        if (alt <= 0) break; // ground impact

        double rho = reentry_density(alt);
        double V = std::sqrt(V_horiz * V_horiz + V_vert * V_vert);
        if (V < 1.0) V = 1.0;

        // Drag force
        double D = 0.5 * rho * V * V * Cd * A_cross;

        // Stagnation heating
        double q_dot = stagnation_heating_dkr(rho, V, r_nose);
        peak_heating = std::max(peak_heating, q_dot);

        // Radiative cooling
        double q_rad = mat.emissivity * STEFAN_BOLTZMANN * T_body * T_body * T_body * T_body;

        // Net heating
        double q_net = q_dot - q_rad; // W/m²

        // Temperature update
        if (!melting) {
            double dT = (q_net * A_cross * dt) / (mass * mat.specific_heat_j_kg_k);
            T_body += dT;

            if (T_body >= mat.melting_point_k) {
                T_body = mat.melting_point_k;
                melting = true;
            }
        } else {
            // Absorbing latent heat at melting point
            Q_melt_absorbed += q_net * A_cross * dt;
            if (Q_melt_absorbed >= Q_melt_total) {
                // Component has fully demised
                frag.demised = true;
                frag.demise_altitude_km = alt;
                frag.peak_heating_kw_m2 = peak_heating / 1000.0;
                frag.mass_kg = 0;
                return frag;
            }
        }

        // Trajectory integration (simplified ballistic)
        double g = MU_EARTH / std::pow((R_EARTH + alt * 1000.0), 2); // m/s²
        double a_drag = D / mass;

        // Decelerate along velocity vector
        V_horiz -= a_drag * (V_horiz / V) * dt;
        V_vert += (g * std::sin(gamma) - a_drag * (V_vert / V)) * dt;

        // Path angle evolution
        if (V > 10.0) {
            gamma = std::atan2(-V_vert, V_horiz);
        }
        // Steepen as object loses horizontal velocity
        if (V_horiz > 0 && alt < 60.0) {
            V_vert += g * dt * 0.1; // gravity pull
        }

        // Altitude change
        double dh_km = -(V_vert * dt) / 1000.0;
        // At high altitude, mainly horizontal; gravity slowly pulls down
        if (alt > 60.0) {
            dh_km = -std::abs(V * std::sin(std::max(-gamma, 0.5 * DEG_TO_RAD))) * dt / 1000.0;
        }
        alt += dh_km;
        if (alt < 0) alt = 0;

        // Adaptive time step
        if (alt < 40.0 && dt > 0.01) dt = 0.01;
    }

    // Survived to ground
    double V_final = std::sqrt(V_horiz * V_horiz + V_vert * V_vert);
    // Terminal velocity for dense objects
    double rho_0 = 1.225;
    double V_terminal = std::sqrt(2.0 * mass * 9.81 / (rho_0 * Cd * A_cross));
    V_final = std::min(V_final, V_terminal);

    frag.impact_velocity_ms = V_final;
    frag.impact_energy_j = 0.5 * mass * V_final * V_final;
    frag.peak_heating_kw_m2 = peak_heating / 1000.0;

    // Casualty area: π × (0.6 + sqrt(A_frag/π))²
    // where A_frag is the fragment cross-section and 0.6 m is the person radius
    // Reference: NASA-STD-8719.14A, §4.7
    frag.area_m2 = PI * std::pow(0.6 + std::sqrt(A_cross / PI), 2);

    return frag;
}

// ════════════════════════════════════════════════════════════════════════════
// Public API
// ════════════════════════════════════════════════════════════════════════════

ReentryResult analyze_reentry(const ReentryConfig& config) {
    ReentryResult result;
    result.total_surviving_mass_kg = 0;
    result.casualty_area_m2 = 0;
    result.casualty_expectation = 0;
    result.ground_footprint_km2 = 0;

    double breakup_alt = config.breakup_altitude_km > 0 ? config.breakup_altitude_km : 78.0;

    for (const auto& comp : config.components) {
        SurvivingFragment frag = simulate_component_reentry(comp, breakup_alt);
        result.fragments.push_back(frag);

        if (!frag.demised) {
            result.total_surviving_mass_kg += frag.mass_kg;
            result.casualty_area_m2 += frag.area_m2;
        }
    }

    // Ground footprint estimation
    // Along-track dispersion ≈ ±800 km for typical LEO re-entry
    // Cross-track ≈ ±20 km
    // Reference: Klinkrad (2006), §10.4
    result.ground_footprint_km2 = 1600.0 * 40.0; // ~64,000 km²

    // Casualty expectation = casualty_area × population_density
    // World average: ~16 people/km² = 1.6e-5 people/m²
    // Reference: NASA-STD-8719.14A, uses latitude-dependent population
    // We use global average as conservative estimate
    result.casualty_expectation = result.casualty_area_m2 * AVG_POP_DENSITY;

    result.compliant_1e4 = (result.casualty_expectation < CASUALTY_RISK_THRESHOLD);

    std::ostringstream oss;
    oss << "SARA re-entry analysis. Breakup at " << breakup_alt << " km. ";
    oss << result.fragments.size() << " components analyzed. ";
    int survived = 0;
    for (const auto& f : result.fragments) {
        if (!f.demised) survived++;
    }
    oss << survived << " survived, " << (result.fragments.size() - survived) << " demised. ";
    oss << "Total surviving mass: " << result.total_surviving_mass_kg << " kg. ";
    oss << "Casualty area: " << result.casualty_area_m2 << " m². ";
    oss << "Casualty expectation: " << result.casualty_expectation;
    if (result.compliant_1e4) oss << " (COMPLIANT < 1e-4)";
    else oss << " (NON-COMPLIANT >= 1e-4)";
    result.notes = oss.str();

    return result;
}

double compute_casualty_risk(const ReentryResult& result) {
    return result.casualty_expectation;
}

} // namespace drama
