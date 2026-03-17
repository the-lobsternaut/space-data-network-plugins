/**
 * @file oscar.cpp
 * @brief OSCAR — Orbital Spacecraft Active Removal / Lifetime prediction
 *
 * Implements King-Hele orbital decay theory with exponential atmosphere model.
 * Atmospheric density varies with altitude and solar activity (F10.7, Ap).
 *
 * References:
 *   - King-Hele, D.G. "Satellite Orbits in an Atmosphere: Theory and
 *     Applications", Blackie, 1987
 *   - Vallado, D.A. "Fundamentals of Astrodynamics and Applications",
 *     4th ed., Chapter 9
 *   - CIRA-72 / Harris-Priester atmosphere model
 *   - ESA DRAMA OSCAR User Manual, Issue 3.1
 */
#include "../include/types.h"
#include "../include/constants.h"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace drama {

// ════════════════════════════════════════════════════════════════════════════
// Harris-Priester atmosphere model (CIRA-72 based)
// Tabulated densities for solar min (F10.7=70) and max (F10.7=250)
// Reference: Montenbruck & Gill, "Satellite Orbits", Table 3.1
// ════════════════════════════════════════════════════════════════════════════

struct AtmosphereTableEntry {
    double altitude_km;
    double rho_min;   // kg/m³ at solar minimum
    double rho_max;   // kg/m³ at solar maximum
    double scale_h;   // local scale height [km]
};

// Subset of Harris-Priester atmosphere (100–2000 km)
// Source: Montenbruck & Gill Table 3.1, ESA DRAMA documentation
static const AtmosphereTableEntry HP_TABLE[] = {
    {100.0, 4.974e-07, 4.974e-07, 5.9},
    {120.0, 2.490e-08, 2.490e-08, 11.5},
    {130.0, 8.377e-09, 8.710e-09, 15.8},
    {140.0, 3.899e-09, 4.059e-09, 18.6},
    {150.0, 2.122e-09, 2.215e-09, 21.2},
    {160.0, 1.263e-09, 1.344e-09, 24.1},
    {170.0, 8.008e-10, 8.758e-10, 27.5},
    {180.0, 5.283e-10, 6.010e-10, 31.4},
    {190.0, 3.617e-10, 4.297e-10, 35.7},
    {200.0, 2.557e-10, 3.162e-10, 40.5},
    {210.0, 1.839e-10, 2.396e-10, 44.8},
    {220.0, 1.341e-10, 1.853e-10, 49.0},
    {230.0, 9.949e-11, 1.455e-10, 53.3},
    {240.0, 7.488e-11, 1.157e-10, 57.5},
    {250.0, 5.709e-11, 9.308e-11, 62.2},
    {260.0, 4.403e-11, 7.555e-11, 67.5},
    {270.0, 3.430e-11, 6.182e-11, 73.0},
    {280.0, 2.697e-11, 5.095e-11, 78.2},
    {290.0, 2.139e-11, 4.226e-11, 83.4},
    {300.0, 1.708e-11, 3.526e-11, 88.5},
    {320.0, 1.099e-11, 2.511e-11, 98.0},
    {340.0, 7.214e-12, 1.819e-11, 107.0},
    {360.0, 4.824e-12, 1.337e-11, 116.0},
    {380.0, 3.274e-12, 9.955e-12, 124.0},
    {400.0, 2.249e-12, 7.492e-12, 133.0},
    {420.0, 1.558e-12, 5.684e-12, 141.0},
    {440.0, 1.091e-12, 4.355e-12, 150.0},
    {460.0, 7.701e-13, 3.362e-12, 160.0},
    {480.0, 5.474e-13, 2.612e-12, 170.0},
    {500.0, 3.916e-13, 2.042e-12, 180.0},
    {520.0, 2.819e-13, 1.605e-12, 190.0},
    {540.0, 2.042e-13, 1.267e-12, 201.0},
    {560.0, 1.488e-13, 1.005e-12, 213.0},
    {580.0, 1.092e-13, 7.997e-13, 225.0},
    {600.0, 8.070e-14, 6.390e-13, 238.0},
    {620.0, 6.012e-14, 5.123e-13, 252.0},
    {640.0, 4.519e-14, 4.121e-13, 267.0},
    {660.0, 3.430e-14, 3.325e-13, 283.0},
    {680.0, 2.632e-14, 2.691e-13, 300.0},
    {700.0, 2.043e-14, 2.185e-13, 320.0},
    {750.0, 1.130e-14, 1.329e-13, 370.0},
    {800.0, 6.567e-15, 8.241e-14, 420.0},
    {850.0, 4.036e-15, 5.217e-14, 475.0},
    {900.0, 2.612e-15, 3.371e-14, 530.0},
    {950.0, 1.774e-15, 2.224e-14, 590.0},
    {1000.0, 1.257e-15, 1.497e-14, 660.0},
    {1100.0, 6.925e-16, 7.152e-15, 800.0},
    {1200.0, 4.232e-16, 3.691e-15, 960.0},
    {1400.0, 1.916e-16, 1.140e-15, 1200.0},
    {1600.0, 1.026e-16, 4.130e-16, 1400.0},
    {1800.0, 5.950e-17, 1.720e-16, 1600.0},
    {2000.0, 3.680e-17, 8.200e-17, 1800.0},
};
static const int HP_TABLE_SIZE = sizeof(HP_TABLE) / sizeof(HP_TABLE[0]);

/**
 * Compute atmospheric density using Harris-Priester model interpolation.
 * Interpolates between solar min/max tables using F10.7 proxy.
 * @param alt_km  Altitude [km]
 * @param f107    10.7 cm solar radio flux [sfu]
 * @return density [kg/m³]
 */
static double harris_priester_density(double alt_km, double f107) {
    if (alt_km < HP_TABLE[0].altitude_km) alt_km = HP_TABLE[0].altitude_km;
    if (alt_km > HP_TABLE[HP_TABLE_SIZE - 1].altitude_km) {
        return HP_TABLE[HP_TABLE_SIZE - 1].rho_min * 1e-3; // negligible
    }

    // Find bracketing entries
    int i = 0;
    for (i = 0; i < HP_TABLE_SIZE - 1; i++) {
        if (HP_TABLE[i + 1].altitude_km >= alt_km) break;
    }

    double h0 = HP_TABLE[i].altitude_km;
    double h1 = HP_TABLE[i + 1].altitude_km;
    double H_local = HP_TABLE[i].scale_h; // km

    // Exponential interpolation within bracket
    double rho_min_i = HP_TABLE[i].rho_min * std::exp(-(alt_km - h0) / H_local);
    double rho_max_i = HP_TABLE[i].rho_max * std::exp(-(alt_km - h0) / H_local);

    // Solar activity interpolation: linear blend between min(F70) and max(F250)
    double frac = std::clamp((f107 - F107_LOW) / (F107_HIGH - F107_LOW), 0.0, 1.0);

    // Harris-Priester uses n=2 exponent for diurnal bulge; we use n=2 blend
    // rho = rho_min * (rho_max/rho_min)^(frac^n) with n≈2 simplified to linear
    // for zonal average (no diurnal variation in orbit-average case)
    double rho = rho_min_i + frac * (rho_max_i - rho_min_i);
    return rho;
}

/**
 * Get scale height at given altitude from Harris-Priester table.
 */
static double get_scale_height_km(double alt_km) {
    if (alt_km <= HP_TABLE[0].altitude_km) return HP_TABLE[0].scale_h;
    if (alt_km >= HP_TABLE[HP_TABLE_SIZE - 1].altitude_km) return HP_TABLE[HP_TABLE_SIZE - 1].scale_h;
    for (int i = 0; i < HP_TABLE_SIZE - 1; i++) {
        if (HP_TABLE[i + 1].altitude_km >= alt_km) {
            double t = (alt_km - HP_TABLE[i].altitude_km) / (HP_TABLE[i + 1].altitude_km - HP_TABLE[i].altitude_km);
            return HP_TABLE[i].scale_h + t * (HP_TABLE[i + 1].scale_h - HP_TABLE[i].scale_h);
        }
    }
    return HP_TABLE[HP_TABLE_SIZE - 1].scale_h;
}

/**
 * Get F10.7 for a given fractional year using solar cycle model.
 * 11-year sinusoidal cycle. Phase reference: solar max ~2025.5 (Cycle 25).
 */
static double get_f107(double year_frac, SolarActivityModel model, double f107_override) {
    switch (model) {
        case SolarActivityModel::CONSTANT:
            return f107_override;
        case SolarActivityModel::LOW_ACTIVITY:
            return F107_LOW;
        case SolarActivityModel::HIGH_ACTIVITY:
            return F107_HIGH;
        case SolarActivityModel::MEAN_CYCLE: {
            // Solar cycle 25 peak ~2025.5, period ~11 years
            double phase = TWO_PI * (year_frac - 2025.5) / 11.0;
            double mean = (F107_HIGH + F107_LOW) / 2.0;
            double amp = (F107_HIGH - F107_LOW) / 2.0;
            return mean + amp * std::cos(phase);
        }
    }
    return F107_MEAN;
}

/**
 * King-Hele orbital decay rate.
 *
 * The semi-major axis decay rate due to atmospheric drag for an orbit with
 * eccentricity e is:
 *
 *   da/dt = -2π·a·ρ_p·(a/B)·f(e)
 *
 * where:
 *   B = ballistic coefficient = m/(Cd·A)
 *   ρ_p = density at perigee
 *   f(e) = function of eccentricity involving modified Bessel functions
 *         ≈ (1+e)^(-1/2) · I₀(ae/H) + ... simplified for moderate e
 *
 * For near-circular orbits (e<0.01), this simplifies to:
 *   da/dt = -2π · ρ_p · a² / B
 *
 * For eccentric orbits, King-Hele theory gives (averaged over one orbit):
 *   da/dt = -(2π·a²·ρ_p / B) · exp(-c·(1-e)) · I₀(c·e)
 *
 * where c = a·e/H (scale height parameter) and I₀ is modified Bessel function.
 *
 * Reference: King-Hele (1987), Eq. 5.8; Vallado (2013) §9.4
 */

/**
 * Modified Bessel function I₀(x) — series approximation.
 * Accurate to ~1e-10 for x < 20.
 */
static double bessel_I0(double x) {
    double sum = 1.0;
    double term = 1.0;
    for (int k = 1; k <= 30; k++) {
        term *= (x * x) / (4.0 * k * k);
        sum += term;
        if (std::abs(term) < 1e-15 * std::abs(sum)) break;
    }
    return sum;
}

/**
 * Compute orbital lifetime using King-Hele theory with Harris-Priester atmosphere.
 *
 * Steps perigee altitude down until re-entry at 120 km, recording the
 * orbit history at each step.
 */
LifetimeResult compute_lifetime(const LifetimeConfig& config) {
    LifetimeResult result;
    result.compliant_25yr = false;

    double a = config.initial_orbit.semi_major_axis_m;
    double e = config.initial_orbit.eccentricity;
    double inc = config.initial_orbit.inclination_rad;
    double B_coeff = config.satellite.ballistic_coefficient(); // m/(Cd·A) [kg/m²]

    double dt_days = config.time_step_days > 0 ? config.time_step_days : 1.0;
    double max_years = config.max_lifetime_years > 0 ? config.max_lifetime_years : 200.0;
    double dt_s = dt_days * SECONDS_PER_DAY;

    double t_years = 0.0;
    double epoch_year = config.epoch.fractional_year();

    // Record initial state
    AltitudeRecord rec0;
    rec0.time_years = 0.0;
    rec0.perigee_km = (a * (1.0 - e) - R_EARTH) / 1000.0;
    rec0.apogee_km = (a * (1.0 + e) - R_EARTH) / 1000.0;
    rec0.eccentricity = e;
    result.history.push_back(rec0);

    // Sampling interval for history (every ~7 days or every step if step > 7 days)
    double history_interval_years = 7.0 / DAYS_PER_YEAR;
    double next_history = history_interval_years;

    const double reentry_alt_m = 120.0 * 1000.0 + R_EARTH; // 120 km

    while (t_years < max_years) {
        double rp = a * (1.0 - e); // perigee radius [m]
        double hp_km = (rp - R_EARTH) / 1000.0; // perigee altitude [km]

        if (rp <= reentry_alt_m) {
            // Re-entry reached
            break;
        }

        // Get atmospheric density at perigee
        double current_year = epoch_year + t_years;
        double f107 = get_f107(current_year, config.solar_model, config.solar_override.f107);
        double rho_p = harris_priester_density(hp_km, f107);

        // Scale height at perigee [m]
        double H = get_scale_height_km(hp_km) * 1000.0;

        // King-Hele decay: orbit-averaged da/dt
        // For eccentric orbits: da/dt = -(2π a² ρ_p / B) · I₀(a·e/H) · exp(-a·e/H)
        // The perigee density already includes the exp(-h_p/H) factor from the atmosphere
        // model, so we compute the Bessel-function correction for eccentricity.
        double c = a * e / H; // eccentricity parameter
        double bessel_factor;
        if (c < 0.001) {
            // Near-circular: I₀(c)·exp(-c) ≈ 1
            bessel_factor = 1.0;
        } else if (c > 50.0) {
            // Large c asymptotic: I₀(c)·exp(-c) ≈ 1/sqrt(2πc)
            bessel_factor = 1.0 / std::sqrt(TWO_PI * c);
        } else {
            bessel_factor = bessel_I0(c) * std::exp(-c);
        }

        // The King-Hele formula gives Δa per revolution, not per second.
        // Convert to da/dt [m/s] by dividing by the orbital period T = 2π√(a³/μ).
        double T_orbit = TWO_PI * std::sqrt(a * a * a / MU_EARTH);
        double da_per_rev = -(TWO_PI * a * a * rho_p / B_coeff) * bessel_factor;
        double da_dt = da_per_rev / T_orbit;

        // Eccentricity decay (King-Hele): de/dt ≈ -(da/dt) / (2a) · (1 - e)
        // For near-circular, e decays faster than a
        // Reference: King-Hele (1987) Eq 5.17
        double de_dt;
        if (e > 1e-6) {
            // Full King-Hele eccentricity decay
            double I1_approx; // I₁(c)/I₀(c) approximation
            if (c < 0.01) {
                I1_approx = c / 2.0;
            } else if (c > 50.0) {
                I1_approx = 1.0 - 1.0 / (2.0 * c);
            } else {
                // I₁(c)/I₀(c) ≈ 1 - 1/(2c) for moderate c, compute properly
                double I0 = bessel_I0(c);
                // I₁ from series
                double sum = 0.0, term = c / 2.0;
                sum = term;
                for (int k = 1; k <= 30; k++) {
                    term *= (c * c) / (4.0 * k * (k + 1));
                    sum += term;
                    if (std::abs(term) < 1e-15 * std::abs(sum)) break;
                }
                I1_approx = sum / I0;
            }
            de_dt = (da_dt / a) * (1.0 - e) / 2.0 * (1.0 - I1_approx);
        } else {
            de_dt = 0.0;
        }

        // Adaptive time stepping: shrink step when decay is fast
        double alt_change = std::abs(da_dt * dt_s);
        double effective_dt = dt_s;
        if (alt_change > 5000.0) { // > 5 km per step, shrink
            effective_dt = std::max(60.0, dt_s * 5000.0 / alt_change);
        }

        // Update orbital elements
        a += da_dt * effective_dt;
        e += de_dt * effective_dt;
        if (e < 0.0) e = 0.0;
        if (e > 0.99) e = 0.99; // bound

        // J2 secular effect on RAAN (for record keeping, doesn't affect decay)
        // inc remains constant for atmospheric drag (no out-of-plane force)

        t_years += effective_dt / SECONDS_PER_YEAR;

        // Record history at intervals
        if (t_years >= next_history) {
            AltitudeRecord rec;
            rec.time_years = t_years;
            rec.perigee_km = (a * (1.0 - e) - R_EARTH) / 1000.0;
            rec.apogee_km = (a * (1.0 + e) - R_EARTH) / 1000.0;
            rec.eccentricity = e;
            result.history.push_back(rec);
            next_history = t_years + history_interval_years;
        }
    }

    // Final record
    AltitudeRecord final_rec;
    final_rec.time_years = t_years;
    final_rec.perigee_km = (a * (1.0 - e) - R_EARTH) / 1000.0;
    final_rec.apogee_km = (a * (1.0 + e) - R_EARTH) / 1000.0;
    final_rec.eccentricity = e;
    result.history.push_back(final_rec);

    result.lifetime_years = t_years;
    double decay_mjd = config.epoch.to_mjd() + t_years * DAYS_PER_YEAR;
    result.decay_date = EpochTime::from_mjd(decay_mjd);
    result.compliant_25yr = (t_years <= POST_MISSION_LIFETIME_YEARS);

    std::ostringstream oss;
    oss << "King-Hele decay with Harris-Priester atmosphere. ";
    oss << "Initial perigee: " << config.initial_orbit.perigee_altitude_km() << " km, ";
    oss << "apogee: " << config.initial_orbit.apogee_altitude_km() << " km. ";
    oss << "Ballistic coeff: " << B_coeff << " kg/m². ";
    oss << "Lifetime: " << t_years << " years.";
    result.notes = oss.str();

    return result;
}

/**
 * Check if orbit satisfies the 25-year post-mission disposal rule.
 */
bool check_25yr_compliance(const LifetimeConfig& config) {
    LifetimeResult lr = compute_lifetime(config);
    return lr.compliant_25yr;
}

} // namespace drama
