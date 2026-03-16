/**
 * @file constants.h
 * @brief Physical and mathematical constants for DRAMA computations
 *
 * Reference values from IAU, IERS, and ESA standards.
 */
#pragma once

#include <cmath>

namespace drama {

// ── Mathematical constants ──────────────────────────────────────────────────
constexpr double PI          = 3.14159265358979323846;
constexpr double TWO_PI      = 2.0 * PI;
constexpr double DEG_TO_RAD  = PI / 180.0;
constexpr double RAD_TO_DEG  = 180.0 / PI;

// ── Earth parameters (WGS-84) ───────────────────────────────────────────────
constexpr double MU_EARTH    = 3.986004418e14;     // m³/s² — Earth gravitational parameter
constexpr double R_EARTH     = 6378137.0;           // m — Earth equatorial radius
constexpr double R_EARTH_KM  = 6378.137;            // km
constexpr double J2          = 1.08263e-3;           // Earth J2 oblateness
constexpr double OMEGA_EARTH = 7.2921159e-5;        // rad/s — Earth rotation rate
constexpr double EARTH_SURFACE_AREA = 5.1e14;       // m² — Earth surface area
constexpr double WORLD_POPULATION   = 8.0e9;        // approximate (2024)
constexpr double AVG_POP_DENSITY    = WORLD_POPULATION / EARTH_SURFACE_AREA; // people/m²

// ── Atmosphere reference (CIRA/MSIS-like exponential) ───────────────────────
constexpr double RHO_0       = 1.225;               // kg/m³ — sea-level density
constexpr double H_SCALE_0   = 8500.0;              // m — base scale height

// ── Solar/geomagnetic ───────────────────────────────────────────────────────
constexpr double F107_MEAN   = 150.0;               // sfu — moderate solar activity
constexpr double F107_LOW    = 70.0;                 // sfu — solar minimum
constexpr double F107_HIGH   = 250.0;               // sfu — solar maximum
constexpr double AP_QUIET    = 4.0;                  // quiet geomagnetic activity
constexpr double AP_MODERATE = 15.0;
constexpr double AP_ACTIVE   = 48.0;

// ── Solar radiation ─────────────────────────────────────────────────────────
constexpr double P_SRP       = 4.56e-6;             // N/m² — solar radiation pressure at 1 AU
constexpr double AU          = 1.496e11;             // m — astronomical unit

// ── Time ────────────────────────────────────────────────────────────────────
constexpr double SECONDS_PER_DAY  = 86400.0;
constexpr double DAYS_PER_YEAR    = 365.25;
constexpr double SECONDS_PER_YEAR = SECONDS_PER_DAY * DAYS_PER_YEAR;

// ── Debris environment (power-law coefficients from MASTER/ORDEM) ───────────
// Cumulative spatial density: S(d) = A * d^(-alpha)
// where d = diameter in meters, S in objects/km³
// These are fits to MASTER-8 population at ~800 km, epoch 2020
constexpr double DEBRIS_POWER_LAW_ALPHA  = 2.6;       // exponent
constexpr double DEBRIS_POWER_LAW_A_800  = 3.6e-8;    // coefficient at 800 km [obj/km³] for d>1cm

// Grün meteoroid flux model coefficients (log10 form)
// F(m) = cumulative flux [impacts/m²/yr] for mass >= m [g]
// Reference: Grün et al. 1985, Icarus 62, 244-272

// ── Re-entry / thermal ─────────────────────────────────────────────────────
constexpr double STEFAN_BOLTZMANN = 5.670374419e-8;  // W/m²/K⁴
constexpr double BOLTZMANN_K     = 1.380649e-23;     // J/K
constexpr double R_GAS           = 8.314;             // J/(mol·K) — universal gas constant

// ── Compliance thresholds ───────────────────────────────────────────────────
constexpr double CASUALTY_RISK_THRESHOLD = 1.0e-4;   // max acceptable casualty expectation
constexpr double POST_MISSION_LIFETIME_YEARS = 25.0;  // IADC/FCC 25-year rule
constexpr double LEO_PROTECTED_ALTITUDE_KM = 2000.0;  // LEO protected region upper bound
constexpr double GEO_GRAVEYARD_MIN_KM = 35786.0 + 200.0; // GEO + 200 km graveyard

} // namespace drama
