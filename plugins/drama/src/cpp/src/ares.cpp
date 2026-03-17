/**
 * @file ares.cpp
 * @brief ARES — Assessment of Risk Event Statistics (Collision Risk)
 *
 * Computes debris and meteoroid flux using:
 *   - NASA ORDEM 3.1 power-law debris population model
 *   - Grün et al. (1985) interplanetary meteoroid flux model
 *   - Poisson collision probability
 *
 * References:
 *   - Krisko, P.H. "NASA's New Orbital Debris Engineering Model ORDEM 3.1",
 *     AIAA 2014-4227
 *   - Grün, E. et al. "Collisional Balance of the Meteoritic Complex",
 *     Icarus 62, 244-272 (1985)
 *   - Kessler, D.J. "Derivation of the Collision Probability between
 *     Orbiting Objects", NASA JSC-20001, 1981
 *   - Vallado, D.A. "Fundamentals of Astrodynamics", 4th ed., §9.6
 */
#include "../include/types.h"
#include "../include/constants.h"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace drama {

// ════════════════════════════════════════════════════════════════════════════
// NASA ORDEM 3.1 debris flux model (parametric fit)
// ════════════════════════════════════════════════════════════════════════════

/**
 * Altitude-dependent spatial density scaling factor.
 *
 * Fitted to ORDEM 3.1 / MASTER-8 population data.
 * LEO debris peaks at 800-850 km and 1400 km (Fengyun / Cosmos-Iridium).
 *
 * @param alt_km  Altitude [km]
 * @return relative density factor (1.0 at 800 km reference)
 */
static double altitude_density_factor(double alt_km) {
    // Two-Gaussian fit to LEO debris spatial density
    // Peak 1: 800 km (primary), Peak 2: 1400 km (secondary)
    // Reference: NASA ORDEM 3.1 Figure 4-2; Liou & Johnson (2006)
    double g1 = std::exp(-0.5 * std::pow((alt_km - 825.0) / 100.0, 2));
    double g2 = 0.4 * std::exp(-0.5 * std::pow((alt_km - 1400.0) / 80.0, 2));

    // Background linear decrease above LEO
    double bg = std::max(0.01, 1.0 - (alt_km - 200.0) / 3000.0);

    return std::max(0.001, g1 + g2 + 0.1 * bg);
}

/**
 * ORDEM power-law cumulative debris flux for a given size threshold.
 *
 * Cumulative flux F(d) = number of impacts per m² per year for objects
 * with diameter >= d.
 *
 * F(d) = F_ref × (d / d_ref)^(-α) × altitude_factor
 *
 * Power-law exponents by size regime (fitted to ORDEM 3.1):
 *   d < 1 mm:   α ≈ 3.5 (sub-mm surface degradation population)
 *   1 mm–1 cm:  α ≈ 2.6 (explosion fragment population)
 *   1 cm–10 cm: α ≈ 2.0 (lethal non-trackable)
 *   > 10 cm:    α ≈ 1.6 (cataloged objects)
 *
 * Reference values at 800 km, epoch 2020 (ORDEM 3.1):
 *   F(≥1cm)  ≈ 2.0e-5 /m²/yr
 *   F(≥1mm)  ≈ 3.6e-3 /m²/yr
 *   F(≥10cm) ≈ 3.7e-7 /m²/yr
 *
 * Source: Krisko (2014), NASA ORDEM 3.1 Technical Report
 */
static double ordem_debris_flux(double diameter_m, double alt_km) {
    double alt_factor = altitude_density_factor(alt_km);

    // Reference flux at 800 km for d >= 1 cm (0.01 m)
    // Source: NASA ORDEM 3.1, Table 5-1
    double F_ref_1cm = 2.0e-5;  // impacts/m²/yr at 800 km

    double flux;
    if (diameter_m >= 0.1) {
        // Cataloged (>10 cm): α = 1.6
        flux = F_ref_1cm * std::pow(diameter_m / 0.01, -1.6);
    } else if (diameter_m >= 0.01) {
        // Lethal non-trackable (1-10 cm): α = 2.0
        flux = F_ref_1cm * std::pow(diameter_m / 0.01, -2.0);
    } else if (diameter_m >= 0.001) {
        // Explosion fragments (1 mm - 1 cm): α = 2.6
        double F_at_1cm = F_ref_1cm;
        flux = F_at_1cm * std::pow(diameter_m / 0.01, -2.6);
    } else {
        // Sub-mm (< 1 mm): α = 3.5, dominated by solid rocket motor slag
        double F_at_1mm = F_ref_1cm * std::pow(0.001 / 0.01, -2.6);
        flux = F_at_1mm * std::pow(diameter_m / 0.001, -3.5);
    }

    return flux * alt_factor;
}

// ════════════════════════════════════════════════════════════════════════════
// Grün meteoroid flux model
// ════════════════════════════════════════════════════════════════════════════

/**
 * Grün interplanetary meteoroid flux model.
 *
 * Cumulative flux of meteoroids with mass >= m impacting a randomly
 * oriented flat plate at 1 AU.
 *
 * F(m) in impacts/m²/s where m in grams.
 *
 * The Grün model is a piecewise power-law fit:
 *   log10(F) = Σ aᵢ (log10(m))^i
 *
 * This implementation uses the simplified form from Grün et al. (1985):
 *   F(m) = (2.2e3 m^0.306 + 15.0)^(-4.38)
 *        + 1.3e-9 (m + 10^11 m² + 10^27 m⁴)^(-0.36)
 *        + 1.3e-16 (m + 10^6 m²)^(-0.85)
 *
 * Reference: Grün et al. 1985, Icarus 62, 244-272, Eq. 7
 *
 * @param mass_g  Meteoroid mass threshold [grams]
 * @return Cumulative flux [impacts/m²/s] at 1 AU
 */
static double grun_meteoroid_flux_per_s(double mass_g) {
    if (mass_g <= 0) return 0;

    double m = mass_g;

    double term1 = std::pow(2.2e3 * std::pow(m, 0.306) + 15.0, -4.38);
    double term2 = 1.3e-9 * std::pow(m + 1e11 * m * m + 1e27 * m * m * m * m, -0.36);
    double term3 = 1.3e-16 * std::pow(m + 1e6 * m * m, -0.85);

    return term1 + term2 + term3;
}

/**
 * Convert meteoroid diameter to mass assuming spherical shape.
 * Meteoroid density: 2500 kg/m³ (stony), Reference: Cour-Palais (1969)
 * @param diameter_m  Diameter [m]
 * @return mass [grams]
 */
static double meteoroid_diameter_to_mass_g(double diameter_m) {
    double rho = 2500.0; // kg/m³
    double vol = (PI / 6.0) * diameter_m * diameter_m * diameter_m;
    return rho * vol * 1000.0; // kg → grams
}

/**
 * Gravitational focusing factor for Earth.
 * Increases meteoroid flux near Earth relative to free-space.
 * G = 1 + (v_esc/v_inf)² where v_esc = sqrt(2μ/r), v_inf ≈ 20 km/s
 * Reference: Grün et al. (1985), Section 2
 */
static double gravitational_focusing(double alt_km) {
    double r = (R_EARTH_KM + alt_km) * 1000.0;
    double v_esc = std::sqrt(2.0 * MU_EARTH / r); // m/s
    double v_inf = 20000.0; // m/s — average meteoroid geocentric velocity
    return 1.0 + (v_esc * v_esc) / (v_inf * v_inf);
}

// ════════════════════════════════════════════════════════════════════════════
// Public API
// ════════════════════════════════════════════════════════════════════════════

FluxResult compute_flux(const FluxConfig& config) {
    FluxResult result;
    result.total_debris_flux = 0;
    result.total_meteoroid_flux = 0;

    double alt_km = config.orbit.mean_altitude_km();
    double min_d = config.min_diameter_m > 0 ? config.min_diameter_m : 0.001; // default 1 mm

    // Debris flux by size bin
    // Size bins: 1mm, 2mm, 5mm, 1cm, 2cm, 5cm, 10cm, 20cm, 50cm, 1m
    double size_bins[] = {0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0};
    int n_bins = sizeof(size_bins) / sizeof(size_bins[0]);

    for (int i = 0; i < n_bins; i++) {
        if (size_bins[i] < min_d) continue;
        FluxBySize fbs;
        fbs.diameter_m = size_bins[i];
        fbs.flux_per_m2_yr = ordem_debris_flux(size_bins[i], alt_km);
        result.debris_flux_by_size.push_back(fbs);
    }

    // Total debris flux at minimum size
    result.total_debris_flux = ordem_debris_flux(min_d, alt_km);

    // Meteoroid flux by size bin
    double grav_focus = gravitational_focusing(alt_km);
    for (int i = 0; i < n_bins; i++) {
        if (size_bins[i] < min_d) continue;
        FluxBySize fbs;
        fbs.diameter_m = size_bins[i];
        double mass_g = meteoroid_diameter_to_mass_g(size_bins[i]);
        // Convert from /m²/s to /m²/yr and apply gravitational focusing
        fbs.flux_per_m2_yr = grun_meteoroid_flux_per_s(mass_g) * SECONDS_PER_YEAR * grav_focus;
        result.meteoroid_flux_by_size.push_back(fbs);
    }

    // Total meteoroid flux at minimum size
    double min_mass_g = meteoroid_diameter_to_mass_g(min_d);
    result.total_meteoroid_flux = grun_meteoroid_flux_per_s(min_mass_g) * SECONDS_PER_YEAR * grav_focus;

    // Total flux
    double total_flux = result.total_debris_flux + result.total_meteoroid_flux;

    // Collision probability using Poisson statistics
    // P(collision) = 1 - exp(-F × A × T)
    // where F = total flux, A = cross-section, T = exposure duration
    // Cross-section estimation: for flux-only computation (no explicit geometry),
    // we report flux per unit area (m²). The collision probability per year
    // is the flux itself (impacts/m²/yr), and cumulative probability over
    // the mission duration uses Poisson statistics for a 1 m² reference area.
    // To get actual collision probability, use compute_collision_risk() with
    // the spacecraft's true cross-sectional area.
    result.collision_probability_per_year = total_flux; // per m² per year
    result.cumulative_collision_probability = 1.0 - std::exp(-total_flux * config.duration_years);

    return result;
}

FluxResult compute_collision_risk(const FluxConfig& config, double cross_section_m2) {
    FluxResult result = compute_flux(config);

    // Scale probabilities by actual cross-section
    double total_flux = result.total_debris_flux + result.total_meteoroid_flux;
    double lambda = total_flux * cross_section_m2 * config.duration_years;

    result.collision_probability_per_year = total_flux * cross_section_m2;
    result.cumulative_collision_probability = 1.0 - std::exp(-lambda);

    return result;
}

} // namespace drama
