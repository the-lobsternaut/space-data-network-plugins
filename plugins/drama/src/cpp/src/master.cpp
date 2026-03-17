/**
 * @file master.cpp
 * @brief MASTER — Meteoroid and Space Debris Terrestrial Environment Reference
 *
 * Provides the space debris and meteoroid environment model:
 *   - Spatial density vs altitude
 *   - Size distribution (cumulative)
 *   - Altitude-dependent flux
 *
 * References:
 *   - Flegel, S. et al. "The MASTER-2009 Space Debris Environment Model",
 *     ESA SP-672, 2009
 *   - Krisko, P.H. "NASA ORDEM 3.1", AIAA 2014-4227
 *   - Liou, J.-C. & Johnson, N.L. "Risks in Space from Orbiting Debris",
 *     Science 311, 340-341 (2006)
 *   - Grün et al. "Collisional Balance of the Meteoritic Complex",
 *     Icarus 62, 244-272 (1985)
 */
#include "../include/types.h"
#include "../include/constants.h"
#include <cmath>
#include <algorithm>

namespace drama {

// ════════════════════════════════════════════════════════════════════════════
// Debris spatial density model
// ════════════════════════════════════════════════════════════════════════════

/**
 * Spatial density of debris objects vs altitude.
 *
 * Fitted to ESA MASTER-8 / NASA ORDEM 3.1 data for objects with d >= size_min.
 *
 * The LEO debris environment has two prominent peaks:
 *   1. 775–850 km: Accumulated breakup fragments (Cosmos-2251/Iridium-33
 *      collision 2009, Chinese ASAT test 2007)
 *   2. 1400 km: Rocket body cluster (upper stages)
 *
 * Spatial density S(h) in objects/km³:
 *   S(h) = A × [G₁(h) + G₂(h) + B(h)]
 *
 * where:
 *   G₁ = exp(-(h-825)²/2σ₁²), σ₁=100 km  (primary peak)
 *   G₂ = 0.35 × exp(-(h-1400)²/2σ₂²), σ₂=80 km (secondary peak)
 *   B(h) = background (MEO/HEO contribution)
 *
 * The coefficient A scales with minimum object size via power law.
 *
 * Source: MASTER-8 population model, epoch 2020
 *         Flegel et al. (2009), updated with 2020 catalog data
 */
static double debris_spatial_density(double alt_km, double size_min_m) {
    // Reference spatial density at 800 km for d >= 1 cm
    // Source: NASA ORDEM 3.1, Figure 4-2
    double S_ref_1cm = 3.6e-8; // objects/km³ for d >= 1 cm at 800 km

    // Scale with size using power law: S(d) ∝ d^(-α)
    // α = 2.6 for explosion fragments (DEBRIS_POWER_LAW_ALPHA)
    double size_factor = std::pow(size_min_m / 0.01, -DEBRIS_POWER_LAW_ALPHA);
    double S_base = S_ref_1cm * size_factor;

    // Altitude profile (two-Gaussian + background)
    double g1 = std::exp(-0.5 * std::pow((alt_km - 825.0) / 100.0, 2));
    double g2 = 0.35 * std::exp(-0.5 * std::pow((alt_km - 1400.0) / 80.0, 2));

    // Background: slow exponential decay from LEO through MEO
    double bg = 0.02 * std::exp(-alt_km / 5000.0);

    return S_base * (g1 + g2 + bg);
}

/**
 * Debris flux from spatial density.
 *
 * Flux F = S × V_rel × (collision cross-section factor)
 *
 * For a target in circular orbit, the average relative velocity with
 * the debris population depends on altitude and inclination distribution:
 *   V_rel ≈ 10 km/s (LEO average)
 *
 * F [impacts/m²/yr] = S [obj/km³] × V_rel [km/yr] × 1e-6 [km²/m²]
 *
 * Reference: Kessler (1981), NASA JSC-20001
 */
static double debris_flux_from_density(double spatial_density, double alt_km) {
    // Average relative velocity in LEO (depends on inclination distribution)
    // Source: Klinkrad (2006), Table 6.1
    double v_rel_km_s;
    if (alt_km < 600) {
        v_rel_km_s = 10.5; // High-inclination dominated
    } else if (alt_km < 1000) {
        v_rel_km_s = 10.0; // Mixed inclination
    } else {
        v_rel_km_s = 8.5;  // More circular, lower relative velocity
    }

    double v_rel_km_yr = v_rel_km_s * SECONDS_PER_YEAR;

    // flux [impacts/m²/yr] = spatial_density [obj/km³] × v_rel [km/yr] × 1e-6 [km²/m²]
    return spatial_density * v_rel_km_yr * 1e-6;
}

/**
 * Meteoroid spatial density using Grün model.
 *
 * Near-Earth meteoroid flux is roughly isotropic and altitude-independent
 * in LEO (shielding by Earth reduces flux by factor ~0.5).
 *
 * Source: Grün et al. (1985), Section 3
 */
static double meteoroid_flux_at_earth(double size_min_m) {
    // Convert diameter to mass (spherical, ρ=2500 kg/m³)
    double vol = (PI / 6.0) * size_min_m * size_min_m * size_min_m;
    double mass_g = 2500.0 * vol * 1000.0;

    // Grün flux model [impacts/m²/s]
    double m = mass_g;
    if (m <= 0) return 0;

    double term1 = std::pow(2.2e3 * std::pow(m, 0.306) + 15.0, -4.38);
    double term2 = 1.3e-9 * std::pow(m + 1e11 * m * m + 1e27 * m * m * m * m, -0.36);
    double term3 = 1.3e-16 * std::pow(m + 1e6 * m * m, -0.85);

    double F_s = term1 + term2 + term3;
    return F_s * SECONDS_PER_YEAR; // convert to /m²/yr
}

// ════════════════════════════════════════════════════════════════════════════
// Size distribution
// ════════════════════════════════════════════════════════════════════════════

/**
 * Cumulative debris object count for d >= diameter at a given altitude shell.
 *
 * N(≥d) = S(h, d) × Volume_shell
 *
 * Shell volume for a thin spherical shell at altitude h:
 *   V_shell = 4π(R_E + h)² × Δh
 *
 * This gives the total number of objects in a 1-km altitude band.
 */
static double cumulative_count_in_shell(double alt_km, double diameter_m, double shell_thickness_km) {
    double r_km = R_EARTH_KM + alt_km;
    double vol_km3 = 4.0 * PI * r_km * r_km * shell_thickness_km;
    double sd = debris_spatial_density(alt_km, diameter_m);
    return sd * vol_km3;
}

// ════════════════════════════════════════════════════════════════════════════
// Public API
// ════════════════════════════════════════════════════════════════════════════

EnvironmentResult get_environment(const EnvironmentConfig& config) {
    EnvironmentResult result;
    result.epoch = config.epoch;
    result.total_debris_objects = 0;
    result.total_meteoroid_flux = 0;

    double alt_min = config.altitude_min_km > 0 ? config.altitude_min_km : 200.0;
    double alt_max = config.altitude_max_km > 0 ? config.altitude_max_km : 2000.0;
    double alt_step = config.altitude_step_km > 0 ? config.altitude_step_km : 50.0;
    double size_min = config.size_min_m > 0 ? config.size_min_m : 0.01; // default 1 cm

    // Debris population by altitude
    for (double h = alt_min; h <= alt_max; h += alt_step) {
        double sd = debris_spatial_density(h, size_min);
        double flux = debris_flux_from_density(sd, h);

        PopulationBin bin;
        bin.altitude_km = h;
        bin.spatial_density = sd;
        bin.flux = flux;
        result.debris_by_altitude.push_back(bin);

        // Accumulate total objects in this shell
        result.total_debris_objects += cumulative_count_in_shell(h, size_min, alt_step);
    }

    // Meteoroid population by altitude (roughly constant, slight Earth shielding)
    for (double h = alt_min; h <= alt_max; h += alt_step) {
        double flux = meteoroid_flux_at_earth(size_min);
        // Earth shielding factor: fraction of sky blocked by Earth
        double r = R_EARTH_KM + h;
        double shield = 0.5 * (1.0 - std::sqrt(1.0 - (R_EARTH_KM * R_EARTH_KM) / (r * r)));
        flux *= (1.0 - shield);

        // Gravitational focusing
        double v_esc = std::sqrt(2.0 * MU_EARTH / (r * 1000.0));
        double v_inf = 20000.0;
        double grav_focus = 1.0 + (v_esc * v_esc) / (v_inf * v_inf);
        flux *= grav_focus;

        PopulationBin bin;
        bin.altitude_km = h;
        bin.spatial_density = 0; // meteoroids are transient, not orbiting
        bin.flux = flux;
        result.meteoroid_by_altitude.push_back(bin);
    }

    // Reference meteoroid flux at 800 km
    result.total_meteoroid_flux = meteoroid_flux_at_earth(size_min);

    // Debris size distribution (cumulative) at reference altitude 800 km
    double size_diameters[] = {0.0001, 0.0003, 0.001, 0.003, 0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0};
    int n_sizes = sizeof(size_diameters) / sizeof(size_diameters[0]);
    for (int i = 0; i < n_sizes; i++) {
        double d = size_diameters[i];
        if (d < config.size_min_m && config.size_min_m > 0) continue;
        if (d > config.size_max_m && config.size_max_m > 0) continue;

        SizeDistribution sd;
        sd.diameter_m = d;
        // Total objects with diameter >= d across all altitudes
        double count = 0;
        for (double h = alt_min; h <= alt_max; h += alt_step) {
            count += cumulative_count_in_shell(h, d, alt_step);
        }
        sd.cumulative_count = count;
        result.debris_size_distribution.push_back(sd);
    }

    // Meteoroid size distribution (Grün model, cumulative flux)
    for (int i = 0; i < n_sizes; i++) {
        double d = size_diameters[i];
        if (d < config.size_min_m && config.size_min_m > 0) continue;
        if (d > config.size_max_m && config.size_max_m > 0) continue;

        SizeDistribution sd;
        sd.diameter_m = d;
        sd.cumulative_count = meteoroid_flux_at_earth(d); // flux as proxy for "count"
        result.meteoroid_size_distribution.push_back(sd);
    }

    return result;
}

} // namespace drama
