/**
 * @file croc.cpp
 * @brief CROC — Cross-Section of Complex Bodies
 *
 * Computes geometric cross-section of spacecraft from primitive shapes.
 * Monte Carlo random orientation averaging for tumbling spacecraft.
 *
 * References:
 *   - ESA DRAMA CROC User Manual
 *   - Klinkrad, H. "Space Debris: Models and Risk Analysis", Springer, 2006
 *   - Stickler, A.C. & Alfriend, K.T. "Elementary Magnetic Attitude
 *     Control System", AIAA J. Spacecraft, 1976
 */
#include "../include/types.h"
#include "../include/constants.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>

namespace drama {

// ════════════════════════════════════════════════════════════════════════════
// Projected area computation for primitives
// ════════════════════════════════════════════════════════════════════════════

/**
 * Compute the projected area of a geometric primitive along a viewing direction.
 *
 * The viewing direction is defined by azimuth (az) and elevation (el) angles:
 *   n̂ = (cos(el)·cos(az), cos(el)·sin(az), sin(el))
 *
 * For each primitive:
 *   BOX:       A_proj = |Lx·ny·nz| + |Ly·nx·nz| + |Lz·nx·ny|
 *              (sum of three face pair projections)
 *   CYLINDER:  Combines circular end-cap and curved surface projections
 *   SPHERE:    A_proj = π·r² (constant regardless of orientation)
 *   FLAT_PANEL: A_proj = Lx·Ly·|nz| (single-sided plate)
 *
 * Reference: Klinkrad (2006), §4.3
 */
static double projected_area(const GeometricPrimitive& prim, double az, double el) {
    double nx = std::cos(el) * std::cos(az);
    double ny = std::cos(el) * std::sin(az);
    double nz = std::sin(el);

    switch (prim.type) {
        case PrimitiveType::BOX: {
            // Box with dimensions Lx × Ly × Lz
            // Projected area = sum of |face normal · view direction| × face area
            // Two faces perpendicular to each axis
            double Lx = prim.dim_x_m;
            double Ly = prim.dim_y_m;
            double Lz = prim.dim_z_m;
            return Ly * Lz * std::abs(nx) +
                   Lx * Lz * std::abs(ny) +
                   Lx * Ly * std::abs(nz);
        }
        case PrimitiveType::CYLINDER: {
            // Cylinder: diameter = dim_x, height = dim_y
            // axis along body z-axis
            double d = prim.dim_x_m;
            double r = d / 2.0;
            double h = prim.dim_y_m;

            // End-cap projection: π·r²·|nz|
            double cap_proj = PI * r * r * std::abs(nz);

            // Curved surface projection: d·h·sqrt(nx²+ny²) / π × 2
            // Actually: projected area of cylinder side = d × h × sin(θ)
            // where θ = angle between view direction and cylinder axis
            double sin_theta = std::sqrt(nx * nx + ny * ny);
            double side_proj = d * h * sin_theta;

            return cap_proj + side_proj;
        }
        case PrimitiveType::SPHERE: {
            // Sphere: diameter = dim_x
            double r = prim.dim_x_m / 2.0;
            return PI * r * r; // constant
        }
        case PrimitiveType::FLAT_PANEL: {
            // Flat panel in XY plane: dim_x × dim_y
            // normal along body z-axis
            return prim.dim_x_m * prim.dim_y_m * std::abs(nz);
        }
    }
    return 0.0;
}

/**
 * Compute total projected area of a spacecraft geometry at a given attitude.
 *
 * Note: This is a simplified union — it sums projections without accounting
 * for inter-body shadowing. For complex geometries, this overestimates
 * slightly but is standard for DRAMA-level analysis.
 */
static double total_projected_area(const SpacecraftGeometry& geom, double az, double el) {
    double total = 0.0;
    for (const auto& prim : geom.primitives) {
        total += projected_area(prim, az, el);
    }
    return total;
}

// ════════════════════════════════════════════════════════════════════════════
// Public API
// ════════════════════════════════════════════════════════════════════════════

/**
 * Compute cross-section at a specific attitude direction.
 */
CrossSectionResult compute_cross_section(const SpacecraftGeometry& geom,
                                          const AttitudeDirection& attitude) {
    CrossSectionResult result;

    double area = total_projected_area(geom, attitude.az_rad, attitude.el_rad);

    CrossSectionAtAttitude csa;
    csa.attitude = attitude;
    csa.area_m2 = area;
    result.area_vs_attitude.push_back(csa);

    result.average_area_m2 = area;
    result.max_area_m2 = area;
    result.min_area_m2 = area;

    return result;
}

/**
 * Compute average cross-section for random tumbling using Monte Carlo.
 *
 * For a randomly tumbling spacecraft, the orbit-averaged cross-section is:
 *   <A> = (1/4π) ∫∫ A(θ,φ) sin(θ) dθ dφ
 *
 * This is computed via Monte Carlo sampling with uniform distribution on
 * the unit sphere. Convergence is typically achieved with N ≈ 5000 samples.
 *
 * For a convex body, an exact result exists:
 *   <A> = S_total / 4     (¼ of total surface area)
 * This provides a useful validation check.
 *
 * Reference: ESA DRAMA CROC User Manual, §3.2; Cauchy's formula
 *
 * @param geom  Spacecraft geometry
 * @param n_samples  Number of Monte Carlo samples (default 10000)
 * @return Cross-section statistics
 */
CrossSectionResult compute_average_cross_section(const SpacecraftGeometry& geom,
                                                  int n_samples) {
    if (n_samples <= 0) n_samples = 10000;

    CrossSectionResult result;
    result.max_area_m2 = 0;
    result.min_area_m2 = 1e30;
    double sum_area = 0;

    // Use deterministic seed for reproducibility
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    // Sample uniformly on the sphere
    // az ∈ [0, 2π), el = arcsin(2u - 1) for u ∈ [0, 1]
    // This gives cos(el) weighting automatically

    // Also record a grid of attitudes for the area_vs_attitude table
    // Use a coarser grid for the table (every 10°)
    int n_az_grid = 36;  // 0° to 350°
    int n_el_grid = 19;  // -90° to 90°

    for (int i_az = 0; i_az < n_az_grid; i_az++) {
        for (int i_el = 0; i_el < n_el_grid; i_el++) {
            double az = i_az * 10.0 * DEG_TO_RAD;
            double el = (-90.0 + i_el * 10.0) * DEG_TO_RAD;
            double area = total_projected_area(geom, az, el);

            CrossSectionAtAttitude csa;
            csa.attitude = {az, el};
            csa.area_m2 = area;
            result.area_vs_attitude.push_back(csa);
        }
    }

    // Monte Carlo for average
    for (int i = 0; i < n_samples; i++) {
        double az = TWO_PI * u01(rng);
        double el = std::asin(2.0 * u01(rng) - 1.0);
        double area = total_projected_area(geom, az, el);

        sum_area += area;
        result.max_area_m2 = std::max(result.max_area_m2, area);
        result.min_area_m2 = std::min(result.min_area_m2, area);
    }

    result.average_area_m2 = sum_area / n_samples;
    return result;
}

} // namespace drama
