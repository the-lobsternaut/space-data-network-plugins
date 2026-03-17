#pragma once
#include "da_pipeline/types.h"
#include <vector>

namespace da_pipeline {

class TrajectorySimulator {
public:
    TrajectorySimulator() = default;

    /// Simulate ascent trajectory using RK4 integration
    std::vector<StateVector> simulate(
        const AscentModelParams& params,
        double t_end,
        double dt = 0.5) const;

    /// Convert geodetic to ECEF (m)
    static Vec3 geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m);

    /// Convert ECEF to geodetic
    static LatLonAlt ecef_to_geodetic(const Vec3& ecef);

    /// Gravity vector at position (m, m/s^2) — J2 included
    static Vec3 gravity(const Vec3& pos);

    /// Exponential atmosphere density model (kg/m^3)
    static double atm_density(double alt_m);

    /// Determine which stage is active at time t
    static int active_stage(const AscentModelParams& params, double t);
};

} // namespace da_pipeline
