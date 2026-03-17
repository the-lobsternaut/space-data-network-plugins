#pragma once
#include "ascent_reconstruct/types.h"
#include <vector>

namespace ascent_reconstruct {

class TrajectorySimulator {
public:
    // Simulate ascent trajectory given model parameters
    std::vector<StateVector> simulate(const AscentModelParams& params,
                                       double t_end,
                                       double dt = 0.5) const;

    // Convert geodetic to ECEF
    static Vec3 geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m);

    // Convert ECEF to geodetic
    static LatLonAlt ecef_to_geodetic(const Vec3& ecef);

    // Compute gravity at position (simple inverse-square + J2)
    static Vec3 gravity(const Vec3& pos);

    // Compute atmospheric density (exponential model)
    static double atm_density(double alt_m);

    // Compute drag acceleration
    static Vec3 drag_acceleration(const Vec3& vel, double alt_m,
                                    double cd, double area, double mass);

    // Compute thrust direction (gravity turn)
    static Vec3 thrust_direction(const Vec3& pos, const Vec3& vel,
                                   double pitch_from_vertical);

    // Determine which stage is active at time t
    static int active_stage(const AscentModelParams& params, double t);

    // Get start time of a given stage
    static double stage_start_time(const AscentModelParams& params, int stage);

private:
    // RK4 derivative function for powered flight
    struct FlightState {
        Vec3 pos;  // ECEF m
        Vec3 vel;  // ECEF m/s
        double mass; // kg
    };

    struct FlightDerivs {
        Vec3 dpos;
        Vec3 dvel;
        double dmass;
    };

    FlightDerivs derivatives(const FlightState& state,
                              const AscentModelParams& params,
                              double t) const;
};

} // namespace ascent_reconstruct
