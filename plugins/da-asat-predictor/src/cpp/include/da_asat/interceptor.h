#pragma once

#include "da_asat/types.h"
#include <vector>

namespace da_asat {

// ─── DA-ASAT Interceptor Model ─────────────────────────────────────────────
//
// Models a direct-ascent anti-satellite interceptor trajectory from launch
// through boost phase, midcourse, and terminal homing. Uses proportional
// navigation guidance in the terminal phase.

class Interceptor {
public:
    // ── Trajectory point ────────────────────────────────────────────────

    struct TrajectoryPoint {
        double time_sec;
        Vec3   position;    // km, ECI
        Vec3   velocity;    // km/s, ECI
        double mass_kg;
        int    current_stage;
        double altitude_km;
        double downrange_km;
    };

    // ── Trajectory simulation ───────────────────────────────────────────

    // Compute interceptor trajectory with proportional navigation
    std::vector<TrajectoryPoint> simulate_trajectory(
        const LaunchSite& site,
        double launch_azimuth_deg,
        double launch_elevation_deg,
        const InterceptorParams& params,
        const std::vector<OrbitalState>& target_states,
        double dt_sec = 1.0,
        double max_time_sec = 600.0) const;

    // ── Launch solution ─────────────────────────────────────────────────

    struct LaunchSolution {
        double azimuth_deg;
        double elevation_deg;
        double time_of_flight_sec;
        double delta_v_kms;
        bool   converged;
    };

    LaunchSolution solve_launch_params(
        const LaunchSite& site,
        const OrbitalState& target_at_intercept,
        const InterceptorParams& params,
        double intercept_time_sec) const;

    // ── Delta-V computation ─────────────────────────────────────────────

    // Compute delta-V required for direct ascent to intercept point
    double compute_delta_v(const LaunchSite& site,
                           const Vec3& intercept_position,
                           double time_of_flight_sec) const;

    // ── Proportional navigation ─────────────────────────────────────────

    // Proportional navigation guidance law
    Vec3 compute_pn_acceleration(const Vec3& los,
                                  const Vec3& los_rate,
                                  double closing_speed,
                                  double nav_gain = 4.0) const;

    // ── Default interceptor ─────────────────────────────────────────────

    // Default interceptor parameters (generic medium-range DA-ASAT)
    static InterceptorParams default_interceptor();

private:
    static constexpr double PI             = 3.14159265358979323846;
    static constexpr double DEG_TO_RAD     = PI / 180.0;
    static constexpr double RAD_TO_DEG     = 180.0 / PI;
    static constexpr double G0             = 9.80665;            // m/s^2
    static constexpr double MU_EARTH       = 398600.4418;        // km^3/s^2
    static constexpr double EARTH_RADIUS_KM = 6378.137;
    static constexpr double OMEGA_EARTH    = 7.2921159e-5;       // rad/s

    // Atmospheric density model (exponential)
    double atmospheric_density(double altitude_km) const;

    // Convert launch site to ECI position
    Vec3 site_to_eci(const LaunchSite& site, double gmst) const;

    // Aerodynamic drag force
    Vec3 compute_drag(const Vec3& velocity, double altitude_km,
                      double mass_kg, double cd_a = 0.5) const;
};

} // namespace da_asat
