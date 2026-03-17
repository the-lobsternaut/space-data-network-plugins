#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace da_asat {

// ─── Time helpers ───────────────────────────────────────────────────────────

using Timestamp = int64_t; // Unix epoch seconds (UTC)

// ─── Vector / coordinate types ──────────────────────────────────────────────

struct Vec3 {
    double x, y, z;
};

struct LatLon {
    double lat;  // degrees
    double lon;  // degrees
};

struct LatLonAlt {
    double lat;     // degrees
    double lon;     // degrees
    double alt_km;  // above WGS-84 ellipsoid
};

// ─── Two-Line Element Set ──────────────────────────────────────────────────

struct TLE {
    std::string name;
    std::string line1, line2;
    int         norad_id;
    double      epoch_year, epoch_day;
    double      inclination;    // deg
    double      raan;           // deg
    double      eccentricity;
    double      arg_perigee;    // deg
    double      mean_anomaly;   // deg
    double      mean_motion;    // rev/day
    double      bstar;
};

// ─── Orbital state vector ──────────────────────────────────────────────────

struct OrbitalState {
    Vec3      position;  // km, ECI/TEME
    Vec3      velocity;  // km/s
    Timestamp epoch;
};

// ─── Launch site ───────────────────────────────────────────────────────────

struct LaunchSite {
    std::string site_id;
    std::string name;
    double      lat, lon;     // degrees
    double      altitude_km;  // above WGS-84 ellipsoid
};

// ─── Interceptor vehicle model ─────────────────────────────────────────────

struct InterceptorParams {
    std::string         vehicle_name;
    int                 num_stages;
    std::vector<double> stage_thrust_kn;        // Thrust per stage (kN)
    std::vector<double> stage_burn_time_sec;     // Burn time per stage
    std::vector<double> stage_mass_kg;           // Total mass at stage ignition
    std::vector<double> stage_propellant_kg;     // Propellant mass per stage
    double              kill_vehicle_mass_kg;    // Final KV mass
    double              max_lateral_accel_g;     // KV max divert capability
};

// ─── Intercept point geometry ──────────────────────────────────────────────

struct InterceptPoint {
    LatLonAlt position;          // Geographic position of intercept
    double    altitude_km;
    double    range_km;          // Slant range from launch site
    double    time_of_flight_sec;
    double    closing_velocity_kms;  // Relative velocity at intercept
    double    miss_distance_km;      // Predicted miss (0 = direct hit)
    Vec3      interceptor_vel;       // Interceptor velocity at intercept
    Vec3      target_vel;            // Target velocity at intercept
};

// ─── Engagement feasibility ────────────────────────────────────────────────

enum class Feasibility {
    FEASIBLE,
    INSUFFICIENT_ENERGY,     // Delta-V exceeds capability
    GEOMETRY_INFEASIBLE,     // Target doesn't pass near enough
    TIMING_INFEASIBLE,       // Window too short
    ALTITUDE_TOO_LOW,        // Target below minimum engagement alt
    ALTITUDE_TOO_HIGH        // Target above maximum engagement alt
};

// ─── Engagement result ─────────────────────────────────────────────────────

struct EngagementResult {
    int            target_norad_id;
    std::string    target_name;
    std::string    launch_site_id;
    Feasibility    feasibility;
    double         launch_azimuth_deg;
    double         launch_elevation_deg;
    double         total_delta_v_kms;
    double         time_of_flight_sec;
    InterceptPoint intercept;
    Timestamp      launch_time;
    Timestamp      intercept_time;
    double         confidence;
};

// ─── Engagement window ─────────────────────────────────────────────────────

struct EngagementWindow {
    Timestamp        window_start;
    Timestamp        window_end;
    double           best_launch_azimuth_deg;
    double           min_delta_v_kms;
    EngagementResult best_engagement;
};

} // namespace da_asat
