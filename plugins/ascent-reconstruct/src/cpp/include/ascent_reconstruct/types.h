#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>

namespace ascent_reconstruct {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;
constexpr double R_EARTH = 6378137.0;  // meters
constexpr double GM = 3.986004418e14;  // m^3/s^2
constexpr double G0 = 9.80665;         // m/s^2
constexpr double OMEGA_EARTH = 7.2921159e-5; // rad/s

struct Vec3 {
    double x=0, y=0, z=0;
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { double n=norm(); return n>0 ? Vec3{x/n, y/n, z/n} : Vec3{0,0,0}; }
};

struct LatLon {
    double lat = 0.0; // degrees
    double lon = 0.0; // degrees
};

struct LatLonAlt {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0; // meters
};

// A single observation from a radar or optical sensor
struct Observation {
    double time;    // seconds since epoch
    Vec3 position;  // ECEF position in meters (if available)
    double range = 0.0;    // slant range in meters (radar)
    double azimuth = 0.0;  // azimuth in radians (radar/optical)
    double elevation = 0.0; // elevation in radians (radar/optical)
    double range_rate = 0.0; // range rate in m/s (radar doppler)
    bool has_position = false;
    bool has_range = false;
    bool has_angles = false;
    bool has_range_rate = false;
    double weight = 1.0; // observation weight for least squares
};

// A collection of observations forming a tracklet
struct Tracklet {
    std::string tracklet_id;
    std::string sensor_id;
    LatLonAlt sensor_location; // sensor position
    std::vector<Observation> observations;
    int64_t epoch_unix = 0; // Unix time of first observation
};

// State vector at a point in time
struct StateVector {
    double time;    // seconds since epoch
    Vec3 position;  // ECEF meters
    Vec3 velocity;  // ECEF m/s
    double mass = 0.0; // kg (if estimated)
    double thrust = 0.0; // N (if estimated)
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double flight_path_angle_deg = 0.0;
    double heading_deg = 0.0;
    int stage = 1;
};

// Staging event detected from trajectory discontinuity
struct StagingEvent {
    double time;          // seconds since epoch
    int from_stage;
    int to_stage;
    double altitude_m;
    double speed_mps;
    double delta_accel;   // acceleration change at staging (m/s^2)
    Vec3 position;
};

// Ascent model parameters (what we're fitting)
struct AscentModelParams {
    // Launch site
    double launch_lat = 0.0;  // degrees
    double launch_lon = 0.0;  // degrees
    double launch_alt = 0.0;  // meters

    // Launch time
    double launch_time = 0.0; // seconds since tracklet epoch

    // Launch azimuth and elevation
    double launch_azimuth = 0.0; // radians
    double initial_pitch = 85.0 * DEG2RAD; // initial pitch from horizontal

    // Per-stage parameters
    struct StageParams {
        double thrust = 0.0;       // N
        double isp = 0.0;         // s
        double mass_initial = 0.0; // kg
        double mass_final = 0.0;   // kg (after burnout)
        double burn_time = 0.0;    // s
        double pitch_rate = 0.0;   // rad/s (gravity turn rate)
    };
    std::vector<StageParams> stages;
};

// Result of trajectory reconstruction
struct ReconstructionResult {
    bool converged = false;
    int iterations = 0;
    double rms_residual = 0.0; // meters

    // Estimated parameters
    LatLonAlt estimated_launch_site;
    double estimated_launch_time = 0.0;
    double launch_azimuth_deg = 0.0;

    // Reconstructed trajectory
    std::vector<StateVector> trajectory;

    // Staging events
    std::vector<StagingEvent> staging_events;

    // Burnout conditions (final state at end of powered flight)
    StateVector burnout_state;

    // Fitted model parameters
    AscentModelParams fitted_params;
};

} // namespace ascent_reconstruct
