#pragma once
// Domain Awareness Pipeline — Header-only type definitions
// Combines key types from launch-predict, ntm-scraper, tle-correlator,
// ascent-reconstruct, and da-asat-predictor into namespace da_pipeline.

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;
constexpr double R_EARTH_KM = 6378.137;
constexpr double R_EARTH_M = 6378137.0;
constexpr double MU_EARTH = 398600.4418;        // km^3/s^2
constexpr double J2 = 1.08263e-3;
constexpr double OMEGA_EARTH = 7.2921159e-5;     // rad/s
constexpr double G0 = 9.80665;                    // m/s^2
constexpr double NM_TO_KM = 1.852;
constexpr double KM_TO_NM = 1.0 / 1.852;
constexpr double MIN_PER_DAY = 1440.0;
constexpr double SEC_PER_DAY = 86400.0;

// ---------------------------------------------------------------------------
// From launch-predict
// ---------------------------------------------------------------------------
using Timestamp = int64_t;  // Unix seconds

struct LatLon {
    double lat = 0;
    double lon = 0;
};

struct Polygon {
    std::vector<LatLon> vertices;
};

enum class NotamClassification { LAUNCH, REENTRY, HAZARD, AIRSPACE, ROUTINE };

struct NOTAM {
    std::string notam_id;
    std::string location_id;
    Timestamp issued = 0;
    Timestamp effective_start = 0;
    Timestamp effective_end = 0;
    double altitude_floor_ft = 0;
    double altitude_ceiling_ft = 0;
    double center_lat = 0;
    double center_lon = 0;
    double radius_nm = 0;
    Polygon polygon;
    std::string notam_text;
    NotamClassification classification = NotamClassification::ROUTINE;
};

struct LaunchSite {
    std::string site_id;
    std::string name;
    double lat = 0;
    double lon = 0;
    std::vector<double> typical_azimuths;
};

struct LaunchPad {
    std::string pad_id;
    std::string site_id;
    std::string name;
    double lat = 0;
    double lon = 0;
};

// ---------------------------------------------------------------------------
// From ntm-scraper
// ---------------------------------------------------------------------------
enum class GeometryType { CIRCLE, POLYGON, UNKNOWN };

enum class HazardClassification {
    ROCKET_DEBRIS, SPLASHDOWN, BOOSTER_RECOVERY,
    FAIRING_RECOVERY, GENERAL_HAZARD, UNKNOWN
};

enum class BroadcastArea { HYDROLANT, HYDROPAC, NAVAREA_IV, NAVAREA_XII, UNKNOWN };

struct Circle {
    LatLon center;
    double radius_nm = 0;
};

struct ExclusionZone {
    std::string id;
    GeometryType type = GeometryType::UNKNOWN;
    Circle circle;
    Polygon polygon;
    double min_lat = 90;
    double max_lat = -90;
    double min_lon = 180;
    double max_lon = -180;
    Timestamp effective_start = 0;
    Timestamp effective_end = 0;

    void compute_bounds() {
        if (type == GeometryType::CIRCLE) {
            double dlat = (circle.radius_nm * NM_TO_KM) / 111.32;
            double dlon = dlat / std::cos(circle.center.lat * DEG2RAD);
            min_lat = circle.center.lat - dlat;
            max_lat = circle.center.lat + dlat;
            min_lon = circle.center.lon - dlon;
            max_lon = circle.center.lon + dlon;
        } else if (type == GeometryType::POLYGON) {
            min_lat = 90; max_lat = -90; min_lon = 180; max_lon = -180;
            for (auto& v : polygon.vertices) {
                if (v.lat < min_lat) min_lat = v.lat;
                if (v.lat > max_lat) max_lat = v.lat;
                if (v.lon < min_lon) min_lon = v.lon;
                if (v.lon > max_lon) max_lon = v.lon;
            }
        }
    }
};

struct MaritimeNotice {
    std::string notice_id;
    std::string raw_text;
    std::string authority;
    std::string cancel_notice_id;
    BroadcastArea area = BroadcastArea::UNKNOWN;
    Timestamp effective_start = 0;
    Timestamp effective_end = 0;
    HazardClassification classification = HazardClassification::UNKNOWN;
    ExclusionZone zone;
    bool is_cancelled = false;
    double confidence = 0;
};

// ---------------------------------------------------------------------------
// From tle-correlator
// ---------------------------------------------------------------------------
struct Vec3 {
    double x = 0, y = 0, z = 0;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        double n = norm();
        if (n < 1e-15) return {0, 0, 0};
        return {x / n, y / n, z / n};
    }
};

struct TLE {
    std::string name;
    int norad_id = 0;
    std::string intl_designator;
    int epoch_year = 0;
    double epoch_day = 0;
    double mean_motion_dot = 0;
    double mean_motion_ddot = 0;
    double bstar = 0;
    double inclination_deg = 0;
    double raan_deg = 0;
    double eccentricity = 0;
    double arg_perigee_deg = 0;
    double mean_anomaly_deg = 0;
    double mean_motion = 0;      // revs/day
    int rev_number = 0;
};

struct OrbitalElements {
    double a = 0;       // semi-major axis (km)
    double e = 0;       // eccentricity
    double i = 0;       // inclination (rad)
    double raan = 0;    // RAAN (rad)
    double argp = 0;    // argument of perigee (rad)
    double M = 0;       // mean anomaly (rad)
    double n = 0;       // mean motion (rad/min)
    double bstar = 0;
    double epoch_jd = 0;
};

struct GroundTrackPoint {
    LatLon position;
    double altitude_km = 0;
    Timestamp timestamp = 0;
    double velocity_kmps = 0;
    bool ascending = false;
};

struct Correlation {
    std::string zone_id;
    int norad_id = 0;
    std::string object_name;
    double confidence = 0;
    Timestamp predicted_crossing_time = 0;
    LatLon crossing_point;
    double min_distance_km = 0;
    bool ascending_pass = false;
    double inclination_match = 0;
};

// ---------------------------------------------------------------------------
// From ascent-reconstruct
// ---------------------------------------------------------------------------
struct LatLonAlt {
    double lat = 0;
    double lon = 0;
    double alt = 0;
};

struct Observation {
    double time = 0;
    Vec3 position;
    double range = 0;
    double azimuth = 0;
    double elevation = 0;
    double range_rate = 0;
    bool has_position = false;
    bool has_range = false;
    bool has_angles = false;
    bool has_range_rate = false;
    double weight = 1.0;
};

struct Tracklet {
    std::string tracklet_id;
    std::string sensor_id;
    LatLonAlt sensor_location;
    std::vector<Observation> observations;
    int64_t epoch_unix = 0;
};

struct StateVector {
    double time = 0;
    Vec3 position;
    Vec3 velocity;
    double mass = 0;
    double thrust = 0;
    double altitude_m = 0;
    double speed_mps = 0;
    double flight_path_angle_deg = 0;
    double heading_deg = 0;
    int stage = 0;
};

struct StagingEvent {
    double time = 0;
    int from_stage = 0;
    int to_stage = 0;
    double altitude_m = 0;
    double speed_mps = 0;
    double delta_accel = 0;
    Vec3 position;
};

struct AscentModelParams {
    double launch_lat = 0;
    double launch_lon = 0;
    double launch_alt = 0;
    double launch_time = 0;
    double launch_azimuth = 0;
    double initial_pitch = 0;

    struct StageParams {
        double thrust = 0;       // N
        double isp = 0;          // s
        double mass_initial = 0; // kg
        double mass_final = 0;   // kg
        double burn_time = 0;    // s
        double pitch_rate = 0;   // deg/s
    };
    std::vector<StageParams> stages;
};

struct ReconstructionResult {
    bool converged = false;
    int iterations = 0;
    double rms_residual = 0;
    LatLonAlt estimated_launch_site;
    double estimated_launch_time = 0;
    double launch_azimuth_deg = 0;
    std::vector<StateVector> trajectory;
    std::vector<StagingEvent> staging_events;
    StateVector burnout_state;
    AscentModelParams fitted_params;
};

// ---------------------------------------------------------------------------
// From da-asat-predictor
// ---------------------------------------------------------------------------
enum class Feasibility {
    FEASIBLE, INSUFFICIENT_ENERGY, GEOMETRY_INFEASIBLE,
    TIMING_INFEASIBLE, ALTITUDE_TOO_LOW, ALTITUDE_TOO_HIGH
};

struct InterceptorParams {
    std::string vehicle_name;
    int num_stages = 0;
    std::vector<double> stage_thrust_kn;
    std::vector<double> stage_burn_time_sec;
    std::vector<double> stage_mass_kg;
    std::vector<double> stage_propellant_kg;
    double kill_vehicle_mass_kg = 0;
    double max_lateral_accel_g = 0;
};

struct InterceptPoint {
    LatLonAlt position;
    double altitude_km = 0;
    double range_km = 0;
    double time_of_flight_sec = 0;
    double closing_velocity_kms = 0;
    double miss_distance_km = 0;
    Vec3 interceptor_vel;
    Vec3 target_vel;
};

struct EngagementResult {
    int target_norad_id = 0;
    std::string target_name;
    std::string launch_site_id;
    Feasibility feasibility = Feasibility::GEOMETRY_INFEASIBLE;
    double launch_azimuth_deg = 0;
    double launch_elevation_deg = 0;
    double total_delta_v_kms = 0;
    double time_of_flight_sec = 0;
    InterceptPoint intercept;
    Timestamp launch_time = 0;
    Timestamp intercept_time = 0;
    double confidence = 0;
};

struct EngagementWindow {
    Timestamp window_start = 0;
    Timestamp window_end = 0;
    double best_launch_azimuth_deg = 0;
    double min_delta_v_kms = 0;
    EngagementResult best_engagement;
};

// ---------------------------------------------------------------------------
// Pipeline-specific types
// ---------------------------------------------------------------------------
enum class ThreatLevel { LOW, MODERATE, HIGH, CRITICAL };

enum class ActivityType {
    LAUNCH_PREPARATION, LAUNCH_DETECTED, REENTRY,
    ASAT_CONCERN, ROUTINE_ACTIVITY, UNKNOWN
};

struct ExclusionZoneGroup {
    std::string group_id;
    std::vector<ExclusionZone> notam_zones;
    std::vector<ExclusionZone> ntm_zones;
    LaunchSite associated_site;
    double azimuth_deg = 0;
    ActivityType activity = ActivityType::UNKNOWN;
};

struct AscentTrajectoryFamily {
    std::string family_id;
    double min_azimuth_deg = 0;
    double max_azimuth_deg = 0;
    double estimated_inclination_deg = 0;
    std::vector<StateVector> nominal_trajectory;
    std::vector<std::vector<StateVector>> dispersions;
};

struct ThreatAssessment {
    std::string site_id;
    ThreatLevel level = ThreatLevel::LOW;
    std::string rationale;
    std::vector<EngagementResult> feasible_engagements;
    std::vector<int> threatened_norad_ids;
};

struct SituationalAwareness {
    Timestamp generated_at = 0;
    Timestamp analysis_window_start = 0;
    Timestamp analysis_window_end = 0;

    // Parsed inputs
    std::vector<NOTAM> parsed_notams;
    std::vector<MaritimeNotice> parsed_ntms;
    std::vector<TLE> tle_catalog;

    // Exclusion zone analysis
    std::vector<ExclusionZoneGroup> zone_groups;

    // TLE correlations
    std::vector<Correlation> correlations;

    // Trajectory analysis
    std::vector<AscentTrajectoryFamily> trajectory_families;
    std::vector<ReconstructionResult> reconstructions;

    // ASAT threat assessment
    std::vector<ThreatAssessment> threat_assessments;

    // Summary
    int total_active_zones = 0;
    int launch_sites_active = 0;
    int tle_correlations_found = 0;
    int asat_threats_identified = 0;
    ThreatLevel overall_threat = ThreatLevel::LOW;
    std::string summary_text;
};

} // namespace da_pipeline
