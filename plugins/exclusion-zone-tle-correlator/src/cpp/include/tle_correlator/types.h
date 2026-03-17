#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>

namespace tle_correlator {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;
constexpr double R_EARTH_KM = 6378.137;
constexpr double MU_EARTH = 398600.4418; // km^3/s^2
constexpr double J2 = 1.08263e-3;
constexpr double OMEGA_EARTH = 7.2921159e-5; // rad/s
constexpr double MIN_PER_DAY = 1440.0;
constexpr double SEC_PER_DAY = 86400.0;

struct LatLon { double lat = 0.0; double lon = 0.0; };

struct Vec3 {
    double x=0, y=0, z=0;
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { double n=norm(); return {x/n, y/n, z/n}; }
};

// Polygon exclusion zone
struct ExclusionZone {
    std::string id;
    std::vector<LatLon> vertices; // polygon vertices (closed)
    LatLon center;
    double radius_nm = 0.0; // if circle-type
    bool is_circle = false;
    int64_t effective_start = 0;
    int64_t effective_end = 0;

    // Bounding box
    double min_lat=90, max_lat=-90, min_lon=180, max_lon=-180;
    void compute_bounds() {
        if (is_circle) {
            double dlat = radius_nm / 60.0;
            double dlon = dlat / std::cos(center.lat * DEG2RAD);
            min_lat = center.lat - dlat; max_lat = center.lat + dlat;
            min_lon = center.lon - dlon; max_lon = center.lon + dlon;
        } else {
            for (auto& v : vertices) {
                if (v.lat < min_lat) min_lat = v.lat;
                if (v.lat > max_lat) max_lat = v.lat;
                if (v.lon < min_lon) min_lon = v.lon;
                if (v.lon > max_lon) max_lon = v.lon;
            }
        }
    }
};

// Two-line element set
struct TLE {
    std::string name;
    int norad_id = 0;
    std::string intl_designator;
    int epoch_year = 0; // 2-digit
    double epoch_day = 0.0; // day of year + fractional
    double mean_motion_dot = 0.0; // rev/day^2
    double mean_motion_ddot = 0.0; // rev/day^3
    double bstar = 0.0;
    double inclination_deg = 0.0;
    double raan_deg = 0.0;
    double eccentricity = 0.0; // decimal (already divided)
    double arg_perigee_deg = 0.0;
    double mean_anomaly_deg = 0.0;
    double mean_motion = 0.0; // rev/day
    int rev_number = 0;
};

// SGP4 orbital elements (internal)
struct OrbitalElements {
    double a;      // semi-major axis (Earth radii)
    double e;      // eccentricity
    double i;      // inclination (rad)
    double raan;   // right ascension of ascending node (rad)
    double argp;   // argument of perigee (rad)
    double M;      // mean anomaly (rad)
    double n;      // mean motion (rad/min)
    double bstar;  // drag term
    double epoch_jd; // Julian date of epoch
};

// Ground track point with timestamp
struct GroundTrackPoint {
    LatLon position;
    double altitude_km = 0.0;
    int64_t timestamp = 0;
    double velocity_kmps = 0.0;
    bool ascending = false; // ascending or descending pass
};

// Correlation result
struct Correlation {
    std::string zone_id;
    int norad_id = 0;
    std::string object_name;
    double confidence = 0.0; // [0,1]
    int64_t predicted_crossing_time = 0;
    LatLon crossing_point;
    double min_distance_km = 0.0;
    bool ascending_pass = false;
    double inclination_match = 0.0; // how well zone azimuth matches orbit inclination
};

// Launch time prediction
struct LaunchTimePrediction {
    int64_t launch_time = 0;
    double confidence = 0.0;
    int norad_id = 0;
    std::string rationale;
};

} // namespace tle_correlator
