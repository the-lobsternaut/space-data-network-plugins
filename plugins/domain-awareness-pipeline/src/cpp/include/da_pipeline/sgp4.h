#pragma once
#include "da_pipeline/types.h"
#include <string>
#include <vector>

namespace da_pipeline {

class SGP4 {
public:
    /// Parse a single TLE (3 lines: name, line1, line2)
    static TLE parse_tle(const std::string& line0, const std::string& line1, const std::string& line2);

    /// Parse a multi-TLE catalog (3-line format blocks)
    static std::vector<TLE> parse_catalog(const std::string& text);

    /// Convert TLE to orbital elements
    static OrbitalElements tle_to_elements(const TLE& tle);

    /// SGP4 propagation — tsince in minutes from epoch
    /// pos in km, vel in km/s (TEME frame)
    static void propagate(const OrbitalElements& elems, double tsince_min, Vec3& pos, Vec3& vel);

    /// Convert ECI position to geodetic lat/lon (degrees)
    static LatLon eci_to_geodetic(const Vec3& pos, double gmst);

    /// Julian date from calendar
    static double julian_date(int y, int m, int d, int h, int min, double sec);

    /// GMST angle (radians) from Julian date
    static double gmst_from_jd(double jd);

    /// Generate ground track points over a time window
    static std::vector<GroundTrackPoint> ground_track(
        const TLE& tle, Timestamp start, Timestamp end, double step_sec = 60.0);

    /// Convert TLE epoch (2-digit year + day-of-year) to Unix timestamp
    static Timestamp tle_epoch_to_unix(int epoch_year, double epoch_day);

private:
    /// Solve Kepler's equation M = E - e*sin(E) for E
    static double solve_kepler(double M, double e, double tol = 1e-12, int max_iter = 50);
};

} // namespace da_pipeline
