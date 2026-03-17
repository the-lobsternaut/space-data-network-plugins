#pragma once
#include "tle_correlator/types.h"
#include <vector>

namespace tle_correlator {

class SGP4 {
public:
    // Parse a TLE from two-line text
    static TLE parse_tle(const std::string& line0, const std::string& line1, const std::string& line2);

    // Initialize orbital elements from TLE
    static OrbitalElements tle_to_elements(const TLE& tle);

    // Propagate to minutes since epoch, return position and velocity in ECI (km, km/s)
    static void propagate(const OrbitalElements& elems, double tsince_min,
                          Vec3& pos_eci, Vec3& vel_eci);

    // Convert ECI to geodetic (lat, lon, alt)
    static LatLon eci_to_geodetic(const Vec3& pos_eci, double gmst);

    // Compute GMST from Julian date
    static double julian_date(int year, int month, int day, int hour, int min, double sec);
    static double gmst_from_jd(double jd);

    // Generate ground track over a time span
    static std::vector<GroundTrackPoint> ground_track(const TLE& tle,
                                                        int64_t start_time,
                                                        int64_t end_time,
                                                        double step_sec = 60.0);

    // Convert TLE epoch to Unix timestamp
    static int64_t tle_epoch_to_unix(int epoch_year, double epoch_day);
};

} // namespace tle_correlator
