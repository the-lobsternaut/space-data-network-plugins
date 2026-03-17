#pragma once

#include "da_asat/types.h"
#include <optional>
#include <string>

namespace da_asat {

// ─── Orbit Propagator ──────────────────────────────────────────────────────
//
// Simplified SGP4-style propagator for target satellite state prediction.
// Applies J2 secular perturbations and Bstar drag on the semi-major axis.

class Propagator {
public:
    static std::optional<TLE> parse_tle(const std::string& name,
                                         const std::string& line1,
                                         const std::string& line2);

    static std::optional<OrbitalState> propagate(const TLE& tle, Timestamp time);

    static LatLonAlt state_to_geodetic(const OrbitalState& state);

    static double compute_gmst(Timestamp time);

    static double orbital_period_sec(const TLE& tle);

private:
    static constexpr double PI             = 3.14159265358979323846;
    static constexpr double TWO_PI         = 2.0 * PI;
    static constexpr double DEG_TO_RAD     = PI / 180.0;
    static constexpr double RAD_TO_DEG     = 180.0 / PI;
    static constexpr double MU_EARTH       = 398600.4418;       // km^3/s^2
    static constexpr double EARTH_RADIUS_KM = 6378.137;
    static constexpr double J2             = 1.08263e-3;
    static constexpr double OMEGA_EARTH    = 7.2921159e-5;      // rad/s

    static Timestamp tle_epoch_to_unix(double year, double day);
    static double solve_kepler(double M, double e, int max_iter = 30);
};

} // namespace da_asat
