#include "da_asat/propagator.h"
#include <cmath>
#include <cstring>
#include <sstream>

namespace da_asat {

// ─── TLE epoch to Unix timestamp ───────────────────────────────────────────

Timestamp Propagator::tle_epoch_to_unix(double year, double day) {
    // TLE epoch year is 2-digit; 57-99 -> 1957-1999, 00-56 -> 2000-2056
    int full_year = (year < 57.0) ? 2000 + static_cast<int>(year)
                                   : 1900 + static_cast<int>(year);

    // Days since Unix epoch (1970-01-01)
    // Compute Julian Day Number for Jan 1 of the epoch year
    int y = full_year;
    int m = 1;
    int d = 1;
    int jd = d - 32075 + 1461 * (y + 4800 + (m - 14) / 12) / 4
             + 367 * (m - 2 - (m - 14) / 12 * 12) / 12
             - 3 * ((y + 4900 + (m - 14) / 12) / 100) / 4;

    // Julian Day of Unix epoch (1970-01-01 = JD 2440588)
    static constexpr int JD_UNIX_EPOCH = 2440588;
    int64_t days_since_epoch = static_cast<int64_t>(jd) - JD_UNIX_EPOCH;

    // Add fractional day (TLE day is 1-based, so subtract 1)
    double total_days = static_cast<double>(days_since_epoch) + (day - 1.0);
    return static_cast<Timestamp>(total_days * 86400.0);
}

// ─── Solve Kepler's equation (Newton-Raphson) ──────────────────────────────

double Propagator::solve_kepler(double M, double e, int max_iter) {
    // E - e*sin(E) = M
    double E = M; // Initial guess
    for (int i = 0; i < max_iter; ++i) {
        double dE = (E - e * std::sin(E) - M) / (1.0 - e * std::cos(E));
        E -= dE;
        if (std::abs(dE) < 1.0e-12) break;
    }
    return E;
}

// ─── Compute GMST from Unix timestamp ──────────────────────────────────────

double Propagator::compute_gmst(Timestamp time) {
    // Julian centuries from J2000.0 (2000-01-01 12:00:00 TT)
    // J2000.0 = Unix 946728000 (approx, ignoring TT-UTC offset)
    static constexpr int64_t J2000_UNIX = 946728000;
    double T = static_cast<double>(time - J2000_UNIX) / (86400.0 * 36525.0);

    // GMST in seconds at 0h UT1 (IAU 1982 model)
    double gmst_sec = 67310.54841
                    + (876600.0 * 3600.0 + 8640184.812866) * T
                    + 0.093104 * T * T
                    - 6.2e-6 * T * T * T;

    // Convert to radians and normalize to [0, 2*PI)
    double gmst = std::fmod(gmst_sec * TWO_PI / 86400.0, TWO_PI);
    if (gmst < 0.0) gmst += TWO_PI;
    return gmst;
}

// ─── Orbital period from mean motion ───────────────────────────────────────

double Propagator::orbital_period_sec(const TLE& tle) {
    // mean_motion is in rev/day
    if (tle.mean_motion <= 0.0) return 0.0;
    return 86400.0 / tle.mean_motion;
}

// ─── Parse TLE ─────────────────────────────────────────────────────────────

std::optional<TLE> Propagator::parse_tle(const std::string& name,
                                          const std::string& line1,
                                          const std::string& line2) {
    // Validate line lengths
    if (line1.size() < 69 || line2.size() < 69) return std::nullopt;
    if (line1[0] != '1' || line2[0] != '2') return std::nullopt;

    TLE tle;
    tle.name  = name;
    tle.line1 = line1;
    tle.line2 = line2;

    // Line 1 fields
    try {
        tle.norad_id   = std::stoi(line1.substr(2, 5));
        tle.epoch_year = std::stod(line1.substr(18, 2));
        tle.epoch_day  = std::stod(line1.substr(20, 12));

        // Bstar drag term (columns 54-61): format is " NNNNN-N" meaning 0.NNNNN * 10^-N
        std::string bstar_str = line1.substr(53, 8);
        // Remove spaces
        size_t start = bstar_str.find_first_not_of(' ');
        if (start != std::string::npos) {
            bstar_str = bstar_str.substr(start);
        }
        // Parse assumed-decimal format: e.g., "30000-4" -> 0.30000e-4
        if (!bstar_str.empty()) {
            // Find the exponent sign (last '-' or '+' that isn't at position 0)
            size_t exp_pos = bstar_str.find_last_of("+-");
            if (exp_pos != std::string::npos && exp_pos > 0) {
                std::string mantissa = bstar_str.substr(0, exp_pos);
                std::string exponent = bstar_str.substr(exp_pos);
                double mant = std::stod("0." + mantissa);
                int exp_val = std::stoi(exponent);
                tle.bstar = mant * std::pow(10.0, exp_val);
            } else {
                tle.bstar = 0.0;
            }
        } else {
            tle.bstar = 0.0;
        }
    } catch (...) {
        return std::nullopt;
    }

    // Line 2 fields
    try {
        tle.inclination   = std::stod(line2.substr(8, 8));
        tle.raan          = std::stod(line2.substr(17, 8));

        // Eccentricity: assumed leading decimal point
        std::string ecc_str = line2.substr(26, 7);
        tle.eccentricity = std::stod("0." + ecc_str);

        tle.arg_perigee   = std::stod(line2.substr(34, 8));
        tle.mean_anomaly  = std::stod(line2.substr(43, 8));
        tle.mean_motion   = std::stod(line2.substr(52, 11));
    } catch (...) {
        return std::nullopt;
    }

    return tle;
}

// ─── Propagate TLE to a given time ─────────────────────────────────────────
//
// Simplified SGP4: applies J2 secular perturbations on RAAN, argument of
// perigee, and mean anomaly. Includes Bstar drag on the semi-major axis.

std::optional<OrbitalState> Propagator::propagate(const TLE& tle, Timestamp time) {
    // Compute semi-major axis from mean motion
    double n0 = tle.mean_motion * TWO_PI / 86400.0; // rad/s
    if (n0 <= 0.0) return std::nullopt;

    double a0 = std::pow(MU_EARTH / (n0 * n0), 1.0 / 3.0); // km

    // Time since TLE epoch
    Timestamp epoch_unix = tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);
    double dt = static_cast<double>(time - epoch_unix); // seconds

    double e0   = tle.eccentricity;
    double i0   = tle.inclination * DEG_TO_RAD;
    double raan0 = tle.raan * DEG_TO_RAD;
    double w0   = tle.arg_perigee * DEG_TO_RAD;
    double M0   = tle.mean_anomaly * DEG_TO_RAD;

    // J2 secular perturbation rates
    double p0    = a0 * (1.0 - e0 * e0);
    double cos_i = std::cos(i0);
    double sin_i = std::sin(i0);

    double n_dot_factor = 1.5 * J2 * (EARTH_RADIUS_KM / p0) * (EARTH_RADIUS_KM / p0);

    // RAAN rate (rad/s)
    double raan_dot = -n0 * n_dot_factor * cos_i;

    // Argument of perigee rate (rad/s)
    double w_dot = n0 * n_dot_factor * (2.0 - 2.5 * sin_i * sin_i);

    // Mean motion with J2 correction
    double n_corrected = n0 * (1.0 + n_dot_factor * std::sqrt(1.0 - e0 * e0) *
                         (1.0 - 1.5 * sin_i * sin_i));

    // Bstar drag: semi-major axis decay
    double a = a0 - 2.0 / 3.0 * a0 * tle.bstar * n0 * dt;
    if (a < EARTH_RADIUS_KM) a = EARTH_RADIUS_KM + 1.0; // Clamp

    double e = e0; // Eccentricity change is small for short propagation

    // Updated orbital elements at time t
    double raan = raan0 + raan_dot * dt;
    double w    = w0    + w_dot * dt;
    double M    = M0    + n_corrected * dt;

    // Normalize mean anomaly
    M = std::fmod(M, TWO_PI);
    if (M < 0.0) M += TWO_PI;

    // Solve Kepler's equation
    double E = solve_kepler(M, e);

    // True anomaly
    double sin_nu = std::sqrt(1.0 - e * e) * std::sin(E) / (1.0 - e * std::cos(E));
    double cos_nu = (std::cos(E) - e) / (1.0 - e * std::cos(E));
    double nu = std::atan2(sin_nu, cos_nu);

    // Radius
    double r = a * (1.0 - e * std::cos(E));

    // Position in orbital plane (perifocal frame)
    double x_pf = r * cos_nu;
    double y_pf = r * sin_nu;

    // Velocity in orbital plane
    double h = std::sqrt(MU_EARTH * a * (1.0 - e * e)); // specific angular momentum
    double vx_pf = -MU_EARTH / h * std::sin(nu);
    double vy_pf =  MU_EARTH / h * (e + std::cos(nu));

    // Rotation from perifocal to TEME (ECI)
    double cos_w = std::cos(w), sin_w = std::sin(w);
    double cos_raan = std::cos(raan), sin_raan = std::sin(raan);
    double cos_i_val = std::cos(i0), sin_i_val = std::sin(i0);

    // Rotation matrix elements (perifocal -> ECI)
    double R11 = cos_raan * cos_w - sin_raan * sin_w * cos_i_val;
    double R12 = -cos_raan * sin_w - sin_raan * cos_w * cos_i_val;
    double R21 = sin_raan * cos_w + cos_raan * sin_w * cos_i_val;
    double R22 = -sin_raan * sin_w + cos_raan * cos_w * cos_i_val;
    double R31 = sin_w * sin_i_val;
    double R32 = cos_w * sin_i_val;

    OrbitalState state;
    state.position.x = R11 * x_pf + R12 * y_pf;
    state.position.y = R21 * x_pf + R22 * y_pf;
    state.position.z = R31 * x_pf + R32 * y_pf;

    state.velocity.x = R11 * vx_pf + R12 * vy_pf;
    state.velocity.y = R21 * vx_pf + R22 * vy_pf;
    state.velocity.z = R31 * vx_pf + R32 * vy_pf;

    state.epoch = time;
    return state;
}

// ─── State to geodetic coordinates ─────────────────────────────────────────

LatLonAlt Propagator::state_to_geodetic(const OrbitalState& state) {
    double gmst = compute_gmst(state.epoch);

    // Rotate from TEME to ECEF
    double cos_g = std::cos(gmst);
    double sin_g = std::sin(gmst);

    double x_ecef = cos_g * state.position.x + sin_g * state.position.y;
    double y_ecef = -sin_g * state.position.x + cos_g * state.position.y;
    double z_ecef = state.position.z;

    // Geodetic coordinates (simplified, no WGS-84 iteration)
    double r_xy  = std::sqrt(x_ecef * x_ecef + y_ecef * y_ecef);
    double lon   = std::atan2(y_ecef, x_ecef) * RAD_TO_DEG;
    double lat   = std::atan2(z_ecef, r_xy) * RAD_TO_DEG;
    double alt   = std::sqrt(x_ecef * x_ecef + y_ecef * y_ecef + z_ecef * z_ecef) - EARTH_RADIUS_KM;

    return {lat, lon, alt};
}

} // namespace da_asat
