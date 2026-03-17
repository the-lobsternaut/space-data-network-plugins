#include "da_pipeline/sgp4.h"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// parse_tle
// ---------------------------------------------------------------------------
TLE SGP4::parse_tle(const std::string& line0,
                    const std::string& line1,
                    const std::string& line2) {
    TLE tle;

    // Line 0: name (trim whitespace)
    tle.name = line0;
    while (!tle.name.empty() && (tle.name.back() == '\r' || tle.name.back() == ' '))
        tle.name.pop_back();
    while (!tle.name.empty() && tle.name.front() == ' ')
        tle.name.erase(tle.name.begin());

    // Line 1: 1 NNNNNC NNNNNAAA NNNNN.NNNNNNNN +.NNNNNNNN +NNNNN-N +NNNNN-N N NNNNN
    if (line1.size() < 69) throw std::runtime_error("TLE line1 too short");

    tle.norad_id = std::stoi(line1.substr(2, 5));
    tle.intl_designator = line1.substr(9, 8);
    // Trim
    while (!tle.intl_designator.empty() && tle.intl_designator.back() == ' ')
        tle.intl_designator.pop_back();

    tle.epoch_year = std::stoi(line1.substr(18, 2));
    tle.epoch_day = std::stod(line1.substr(20, 12));

    // Mean motion dot (rev/day^2 / 2)
    tle.mean_motion_dot = std::stod(line1.substr(33, 10));

    // Mean motion double dot — special format: +NNNNN-N
    {
        std::string s = line1.substr(44, 8);
        // Format: " NNNNN-N" → 0.NNNNN * 10^-N
        // Replace leading space with sign
        while (!s.empty() && s[0] == ' ') s.erase(s.begin());
        if (s.size() >= 2) {
            // Find the exponent part (last two chars: sign + digit)
            char sign = '+';
            if (s[0] == '-' || s[0] == '+') {
                sign = s[0];
                s = s.substr(1);
            }
            // s is now "NNNNN-N" or "NNNNN+N"
            size_t exp_pos = s.find_last_of("+-");
            if (exp_pos != std::string::npos && exp_pos > 0) {
                double mantissa = std::stod("0." + s.substr(0, exp_pos));
                int exp_val = std::stoi(s.substr(exp_pos));
                tle.mean_motion_ddot = mantissa * std::pow(10.0, exp_val);
                if (sign == '-') tle.mean_motion_ddot = -tle.mean_motion_ddot;
            }
        }
    }

    // BSTAR — same special format
    {
        std::string s = line1.substr(53, 8);
        while (!s.empty() && s[0] == ' ') s.erase(s.begin());
        if (s.size() >= 2) {
            char sign = '+';
            if (s[0] == '-' || s[0] == '+') {
                sign = s[0];
                s = s.substr(1);
            }
            size_t exp_pos = s.find_last_of("+-");
            if (exp_pos != std::string::npos && exp_pos > 0) {
                double mantissa = std::stod("0." + s.substr(0, exp_pos));
                int exp_val = std::stoi(s.substr(exp_pos));
                tle.bstar = mantissa * std::pow(10.0, exp_val);
                if (sign == '-') tle.bstar = -tle.bstar;
            }
        }
    }

    // Line 2: 2 NNNNN NNN.NNNN NNN.NNNN NNNNNNN NNN.NNNN NNN.NNNN NN.NNNNNNNNNNNNNN
    if (line2.size() < 69) throw std::runtime_error("TLE line2 too short");

    tle.inclination_deg = std::stod(line2.substr(8, 8));
    tle.raan_deg = std::stod(line2.substr(17, 8));

    // Eccentricity: assumed decimal point "NNNNNNN" → 0.NNNNNNN
    tle.eccentricity = std::stod("0." + line2.substr(26, 7));

    tle.arg_perigee_deg = std::stod(line2.substr(34, 8));
    tle.mean_anomaly_deg = std::stod(line2.substr(43, 8));
    tle.mean_motion = std::stod(line2.substr(52, 11));

    if (line2.size() >= 68) {
        std::string rev_str = line2.substr(63, 5);
        while (!rev_str.empty() && rev_str[0] == ' ') rev_str.erase(rev_str.begin());
        if (!rev_str.empty()) {
            try { tle.rev_number = std::stoi(rev_str); } catch (...) {}
        }
    }

    return tle;
}

// ---------------------------------------------------------------------------
// parse_catalog
// ---------------------------------------------------------------------------
std::vector<TLE> SGP4::parse_catalog(const std::string& text) {
    std::vector<TLE> catalog;
    std::istringstream iss(text);
    std::string line;
    std::vector<std::string> lines;

    while (std::getline(iss, line)) {
        // Strip carriage return
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) lines.push_back(line);
    }

    // Process in groups of 3
    for (size_t i = 0; i + 2 < lines.size(); ) {
        // Check if line starts with "1 " (line1) or is a name line
        if (lines[i].size() >= 2 && lines[i][0] == '1' && lines[i][1] == ' ') {
            // Two-line format without name
            try {
                TLE tle = parse_tle("", lines[i], lines[i + 1]);
                catalog.push_back(tle);
                i += 2;
            } catch (...) { i++; }
        } else if (i + 2 < lines.size() &&
                   lines[i + 1].size() >= 2 && lines[i + 1][0] == '1' && lines[i + 1][1] == ' ') {
            // Three-line format with name
            try {
                TLE tle = parse_tle(lines[i], lines[i + 1], lines[i + 2]);
                catalog.push_back(tle);
                i += 3;
            } catch (...) { i++; }
        } else {
            i++;
        }
    }

    return catalog;
}

// ---------------------------------------------------------------------------
// tle_to_elements
// ---------------------------------------------------------------------------
OrbitalElements SGP4::tle_to_elements(const TLE& tle) {
    OrbitalElements e;
    e.i = tle.inclination_deg * DEG2RAD;
    e.raan = tle.raan_deg * DEG2RAD;
    e.e = tle.eccentricity;
    e.argp = tle.arg_perigee_deg * DEG2RAD;
    e.M = tle.mean_anomaly_deg * DEG2RAD;
    e.bstar = tle.bstar;

    // Mean motion in rad/min
    e.n = tle.mean_motion * 2.0 * PI / MIN_PER_DAY;

    // Semi-major axis from mean motion
    // n = sqrt(mu/a^3) → a = (mu/n^2)^(1/3)
    // But n is in rad/min, mu in km^3/s^2
    // Convert n to rad/s: n_s = n / 60
    double n_s = e.n / 60.0;
    e.a = std::pow(MU_EARTH / (n_s * n_s), 1.0 / 3.0);

    // Epoch JD
    int year = tle.epoch_year;
    if (year < 57) year += 2000; else year += 1900;
    e.epoch_jd = julian_date(year, 1, 1, 0, 0, 0.0) + tle.epoch_day - 1.0;

    return e;
}

// ---------------------------------------------------------------------------
// solve_kepler
// ---------------------------------------------------------------------------
double SGP4::solve_kepler(double M, double e, double tol, int max_iter) {
    // Newton-Raphson: E_{n+1} = E_n - (E_n - e*sin(E_n) - M) / (1 - e*cos(E_n))
    double E = M;
    if (e > 0.8) E = PI; // Better initial guess for high eccentricity

    for (int i = 0; i < max_iter; i++) {
        double f = E - e * std::sin(E) - M;
        double fp = 1.0 - e * std::cos(E);
        double dE = f / fp;
        E -= dE;
        if (std::abs(dE) < tol) break;
    }
    return E;
}

// ---------------------------------------------------------------------------
// propagate — simplified SGP4 with J2 secular perturbations
// ---------------------------------------------------------------------------
void SGP4::propagate(const OrbitalElements& elems, double tsince_min,
                     Vec3& pos, Vec3& vel) {
    double a0 = elems.a;
    double e0 = elems.e;
    double i0 = elems.i;
    double raan0 = elems.raan;
    double argp0 = elems.argp;
    double M0 = elems.M;
    double n0 = elems.n;  // rad/min
    double bstar = elems.bstar;

    // Recover original mean motion and semi-major axis (SGP4 un-Kozai)
    double cosio = std::cos(i0);
    double sinio = std::sin(i0);
    double theta2 = cosio * cosio;

    // J2 perturbation constants
    double k2 = 0.5 * J2 * R_EARTH_KM * R_EARTH_KM; // Actually 1/2 * J2 * R_E^2

    double a1 = a0;
    double p = a1 * (1.0 - e0 * e0);
    double xi = 1.0 / (a1 - 1.0); // Not used in simplified version

    // Secular rates (J2 only)
    double n_dot_secular = 0;
    if (p > 0.1) {
        double p2 = p * p;
        // RAAN rate: d(RAAN)/dt = -3/2 * n * J2 * (R_E/p)^2 * cos(i)
        double raan_dot = -1.5 * n0 * J2 * (R_EARTH_KM * R_EARTH_KM / (p * p)) * cosio;

        // Argument of perigee rate
        double argp_dot = 0.75 * n0 * J2 * (R_EARTH_KM * R_EARTH_KM / (p * p)) *
                          (5.0 * theta2 - 1.0);

        // Mean anomaly rate (include J2 correction)
        double M_dot = n0 + 0.75 * n0 * J2 * (R_EARTH_KM * R_EARTH_KM / (p * p)) *
                       std::sqrt(1.0 - e0 * e0) * (3.0 * theta2 - 1.0);

        // Drag: simple decay
        double n_drag = n0;
        double e_drag = e0;
        if (bstar != 0) {
            // Simplified drag: semi-major axis decay
            double delta_a = -2.0 / 3.0 * bstar * a0 * tsince_min / MIN_PER_DAY;
            a1 = a0 + delta_a;
            if (a1 < R_EARTH_KM + 100) a1 = R_EARTH_KM + 100; // Clamp
            n_drag = std::sqrt(MU_EARTH / (a1 * a1 * a1)) * 60.0; // rad/min
            e_drag = e0 - 2.0 / 3.0 * bstar * e0 * tsince_min / MIN_PER_DAY;
            if (e_drag < 1e-6) e_drag = 1e-6;
            if (e_drag > 0.999) e_drag = 0.999;
        }

        // Updated elements at tsince
        double raan = raan0 + raan_dot * tsince_min;
        double argp = argp0 + argp_dot * tsince_min;
        double M = M0 + M_dot * tsince_min;

        // Normalize M to [0, 2*PI)
        M = std::fmod(M, 2.0 * PI);
        if (M < 0) M += 2.0 * PI;

        // Solve Kepler's equation
        double E = solve_kepler(M, e_drag);

        // True anomaly
        double sinE = std::sin(E);
        double cosE = std::cos(E);
        double sinv = std::sqrt(1.0 - e_drag * e_drag) * sinE / (1.0 - e_drag * cosE);
        double cosv = (cosE - e_drag) / (1.0 - e_drag * cosE);
        double nu = std::atan2(sinv, cosv);

        // Distance
        double r = a1 * (1.0 - e_drag * cosE);

        // Argument of latitude
        double u = argp + nu;
        double sin_u = std::sin(u);
        double cos_u = std::cos(u);

        // Position in orbital plane
        double xp = r * cos_u;
        double yp = r * sin_u;

        // Velocity in orbital plane
        double sqrt_mu_p = std::sqrt(MU_EARTH / (a1 * (1.0 - e_drag * e_drag)));
        double vxp = -sqrt_mu_p * std::sin(nu);
        double vyp = sqrt_mu_p * (e_drag + std::cos(nu));

        // Rotate velocity to argument of latitude frame
        double cos_argp = std::cos(argp);
        double sin_argp = std::sin(argp);
        double vx_u = vxp * cos_argp - vyp * sin_argp;  // Not quite right for the rotation
        double vy_u = vxp * sin_argp + vyp * cos_argp;

        // Actually, let's use the standard perifocal → ECI rotation:
        double sin_raan = std::sin(raan);
        double cos_raan = std::cos(raan);

        // Position in ECI (TEME)
        pos.x = xp * (cos_raan * std::cos(argp) - sin_raan * std::sin(argp) * cosio) -
                yp * (cos_raan * std::sin(argp) + sin_raan * std::cos(argp) * cosio);
        // Wait — let me use proper perifocal vectors
        // Actually we already have u = argp + nu, so:
        // r_vec = r * [cos_u * cos_raan - sin_u * sin_raan * cos_i,
        //              cos_u * sin_raan + sin_u * cos_raan * cos_i,
        //              sin_u * sin_i]
        pos.x = r * (cos_u * cos_raan - sin_u * sin_raan * cosio);
        pos.y = r * (cos_u * sin_raan + sin_u * cos_raan * cosio);
        pos.z = r * (sin_u * sinio);

        // Velocity in ECI
        // v = r_dot * r_hat + r * nu_dot * s_hat
        // where r_dot = sqrt(mu/p) * e * sin(nu)
        // and r * nu_dot = sqrt(mu*p) / r = sqrt(mu/p) * (1 + e*cos(nu))
        double rdot = std::sqrt(MU_EARTH / (a1 * (1.0 - e_drag * e_drag))) * e_drag * std::sin(nu);
        double rfdot = std::sqrt(MU_EARTH / (a1 * (1.0 - e_drag * e_drag))) * (1.0 + e_drag * std::cos(nu));

        // r_hat and s_hat in ECI
        double ux = cos_u * cos_raan - sin_u * sin_raan * cosio;
        double uy = cos_u * sin_raan + sin_u * cos_raan * cosio;
        double uz = sin_u * sinio;

        double sx = -sin_u * cos_raan - cos_u * sin_raan * cosio;
        double sy = -sin_u * sin_raan + cos_u * cos_raan * cosio;
        double sz = cos_u * sinio;

        vel.x = rdot * ux + rfdot * sx;
        vel.y = rdot * uy + rfdot * sy;
        vel.z = rdot * uz + rfdot * sz;
    } else {
        // Degenerate case
        pos = {a0, 0, 0};
        vel = {0, std::sqrt(MU_EARTH / a0), 0};
    }
}

// ---------------------------------------------------------------------------
// eci_to_geodetic
// ---------------------------------------------------------------------------
LatLon SGP4::eci_to_geodetic(const Vec3& pos, double gmst) {
    LatLon ll;

    // Longitude = atan2(y, x) - GMST
    double theta = std::atan2(pos.y, pos.x);
    double lon = theta - gmst;

    // Normalize to [-PI, PI]
    while (lon < -PI) lon += 2.0 * PI;
    while (lon > PI) lon -= 2.0 * PI;

    // Latitude — iterative for oblate Earth
    double r_xy = std::sqrt(pos.x * pos.x + pos.y * pos.y);
    double lat = std::atan2(pos.z, r_xy); // Initial guess (spherical)

    // Iterate with WGS-84 flattening
    double f = 1.0 / 298.257223563;
    double e2 = 2 * f - f * f;
    for (int i = 0; i < 10; i++) {
        double sin_lat = std::sin(lat);
        double N = R_EARTH_KM / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
        lat = std::atan2(pos.z + N * e2 * sin_lat, r_xy);
    }

    ll.lat = lat * RAD2DEG;
    ll.lon = lon * RAD2DEG;
    return ll;
}

// ---------------------------------------------------------------------------
// julian_date
// ---------------------------------------------------------------------------
double SGP4::julian_date(int y, int m, int d, int h, int min, double sec) {
    // Standard Julian date formula
    if (m <= 2) { y--; m += 12; }
    int A = y / 100;
    int B = 2 - A + A / 4;
    double JD = std::floor(365.25 * (y + 4716)) +
                std::floor(30.6001 * (m + 1)) +
                d + B - 1524.5;
    JD += (h + min / 60.0 + sec / 3600.0) / 24.0;
    return JD;
}

// ---------------------------------------------------------------------------
// gmst_from_jd
// ---------------------------------------------------------------------------
double SGP4::gmst_from_jd(double jd) {
    // GMST at 0h UT
    double T = (jd - 2451545.0) / 36525.0;
    double gmst = 280.46061837 +
                  360.98564736629 * (jd - 2451545.0) +
                  T * T * (0.000387933 - T / 38710000.0);
    gmst = std::fmod(gmst, 360.0);
    if (gmst < 0) gmst += 360.0;
    return gmst * DEG2RAD;
}

// ---------------------------------------------------------------------------
// tle_epoch_to_unix
// ---------------------------------------------------------------------------
Timestamp SGP4::tle_epoch_to_unix(int epoch_year, double epoch_day) {
    int year = epoch_year;
    if (year < 57) year += 2000; else year += 1900;

    // Days from 1970-01-01 to year-01-01
    auto is_leap = [](int y) { return (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)); };
    int64_t days = 0;
    for (int y = 1970; y < year; y++) days += is_leap(y) ? 366 : 365;

    // epoch_day is 1-based day of year + fractional
    double total_days = days + epoch_day - 1.0;
    return static_cast<Timestamp>(total_days * SEC_PER_DAY);
}

// ---------------------------------------------------------------------------
// ground_track
// ---------------------------------------------------------------------------
std::vector<GroundTrackPoint> SGP4::ground_track(
    const TLE& tle, Timestamp start, Timestamp end, double step_sec) {
    std::vector<GroundTrackPoint> track;

    OrbitalElements elems = tle_to_elements(tle);
    Timestamp epoch_unix = tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);

    for (Timestamp t = start; t <= end; t += static_cast<Timestamp>(step_sec)) {
        double tsince_min = static_cast<double>(t - epoch_unix) / 60.0;

        Vec3 pos, vel;
        propagate(elems, tsince_min, pos, vel);

        // JD at this time
        double jd = 2440587.5 + static_cast<double>(t) / SEC_PER_DAY;
        double gmst = gmst_from_jd(jd);

        LatLon ll = eci_to_geodetic(pos, gmst);
        double alt = pos.norm() - R_EARTH_KM;

        GroundTrackPoint gtp;
        gtp.position = ll;
        gtp.altitude_km = alt;
        gtp.timestamp = t;
        gtp.velocity_kmps = vel.norm();
        gtp.ascending = vel.z > 0;

        track.push_back(gtp);
    }

    return track;
}

} // namespace da_pipeline
