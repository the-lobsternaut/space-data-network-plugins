#include "tle_correlator/sgp4.h"
#include <cmath>
#include <cstring>
#include <sstream>

namespace tle_correlator {

// ---- Helpers ----------------------------------------------------------------

static double parse_double(const std::string& s, int start, int len) {
    if (start + len > static_cast<int>(s.size())) return 0.0;
    std::string sub = s.substr(start, len);
    size_t first = sub.find_first_not_of(' ');
    if (first == std::string::npos) return 0.0;
    size_t last = sub.find_last_not_of(' ');
    sub = sub.substr(first, last - first + 1);
    try { return std::stod(sub); } catch (...) { return 0.0; }
}

static int parse_int(const std::string& s, int start, int len) {
    if (start + len > static_cast<int>(s.size())) return 0;
    std::string sub = s.substr(start, len);
    size_t first = sub.find_first_not_of(' ');
    if (first == std::string::npos) return 0;
    size_t last = sub.find_last_not_of(' ');
    sub = sub.substr(first, last - first + 1);
    try { return std::stoi(sub); } catch (...) { return 0; }
}

// Parse TLE exponential notation: " 12345-6" -> 0.12345e-6
static double parse_tle_exponential(const std::string& s, int start, int len) {
    if (start + len > static_cast<int>(s.size())) return 0.0;
    std::string sub = s.substr(start, len);
    size_t first = sub.find_first_not_of(' ');
    if (first == std::string::npos) return 0.0;
    size_t last = sub.find_last_not_of(' ');
    sub = sub.substr(first, last - first + 1);
    if (sub.empty() || sub == "00000-0" || sub == " 00000-0") return 0.0;

    // Find the last sign character (the exponent sign)
    int exp_sign_pos = -1;
    for (int i = static_cast<int>(sub.size()) - 1; i >= 1; --i) {
        if (sub[i] == '+' || sub[i] == '-') { exp_sign_pos = i; break; }
    }
    if (exp_sign_pos < 0) return 0.0;

    std::string mantissa_str = sub.substr(0, exp_sign_pos);
    std::string exp_str = sub.substr(exp_sign_pos);

    double sign = 1.0;
    if (!mantissa_str.empty() && mantissa_str[0] == '-') {
        sign = -1.0; mantissa_str = mantissa_str.substr(1);
    } else if (!mantissa_str.empty() && mantissa_str[0] == '+') {
        mantissa_str = mantissa_str.substr(1);
    }

    double mantissa = 0.0;
    try { mantissa = std::stod("0." + mantissa_str); } catch (...) { return 0.0; }
    int exponent = 0;
    try { exponent = std::stoi(exp_str); } catch (...) { return 0.0; }

    return sign * mantissa * std::pow(10.0, exponent);
}

// ---- Kepler equation solver (Newton-Raphson) --------------------------------

static double solve_kepler(double M, double e, int max_iter = 30) {
    double E = M;
    if (e > 0.8) E = PI;
    for (int i = 0; i < max_iter; ++i) {
        double dE = (E - e * std::sin(E) - M) / (1.0 - e * std::cos(E));
        E -= dE;
        if (std::abs(dE) < 1.0e-12) break;
    }
    return E;
}

// ---- parse_tle --------------------------------------------------------------

TLE SGP4::parse_tle(const std::string& line0, const std::string& line1, const std::string& line2) {
    TLE tle;
    // Line 0: object name (trim trailing whitespace)
    tle.name = line0;
    while (!tle.name.empty() && (tle.name.back() == ' ' || tle.name.back() == '\r' || tle.name.back() == '\n'))
        tle.name.pop_back();

    // Line 1: col 2-6 catalog number, col 9-16 intl desig, col 18-19 epoch year,
    //          col 20-31 epoch day, col 33-42 mean_motion_dot, col 44-51 mean_motion_ddot,
    //          col 53-60 bstar
    tle.norad_id = parse_int(line1, 2, 5);

    if (line1.size() > 16) {
        std::string intl = line1.substr(9, 8);
        size_t f = intl.find_first_not_of(' ');
        size_t l = intl.find_last_not_of(' ');
        if (f != std::string::npos) tle.intl_designator = intl.substr(f, l - f + 1);
    }

    tle.epoch_year = parse_int(line1, 18, 2);
    tle.epoch_day = parse_double(line1, 20, 12);
    tle.mean_motion_dot = parse_double(line1, 33, 10);
    tle.mean_motion_ddot = parse_tle_exponential(line1, 44, 8);
    tle.bstar = parse_tle_exponential(line1, 53, 8);

    // Line 2: col 8-15 inclination, col 17-24 RAAN, col 26-32 eccentricity (implied decimal),
    //          col 34-41 arg perigee, col 43-50 mean anomaly, col 52-62 mean motion,
    //          col 63-67 rev number
    tle.inclination_deg = parse_double(line2, 8, 8);
    tle.raan_deg = parse_double(line2, 17, 8);

    // Eccentricity: implied leading "0."
    if (line2.size() > 33) {
        std::string ecc_str = line2.substr(26, 7);
        size_t ef = ecc_str.find_first_not_of(' ');
        if (ef != std::string::npos) ecc_str = ecc_str.substr(ef);
        try { tle.eccentricity = std::stod("0." + ecc_str); } catch (...) { tle.eccentricity = 0.0; }
    }

    tle.arg_perigee_deg = parse_double(line2, 34, 8);
    tle.mean_anomaly_deg = parse_double(line2, 43, 8);
    tle.mean_motion = parse_double(line2, 52, 11);
    tle.rev_number = parse_int(line2, 63, 5);

    return tle;
}

// ---- tle_to_elements --------------------------------------------------------

OrbitalElements SGP4::tle_to_elements(const TLE& tle) {
    OrbitalElements el;
    el.i = tle.inclination_deg * DEG2RAD;
    el.raan = tle.raan_deg * DEG2RAD;
    el.e = tle.eccentricity;
    el.argp = tle.arg_perigee_deg * DEG2RAD;
    el.M = tle.mean_anomaly_deg * DEG2RAD;
    el.bstar = tle.bstar;

    // Mean motion: rev/day -> rad/min
    el.n = tle.mean_motion * 2.0 * PI / MIN_PER_DAY;

    // Semi-major axis from mean motion: n = sqrt(mu/a^3) (in consistent units)
    // mu in km^3/min^2
    double mu_min = MU_EARTH * 3600.0; // km^3/s^2 -> km^3/min^2
    double a_km = std::pow(mu_min / (el.n * el.n), 1.0 / 3.0);
    el.a = a_km / R_EARTH_KM; // in Earth radii

    // Compute epoch Julian date
    int full_year = (tle.epoch_year >= 57) ? (1900 + tle.epoch_year) : (2000 + tle.epoch_year);
    // Jan 1 of full_year
    double jd_jan1 = julian_date(full_year, 1, 1, 0, 0, 0.0);
    el.epoch_jd = jd_jan1 + (tle.epoch_day - 1.0);

    return el;
}

// ---- propagate --------------------------------------------------------------

void SGP4::propagate(const OrbitalElements& elems, double tsince_min,
                     Vec3& pos_eci, Vec3& vel_eci) {
    double incl = elems.i;
    double raan0 = elems.raan;
    double argp0 = elems.argp;
    double M0 = elems.M;
    double ecc = elems.e;
    double n0 = elems.n;
    double bstar = elems.bstar;

    // Semi-major axis in km
    double mu_min = MU_EARTH * 3600.0;
    double a0_km = std::pow(mu_min / (n0 * n0), 1.0 / 3.0);

    // Semi-latus rectum
    double p0 = a0_km * (1.0 - ecc * ecc);

    // J2 perturbation rates
    double cos_i = std::cos(incl);
    double sin_i = std::sin(incl);
    double cos2_i = cos_i * cos_i;
    double ae = R_EARTH_KM;
    double k2 = 0.5 * J2 * ae * ae;
    double p0_2 = p0 * p0;

    // RAAN rate: dOmega/dt = -1.5 * n * J2 * (Re/p)^2 * cos(i)
    double raan_dot = -1.5 * n0 * (2.0 * k2) / p0_2 * cos_i;

    // Argument of perigee rate
    double argp_dot = 0.75 * n0 * (2.0 * k2) / p0_2 * (5.0 * cos2_i - 1.0);

    // Mean motion with J2 correction
    double M_dot = n0 + 0.75 * n0 * (2.0 * k2) / p0_2 *
                   std::sqrt(1.0 - ecc * ecc) * (3.0 * cos2_i - 1.0);

    // Simplified drag effect on semi-major axis
    double drag_decay = 1.0 - 2.0 * bstar * ae * n0 * tsince_min / a0_km;
    if (drag_decay < 0.1) drag_decay = 0.1;
    double a_t = a0_km * std::pow(drag_decay, 1.0 / 3.0);

    // Updated eccentricity (simplified drag)
    double ecc_t = ecc - bstar * ae * tsince_min * ecc / a0_km;
    if (ecc_t < 1.0e-6) ecc_t = 1.0e-6;
    if (ecc_t > 0.999) ecc_t = 0.999;

    // Updated angles
    double raan_t = raan0 + raan_dot * tsince_min;
    double argp_t = argp0 + argp_dot * tsince_min;
    double M_t = M0 + M_dot * tsince_min;

    // Normalize mean anomaly
    M_t = std::fmod(M_t, 2.0 * PI);
    if (M_t < 0) M_t += 2.0 * PI;

    // Solve Kepler's equation
    double E = solve_kepler(M_t, ecc_t);

    // True anomaly
    double sin_E = std::sin(E);
    double cos_E = std::cos(E);
    double sqrt_1me2 = std::sqrt(1.0 - ecc_t * ecc_t);
    double sin_v = sqrt_1me2 * sin_E / (1.0 - ecc_t * cos_E);
    double cos_v = (cos_E - ecc_t) / (1.0 - ecc_t * cos_E);
    double v = std::atan2(sin_v, cos_v);

    // Argument of latitude
    double u = v + argp_t;

    // Radius
    double r = a_t * (1.0 - ecc_t * cos_E);

    // Position in orbital plane
    double x_orb = r * std::cos(u);
    double y_orb = r * std::sin(u);

    // Velocity in orbital plane
    double n_t = std::sqrt(mu_min / (a_t * a_t * a_t));
    double p_t = a_t * (1.0 - ecc_t * ecc_t);
    double r_dot = a_t * ecc_t * sin_E * n_t / (1.0 - ecc_t * cos_E);
    double r_v_dot = std::sqrt(mu_min * p_t) / r;

    double vx_orb = r_dot * std::cos(u) - r_v_dot * std::sin(u);
    double vy_orb = r_dot * std::sin(u) + r_v_dot * std::cos(u);

    // Rotate to ECI through RAAN and inclination
    double cos_O = std::cos(raan_t);
    double sin_O = std::sin(raan_t);

    pos_eci.x = x_orb * cos_O - y_orb * cos_i * sin_O;
    pos_eci.y = x_orb * sin_O + y_orb * cos_i * cos_O;
    pos_eci.z = y_orb * sin_i;

    vel_eci.x = (vx_orb * cos_O - vy_orb * cos_i * sin_O) / 60.0; // km/min -> km/s
    vel_eci.y = (vx_orb * sin_O + vy_orb * cos_i * cos_O) / 60.0;
    vel_eci.z = (vy_orb * sin_i) / 60.0;
}

// ---- eci_to_geodetic --------------------------------------------------------

LatLon SGP4::eci_to_geodetic(const Vec3& pos_eci, double gmst) {
    double lon = std::atan2(pos_eci.y, pos_eci.x) - gmst;
    // Normalize longitude to [-PI, PI]
    while (lon > PI)  lon -= 2.0 * PI;
    while (lon < -PI) lon += 2.0 * PI;

    double r_xy = std::sqrt(pos_eci.x * pos_eci.x + pos_eci.y * pos_eci.y);
    double lat = std::atan2(pos_eci.z, r_xy);

    return {lat * RAD2DEG, lon * RAD2DEG};
}

// ---- julian_date ------------------------------------------------------------

double SGP4::julian_date(int year, int month, int day, int hour, int min, double sec) {
    // Meeus algorithm
    if (month <= 2) { year -= 1; month += 12; }
    int A = year / 100;
    int B = 2 - A + A / 4;
    double JD = std::floor(365.25 * (year + 4716)) +
                std::floor(30.6001 * (month + 1)) +
                day + B - 1524.5;
    JD += (hour + min / 60.0 + sec / 3600.0) / 24.0;
    return JD;
}

// ---- gmst_from_jd -----------------------------------------------------------

double SGP4::gmst_from_jd(double jd) {
    double T = (jd - 2451545.0) / 36525.0;
    // IAU 1982 model
    double gmst_sec = 67310.54841
                    + (876600.0 * 3600.0 + 8640184.812866) * T
                    + 0.093104 * T * T
                    - 6.2e-6 * T * T * T;
    double gmst_rad = std::fmod(gmst_sec * 2.0 * PI / 86400.0, 2.0 * PI);
    if (gmst_rad < 0) gmst_rad += 2.0 * PI;
    return gmst_rad;
}

// ---- tle_epoch_to_unix ------------------------------------------------------

int64_t SGP4::tle_epoch_to_unix(int epoch_year, double epoch_day) {
    int full_year = (epoch_year >= 57) ? (1900 + epoch_year) : (2000 + epoch_year);

    // Days from Unix epoch (1970-01-01) to Jan 1 of full_year
    int64_t days = 0;
    if (full_year >= 1970) {
        for (int y = 1970; y < full_year; ++y) {
            bool leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
            days += leap ? 366 : 365;
        }
    } else {
        for (int y = full_year; y < 1970; ++y) {
            bool leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
            days -= leap ? 366 : 365;
        }
    }
    // epoch_day is 1-based (Jan 1 = day 1.0)
    double total_days = static_cast<double>(days) + (epoch_day - 1.0);
    return static_cast<int64_t>(total_days * SEC_PER_DAY);
}

// ---- ground_track -----------------------------------------------------------

std::vector<GroundTrackPoint> SGP4::ground_track(const TLE& tle,
                                                   int64_t start_time,
                                                   int64_t end_time,
                                                   double step_sec) {
    std::vector<GroundTrackPoint> track;

    OrbitalElements elems = tle_to_elements(tle);
    int64_t epoch_unix = tle_epoch_to_unix(tle.epoch_year, tle.epoch_day);

    LatLon prev_pos = {0, 0};
    bool have_prev = false;

    for (int64_t t = start_time; t <= end_time; t += static_cast<int64_t>(step_sec)) {
        double tsince_min = static_cast<double>(t - epoch_unix) / 60.0;

        // Limit propagation to ~30 days
        if (std::abs(tsince_min) > 43200.0) continue;

        Vec3 pos, vel;
        propagate(elems, tsince_min, pos, vel);

        double r = pos.norm();
        if (r < R_EARTH_KM || r > 50000.0) continue;

        // Compute GMST
        double jd = static_cast<double>(t) / SEC_PER_DAY + 2440587.5;
        double gmst = gmst_from_jd(jd);

        LatLon geo = eci_to_geodetic(pos, gmst);
        double alt = r - R_EARTH_KM;

        GroundTrackPoint gtp;
        gtp.position = geo;
        gtp.altitude_km = alt;
        gtp.timestamp = t;
        gtp.velocity_kmps = vel.norm();

        // Determine if ascending (latitude increasing)
        if (have_prev) {
            gtp.ascending = (geo.lat > prev_pos.lat);
        }

        track.push_back(gtp);
        prev_pos = geo;
        have_prev = true;
    }

    return track;
}

} // namespace tle_correlator
