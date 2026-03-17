/**
 * @file types.cpp
 * @brief Implementations for member functions declared in types.h
 */
#include "../include/types.h"
#include "../include/constants.h"
#include <cmath>

namespace drama {

// ── KeplerianOrbit ──────────────────────────────────────────────────────────

double KeplerianOrbit::perigee_altitude_km() const {
    double rp = semi_major_axis_m * (1.0 - eccentricity);
    return (rp - R_EARTH) / 1000.0;
}

double KeplerianOrbit::apogee_altitude_km() const {
    double ra = semi_major_axis_m * (1.0 + eccentricity);
    return (ra - R_EARTH) / 1000.0;
}

double KeplerianOrbit::mean_altitude_km() const {
    return (semi_major_axis_m - R_EARTH) / 1000.0;
}

double KeplerianOrbit::period_seconds() const {
    return TWO_PI * std::sqrt(semi_major_axis_m * semi_major_axis_m * semi_major_axis_m / MU_EARTH);
}

// ── EpochTime ───────────────────────────────────────────────────────────────

double EpochTime::to_jd() const {
    // Meeus algorithm for Julian Date
    int y = year;
    int m = month;
    if (m <= 2) {
        y -= 1;
        m += 12;
    }
    int A = y / 100;
    int B = 2 - A + (A / 4);
    double JD = std::floor(365.25 * (y + 4716))
              + std::floor(30.6001 * (m + 1))
              + day + B - 1524.5;
    JD += (hour + minute / 60.0 + second / 3600.0) / 24.0;
    return JD;
}

double EpochTime::to_mjd() const {
    return to_jd() - 2400000.5;
}

EpochTime EpochTime::from_mjd(double mjd) {
    double jd = mjd + 2400000.5;
    // Inverse Julian Date (Meeus)
    double z_d = std::floor(jd + 0.5);
    double f = (jd + 0.5) - z_d;
    int64_t z = static_cast<int64_t>(z_d);
    int64_t a;
    if (z < 2299161) {
        a = z;
    } else {
        int64_t alpha = static_cast<int64_t>((z_d - 1867216.25) / 36524.25);
        a = z + 1 + alpha - alpha / 4;
    }
    int64_t b = a + 1524;
    int64_t c = static_cast<int64_t>((b - 122.1) / 365.25);
    int64_t d = static_cast<int64_t>(365.25 * c);
    int64_t e = static_cast<int64_t>((b - d) / 30.6001);

    EpochTime et;
    et.day = static_cast<int>(b - d - static_cast<int64_t>(30.6001 * e));
    et.month = (e < 14) ? static_cast<int>(e - 1) : static_cast<int>(e - 13);
    et.year = (et.month > 2) ? static_cast<int>(c - 4716) : static_cast<int>(c - 4715);

    double day_frac = f * 24.0;
    et.hour = static_cast<int>(day_frac);
    day_frac = (day_frac - et.hour) * 60.0;
    et.minute = static_cast<int>(day_frac);
    et.second = (day_frac - et.minute) * 60.0;
    return et;
}

double EpochTime::fractional_year() const {
    // Days in year
    bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    int days_in_year = leap ? 366 : 365;
    // Day of year
    static const int mdays[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    int doy = day;
    for (int i = 1; i < month; i++) {
        doy += mdays[i];
        if (i == 2 && leap) doy += 1;
    }
    double frac = (doy - 1 + (hour + minute / 60.0 + second / 3600.0) / 24.0) / days_in_year;
    return year + frac;
}

// ── MaterialProperties ──────────────────────────────────────────────────────

MaterialProperties MaterialProperties::get(MaterialType type) {
    // Reference: NASA SP-8060, ESA DRAMA SARA User Manual
    switch (type) {
        case MaterialType::ALUMINUM:
            return {MaterialType::ALUMINUM, 2700.0, 900.0, 933.0, 3.97e5, 0.09, "Aluminum"};
        case MaterialType::STEEL:
            return {MaterialType::STEEL, 7800.0, 500.0, 1783.0, 2.47e5, 0.35, "Steel"};
        case MaterialType::TITANIUM:
            return {MaterialType::TITANIUM, 4500.0, 523.0, 1941.0, 4.19e5, 0.30, "Titanium"};
        case MaterialType::CFRP:
            return {MaterialType::CFRP, 1600.0, 1000.0, 573.0, 0.0, 0.85, "CFRP"};
        case MaterialType::COPPER:
            return {MaterialType::COPPER, 8900.0, 385.0, 1357.0, 2.05e5, 0.05, "Copper"};
        case MaterialType::GLASS:
            return {MaterialType::GLASS, 2500.0, 840.0, 973.0, 0.0, 0.92, "Glass"};
        case MaterialType::INCONEL:
            return {MaterialType::INCONEL, 8440.0, 444.0, 1573.0, 2.97e5, 0.30, "Inconel"};
        case MaterialType::BERYLLIUM:
            return {MaterialType::BERYLLIUM, 1850.0, 1825.0, 1560.0, 1.10e6, 0.18, "Beryllium"};
        case MaterialType::STAINLESS_STEEL:
            return {MaterialType::STAINLESS_STEEL, 8000.0, 500.0, 1700.0, 2.60e5, 0.35, "Stainless Steel"};
        case MaterialType::GENERIC:
        default:
            return {MaterialType::GENERIC, 3000.0, 700.0, 1200.0, 2.0e5, 0.50, "Generic"};
    }
}

} // namespace drama
