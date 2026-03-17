#include "da_asat/engagement.h"
#include "da_asat/propagator.h"
#include "da_asat/interceptor.h"
#include <cmath>
#include <algorithm>

namespace da_asat {

static constexpr double PI         = 3.14159265358979323846;
static constexpr double DEG_TO_RAD = PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / PI;
static constexpr double MU_EARTH   = 398600.4418;
static constexpr double G0         = 9.80665;
static constexpr double EARTH_RADIUS_KM = 6378.137;

// ─── Maximum engagement altitude from Tsiolkovsky ──────────────────────────
//
// Sum delta-v across all stages using the rocket equation:
//   dv_stage = Isp * g0 * ln(m0 / mf)
// Convert total delta-v to maximum altitude via energy: h = v^2 / (2g)
// corrected for varying gravity via vis-viva.

double EngagementCalculator::max_engagement_altitude(const InterceptorParams& params) const {
    double total_dv = 0.0; // m/s

    double remaining_mass = 0.0;
    // Start from the last stage and work backwards to get payload for each stage
    remaining_mass = params.kill_vehicle_mass_kg;

    // Compute total delta-v (m/s) for all stages
    for (int s = params.num_stages - 1; s >= 0; --s) {
        double m0 = params.stage_mass_kg[s];
        double mp = params.stage_propellant_kg[s];
        double mf = m0 - mp; // Dry mass of this stage

        // For upper stages, payload mass includes everything above
        // But stage_mass_kg already includes upper stages' mass at ignition
        // So we use: m0 = stage_mass_kg[s], mf = stage_mass_kg[s] - propellant_kg[s]

        // Compute Isp from thrust and mass flow rate
        // mdot = propellant / burn_time
        double burn_time = params.stage_burn_time_sec[s];
        double thrust_n  = params.stage_thrust_kn[s] * 1000.0;
        double mdot      = mp / burn_time;
        double isp       = thrust_n / (mdot * G0); // seconds

        double dv = isp * G0 * std::log(m0 / mf);
        total_dv += dv;
    }

    // Convert total delta-v (m/s) to maximum altitude
    // Using energy equation: v^2/2 = g*h (simplified, sea level g)
    // More accurate: use varying gravity
    // h = R * (1 / (1 - v^2*R/(2*mu)) - 1)  from vis-viva
    double v_kms = total_dv / 1000.0; // km/s
    double R = EARTH_RADIUS_KM;

    // Account for gravity and drag losses (~30% for direct-ascent trajectory)
    double v_effective = v_kms * 0.70; // ~30% gravity/drag losses

    double denominator = 1.0 - v_effective * v_effective * R / (2.0 * MU_EARTH);
    if (denominator <= 0.0) {
        return 1.0e6; // Escape velocity — practically unlimited
    }

    double h_max = R * (1.0 / denominator - 1.0);
    return h_max;
}

// ─── Check feasibility ─────────────────────────────────────────────────────

Feasibility EngagementCalculator::check_feasibility(
    const LaunchSite& site,
    const OrbitalState& target,
    const InterceptorParams& params) const {

    // Compute target altitude
    double r = std::sqrt(target.position.x * target.position.x +
                         target.position.y * target.position.y +
                         target.position.z * target.position.z);
    double alt = r - EARTH_RADIUS_KM;

    // Altitude checks
    if (alt < MIN_ENGAGEMENT_ALT_KM) return Feasibility::ALTITUDE_TOO_LOW;
    if (alt > MAX_ENGAGEMENT_ALT_KM) return Feasibility::ALTITUDE_TOO_HIGH;

    // Check against interceptor capability
    double max_alt = max_engagement_altitude(params);
    if (alt > max_alt) return Feasibility::INSUFFICIENT_ENERGY;

    // Compute slant range from launch site to target
    double gmst = Propagator::compute_gmst(target.epoch);
    double lat_rad = site.lat * DEG_TO_RAD;
    double lon_rad = site.lon * DEG_TO_RAD;
    double R_site = EARTH_RADIUS_KM + site.altitude_km;

    // Site position in ECEF
    double x_ecef = R_site * std::cos(lat_rad) * std::cos(lon_rad);
    double y_ecef = R_site * std::cos(lat_rad) * std::sin(lon_rad);
    double z_ecef = R_site * std::sin(lat_rad);

    // Rotate to ECI
    double cos_g = std::cos(gmst);
    double sin_g = std::sin(gmst);
    double x_eci = cos_g * x_ecef - sin_g * y_ecef;
    double y_eci = sin_g * x_ecef + cos_g * y_ecef;
    double z_eci = z_ecef;

    double dx = target.position.x - x_eci;
    double dy = target.position.y - y_eci;
    double dz = target.position.z - z_eci;
    double slant_range = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Elevation angle from launch site to target
    // In site-centered frame, up direction is radial
    double r_site_mag = std::sqrt(x_eci * x_eci + y_eci * y_eci + z_eci * z_eci);
    double up_x = x_eci / r_site_mag;
    double up_y = y_eci / r_site_mag;
    double up_z = z_eci / r_site_mag;

    double range_dot_up = dx * up_x + dy * up_y + dz * up_z;
    double sin_el = range_dot_up / slant_range;
    double elevation_deg = std::asin(std::max(-1.0, std::min(1.0, sin_el))) * RAD_TO_DEG;

    // Must be above horizon with margin for radar tracking
    if (elevation_deg < 5.0) return Feasibility::GEOMETRY_INFEASIBLE;

    // Estimate required delta-v
    Interceptor interceptor;
    double tof_estimate = slant_range / 3.0; // Rough: ~3 km/s average speed
    tof_estimate = std::max(tof_estimate, 60.0);
    tof_estimate = std::min(tof_estimate, 600.0);

    double dv = interceptor.compute_delta_v(site, target.position, tof_estimate);

    // Compare to available delta-v (rough estimate from max altitude calc)
    double total_dv_available = 0.0;
    for (int s = params.num_stages - 1; s >= 0; --s) {
        double m0 = params.stage_mass_kg[s];
        double mp = params.stage_propellant_kg[s];
        double mf = m0 - mp;
        double burn_time = params.stage_burn_time_sec[s];
        double thrust_n  = params.stage_thrust_kn[s] * 1000.0;
        double mdot      = mp / burn_time;
        double isp       = thrust_n / (mdot * G0);
        total_dv_available += isp * G0 * std::log(m0 / mf);
    }
    total_dv_available /= 1000.0; // m/s → km/s
    total_dv_available *= 0.70;   // Gravity/drag losses

    if (dv > total_dv_available) return Feasibility::INSUFFICIENT_ENERGY;

    return Feasibility::FEASIBLE;
}

// ─── Evaluate a single engagement ──────────────────────────────────────────

EngagementResult EngagementCalculator::evaluate_single(
    const LaunchSite& site,
    const TLE& target,
    const InterceptorParams& params,
    Timestamp launch_time) const {

    EngagementResult result;
    result.target_norad_id = target.norad_id;
    result.target_name     = target.name;
    result.launch_site_id  = site.site_id;
    result.launch_time     = launch_time;
    result.confidence      = 0.0;

    // Estimate time-of-flight: try multiple TOFs and find best
    Interceptor interceptor;
    double best_dv = 1.0e12;
    double best_tof = 0.0;
    OrbitalState best_target_state;
    bool found = false;

    for (double tof = 120.0; tof <= 500.0; tof += 30.0) {
        Timestamp intercept_time = launch_time + static_cast<Timestamp>(tof);
        auto target_state = Propagator::propagate(target, intercept_time);
        if (!target_state.has_value()) continue;

        auto feasibility = check_feasibility(site, *target_state, params);
        if (feasibility != Feasibility::FEASIBLE) continue;

        double dv = interceptor.compute_delta_v(site, target_state->position, tof);
        if (dv < best_dv) {
            best_dv = dv;
            best_tof = tof;
            best_target_state = *target_state;
            found = true;
        }
    }

    if (!found) {
        // Try to find out why it's infeasible
        auto target_state = Propagator::propagate(target, launch_time + 300);
        if (target_state.has_value()) {
            result.feasibility = check_feasibility(site, *target_state, params);
        } else {
            result.feasibility = Feasibility::GEOMETRY_INFEASIBLE;
        }
        result.total_delta_v_kms   = 0.0;
        result.time_of_flight_sec  = 0.0;
        result.launch_azimuth_deg  = 0.0;
        result.launch_elevation_deg = 0.0;
        result.intercept_time      = 0;
        return result;
    }

    // Solve launch parameters
    auto sol = interceptor.solve_launch_params(site, best_target_state, params, best_tof);

    result.feasibility         = Feasibility::FEASIBLE;
    result.launch_azimuth_deg  = sol.azimuth_deg;
    result.launch_elevation_deg = sol.elevation_deg;
    result.total_delta_v_kms   = best_dv;
    result.time_of_flight_sec  = best_tof;
    result.intercept_time      = launch_time + static_cast<Timestamp>(best_tof);

    // Fill intercept point
    auto geo = Propagator::state_to_geodetic(best_target_state);
    result.intercept.position = geo;
    result.intercept.altitude_km = geo.alt_km;

    // Slant range
    double gmst = Propagator::compute_gmst(launch_time);
    double lat_rad = site.lat * DEG_TO_RAD;
    double lon_rad = site.lon * DEG_TO_RAD;
    double R_site = EARTH_RADIUS_KM + site.altitude_km;
    double x_ecef = R_site * std::cos(lat_rad) * std::cos(lon_rad);
    double y_ecef = R_site * std::cos(lat_rad) * std::sin(lon_rad);
    double z_ecef = R_site * std::sin(lat_rad);
    double cos_g = std::cos(gmst), sin_g = std::sin(gmst);
    double x_eci = cos_g * x_ecef - sin_g * y_ecef;
    double y_eci = sin_g * x_ecef + cos_g * y_ecef;
    double z_eci = z_ecef;

    double dxr = best_target_state.position.x - x_eci;
    double dyr = best_target_state.position.y - y_eci;
    double dzr = best_target_state.position.z - z_eci;
    result.intercept.range_km = std::sqrt(dxr * dxr + dyr * dyr + dzr * dzr);

    result.intercept.time_of_flight_sec = best_tof;
    result.intercept.target_vel = best_target_state.velocity;

    // Closing velocity estimate: target speed + intercept approach speed
    double v_tgt = std::sqrt(best_target_state.velocity.x * best_target_state.velocity.x +
                              best_target_state.velocity.y * best_target_state.velocity.y +
                              best_target_state.velocity.z * best_target_state.velocity.z);
    result.intercept.closing_velocity_kms = v_tgt + best_dv * 0.6; // Rough estimate
    result.intercept.miss_distance_km     = 0.0; // Assuming ideal engagement

    // Confidence based on delta-v margin
    double max_dv = max_engagement_altitude(params); // Reuse as rough dv proxy
    result.confidence = std::max(0.0, 1.0 - best_dv / 8.0); // Higher dv = lower confidence

    return result;
}

// ─── Find engagement windows ───────────────────────────────────────────────

std::vector<EngagementWindow> EngagementCalculator::find_engagement_windows(
    const LaunchSite& site,
    const TLE& target,
    const InterceptorParams& params,
    Timestamp window_start,
    Timestamp window_end) const {

    std::vector<EngagementWindow> windows;
    double step = 60.0; // 1-minute steps

    bool in_window = false;
    EngagementWindow current_window;
    double min_dv_in_window = 1.0e12;
    EngagementResult best_in_window;

    for (Timestamp t = window_start; t <= window_end; t += static_cast<Timestamp>(step)) {
        auto result = evaluate_single(site, target, params, t);

        if (result.feasibility == Feasibility::FEASIBLE) {
            if (!in_window) {
                // Start new window
                in_window = true;
                current_window.window_start = t;
                min_dv_in_window = result.total_delta_v_kms;
                best_in_window = result;
            }

            if (result.total_delta_v_kms < min_dv_in_window) {
                min_dv_in_window = result.total_delta_v_kms;
                best_in_window = result;
            }

            current_window.window_end = t;
        } else {
            if (in_window) {
                // Close window
                current_window.best_launch_azimuth_deg = best_in_window.launch_azimuth_deg;
                current_window.min_delta_v_kms         = min_dv_in_window;
                current_window.best_engagement         = best_in_window;
                windows.push_back(current_window);
                in_window = false;
                min_dv_in_window = 1.0e12;
            }
        }
    }

    // Close final window if still open
    if (in_window) {
        current_window.best_launch_azimuth_deg = best_in_window.launch_azimuth_deg;
        current_window.min_delta_v_kms         = min_dv_in_window;
        current_window.best_engagement         = best_in_window;
        windows.push_back(current_window);
    }

    return windows;
}

// ─── Evaluate engagements for multiple targets ─────────────────────────────

std::vector<EngagementResult> EngagementCalculator::evaluate_engagements(
    const LaunchSite& site,
    const std::vector<TLE>& targets,
    const InterceptorParams& params,
    Timestamp window_start,
    Timestamp window_end,
    double time_step_sec) const {

    std::vector<EngagementResult> all_results;

    for (const auto& target : targets) {
        auto windows = find_engagement_windows(site, target, params,
                                                window_start, window_end);
        for (const auto& w : windows) {
            all_results.push_back(w.best_engagement);
        }
    }

    // Sort by delta-v (most efficient first)
    std::sort(all_results.begin(), all_results.end(),
        [](const EngagementResult& a, const EngagementResult& b) {
            return a.total_delta_v_kms < b.total_delta_v_kms;
        });

    return all_results;
}

// ─── Compute engagement zone boundary ──────────────────────────────────────
//
// For each azimuth (0-360), compute the maximum range at the given
// target altitude and convert to lat/lon from the launch site.

std::vector<LatLon> EngagementCalculator::compute_engagement_zone_boundary(
    const LaunchSite& site,
    const InterceptorParams& params,
    double target_altitude_km) const {

    std::vector<LatLon> boundary;
    Interceptor interceptor;

    // Compute maximum slant range for this altitude
    double max_alt = max_engagement_altitude(params);
    if (target_altitude_km > max_alt) return boundary; // Empty — can't reach

    // Estimate max range: range = sqrt((R+h)^2 - R^2) for horizon distance
    // but limited by delta-v
    double R = EARTH_RADIUS_KM;
    double R_h = R + target_altitude_km;

    // Geometric horizon distance (max possible)
    double horizon_range = std::sqrt(R_h * R_h - R * R);

    // Delta-v limited range: use energy to compute max TOF, then max range
    // Rough: max range ~ max_dv * typical_tof / 2
    double total_dv = 0.0;
    for (int s = params.num_stages - 1; s >= 0; --s) {
        double m0 = params.stage_mass_kg[s];
        double mp = params.stage_propellant_kg[s];
        double mf = m0 - mp;
        double burn_time = params.stage_burn_time_sec[s];
        double thrust_n  = params.stage_thrust_kn[s] * 1000.0;
        double mdot      = mp / burn_time;
        double isp       = thrust_n / (mdot * G0);
        total_dv += isp * G0 * std::log(m0 / mf);
    }
    total_dv /= 1000.0; // m/s → km/s
    total_dv *= 0.70;   // Losses

    // Max range is roughly the horizontal component of trajectory
    // For a near-vertical ascent, horizontal range comes from geometry
    // Approximate: range_km = sqrt(slant^2 - alt^2) where slant is limited
    double max_slant = std::min(horizon_range, total_dv * 120.0); // 120s nominal TOF factor
    double max_ground_range = std::sqrt(std::max(max_slant * max_slant -
                              target_altitude_km * target_altitude_km, 0.0));

    // Cap at geometric horizon
    max_ground_range = std::min(max_ground_range, horizon_range);

    // Generate boundary polygon (great circle points from site)
    for (int az_deg = 0; az_deg < 360; az_deg += 5) {
        double az_rad = az_deg * DEG_TO_RAD;
        double angular_dist = max_ground_range / R; // radians

        double lat1 = site.lat * DEG_TO_RAD;
        double lon1 = site.lon * DEG_TO_RAD;

        // Great circle destination formula
        double lat2 = std::asin(std::sin(lat1) * std::cos(angular_dist) +
                                std::cos(lat1) * std::sin(angular_dist) * std::cos(az_rad));
        double lon2 = lon1 + std::atan2(
            std::sin(az_rad) * std::sin(angular_dist) * std::cos(lat1),
            std::cos(angular_dist) - std::sin(lat1) * std::sin(lat2));

        LatLon pt;
        pt.lat = lat2 * RAD_TO_DEG;
        pt.lon = lon2 * RAD_TO_DEG;
        boundary.push_back(pt);
    }

    return boundary;
}

} // namespace da_asat
