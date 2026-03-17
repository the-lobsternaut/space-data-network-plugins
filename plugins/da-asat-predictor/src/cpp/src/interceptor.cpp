#include "da_asat/interceptor.h"
#include "da_asat/propagator.h"
#include <cmath>
#include <algorithm>

namespace da_asat {

// ─── Default interceptor parameters ────────────────────────────────────────
//
// Generic 3-stage solid rocket DA-ASAT based on publicly available
// parameters for systems like the SC-19 / SM-3 class.

InterceptorParams Interceptor::default_interceptor() {
    InterceptorParams params;
    params.vehicle_name        = "Generic DA-ASAT";
    params.num_stages          = 3;
    params.stage_thrust_kn     = {500.0, 200.0, 50.0};
    params.stage_burn_time_sec = {60.0, 40.0, 30.0};
    params.stage_mass_kg       = {20000.0, 4500.0, 800.0};
    params.stage_propellant_kg = {15000.0, 3500.0, 600.0};
    params.kill_vehicle_mass_kg = 50.0;
    params.max_lateral_accel_g  = 15.0;
    return params;
}

// ─── Atmospheric density model ─────────────────────────────────────────────
//
// Exponential atmosphere: rho = rho0 * exp(-alt / H)
// Valid below ~100 km; above that, density is effectively zero.

double Interceptor::atmospheric_density(double altitude_km) const {
    static constexpr double RHO0 = 1.225;  // kg/m^3 at sea level
    static constexpr double H    = 8.5;    // Scale height (km)

    if (altitude_km > 100.0) return 0.0;
    if (altitude_km < 0.0)   altitude_km = 0.0;

    return RHO0 * std::exp(-altitude_km / H);
}

// ─── Launch site to ECI position ───────────────────────────────────────────

Vec3 Interceptor::site_to_eci(const LaunchSite& site, double gmst) const {
    double lat_rad = site.lat * DEG_TO_RAD;
    double lon_rad = site.lon * DEG_TO_RAD;
    double R = EARTH_RADIUS_KM + site.altitude_km;

    // Earth-fixed (ECEF) coordinates
    double x_ecef = R * std::cos(lat_rad) * std::cos(lon_rad);
    double y_ecef = R * std::cos(lat_rad) * std::sin(lon_rad);
    double z_ecef = R * std::sin(lat_rad);

    // Rotate from ECEF to ECI by GMST angle
    double cos_g = std::cos(gmst);
    double sin_g = std::sin(gmst);

    Vec3 eci;
    eci.x = cos_g * x_ecef - sin_g * y_ecef;
    eci.y = sin_g * x_ecef + cos_g * y_ecef;
    eci.z = z_ecef;

    return eci;
}

// ─── Aerodynamic drag ──────────────────────────────────────────────────────

Vec3 Interceptor::compute_drag(const Vec3& velocity, double altitude_km,
                                double mass_kg, double cd_a) const {
    if (altitude_km > 80.0 || mass_kg <= 0.0) {
        return {0.0, 0.0, 0.0};
    }

    double rho = atmospheric_density(altitude_km);

    // Velocity magnitude (km/s → m/s for drag, then convert back)
    double v_ms_x = velocity.x * 1000.0;
    double v_ms_y = velocity.y * 1000.0;
    double v_ms_z = velocity.z * 1000.0;
    double v_mag = std::sqrt(v_ms_x * v_ms_x + v_ms_y * v_ms_y + v_ms_z * v_ms_z);

    if (v_mag < 1.0e-6) return {0.0, 0.0, 0.0};

    // F_drag = 0.5 * rho * v^2 * Cd * A (Newtons)
    double drag_force = 0.5 * rho * v_mag * v_mag * cd_a;

    // Acceleration (m/s^2), opposing velocity direction
    double a_drag = drag_force / mass_kg;

    // Convert back to km/s^2
    double a_drag_kms2 = a_drag / 1000.0;

    // Direction: opposing velocity
    Vec3 drag_accel;
    drag_accel.x = -a_drag_kms2 * (v_ms_x / v_mag);
    drag_accel.y = -a_drag_kms2 * (v_ms_y / v_mag);
    drag_accel.z = -a_drag_kms2 * (v_ms_z / v_mag);

    return drag_accel;
}

// ─── Proportional Navigation guidance ──────────────────────────────────────
//
// a_cmd = N * Vc * omega_LOS
// where N = nav gain, Vc = closing velocity, omega_LOS = LOS rate vector

Vec3 Interceptor::compute_pn_acceleration(const Vec3& los,
                                            const Vec3& los_rate,
                                            double closing_speed,
                                            double nav_gain) const {
    // LOS magnitude
    double los_mag = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);
    if (los_mag < 1.0e-10) return {0.0, 0.0, 0.0};

    // LOS unit vector
    double lx = los.x / los_mag;
    double ly = los.y / los_mag;
    double lz = los.z / los_mag;

    // omega_LOS = (LOS x LOS_rate) / |LOS|^2
    double omega_x = (los.y * los_rate.z - los.z * los_rate.y) / (los_mag * los_mag);
    double omega_y = (los.z * los_rate.x - los.x * los_rate.z) / (los_mag * los_mag);
    double omega_z = (los.x * los_rate.y - los.y * los_rate.x) / (los_mag * los_mag);

    // a_cmd = N * Vc * omega_LOS
    double scale = nav_gain * closing_speed;
    Vec3 accel;
    accel.x = scale * omega_x;
    accel.y = scale * omega_y;
    accel.z = scale * omega_z;

    return accel;
}

// ─── Simulate interceptor trajectory ───────────────────────────────────────
//
// Full 3DOF simulation:
//   1. Initialize at launch site ECI position
//   2. Boost phase: thrust along launch direction, staging
//   3. Post-boost: PN guidance toward target
//   4. Track closest approach to target

std::vector<Interceptor::TrajectoryPoint> Interceptor::simulate_trajectory(
    const LaunchSite& site,
    double launch_azimuth_deg,
    double launch_elevation_deg,
    const InterceptorParams& params,
    const std::vector<OrbitalState>& target_states,
    double dt_sec,
    double max_time_sec) const {

    std::vector<TrajectoryPoint> trajectory;

    // Compute GMST at start time
    Timestamp t0 = target_states.empty() ? 0 : target_states[0].epoch;
    double gmst = Propagator::compute_gmst(t0);

    // Initial position (ECI)
    Vec3 pos = site_to_eci(site, gmst);

    // Earth rotation velocity at launch site (km/s, ECI)
    double lat_rad = site.lat * DEG_TO_RAD;
    double R_site = EARTH_RADIUS_KM + site.altitude_km;
    double v_rot = OMEGA_EARTH * R_site * std::cos(lat_rad); // tangential speed

    // Compute initial ECI velocity due to Earth rotation
    double lon_rad = site.lon * DEG_TO_RAD;
    double theta = gmst + lon_rad; // ECI angle
    Vec3 vel;
    vel.x = -v_rot * std::sin(theta);
    vel.y =  v_rot * std::cos(theta);
    vel.z = 0.0;

    // Launch direction in ECI (from azimuth and elevation at the site)
    double az_rad = launch_azimuth_deg * DEG_TO_RAD;
    double el_rad = launch_elevation_deg * DEG_TO_RAD;

    // Local ENU to ECEF rotation
    // East: d(lon), North: d(lat), Up: radial
    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double sin_lon = std::sin(theta); // Use ECI angle
    double cos_lon = std::cos(theta);

    // Launch direction in local ENU
    double e_enu = std::sin(az_rad) * std::cos(el_rad);  // East
    double n_enu = std::cos(az_rad) * std::cos(el_rad);  // North
    double u_enu = std::sin(el_rad);                       // Up

    // ENU to ECI rotation
    Vec3 launch_dir;
    launch_dir.x = -sin_lon * e_enu - sin_lat * cos_lon * n_enu + cos_lat * cos_lon * u_enu;
    launch_dir.y =  cos_lon * e_enu - sin_lat * sin_lon * n_enu + cos_lat * sin_lon * u_enu;
    launch_dir.z =                     cos_lat * n_enu            + sin_lat * u_enu;

    // Normalize launch direction
    double ld_mag = std::sqrt(launch_dir.x * launch_dir.x +
                              launch_dir.y * launch_dir.y +
                              launch_dir.z * launch_dir.z);
    if (ld_mag > 1.0e-10) {
        launch_dir.x /= ld_mag;
        launch_dir.y /= ld_mag;
        launch_dir.z /= ld_mag;
    }

    // Current mass
    double mass = (params.num_stages > 0) ? params.stage_mass_kg[0] : 1000.0;
    int current_stage = 0;
    double stage_time = 0.0; // Time into current stage burn

    // Total burn time per stage (cumulative boundaries)
    std::vector<double> burn_end_times;
    double cum_burn = 0.0;
    for (int s = 0; s < params.num_stages; ++s) {
        cum_burn += params.stage_burn_time_sec[s];
        burn_end_times.push_back(cum_burn);
    }
    double total_burn_time = cum_burn;

    // Track closest approach
    double min_dist = 1.0e12;
    double elapsed = 0.0;

    while (elapsed < max_time_sec) {
        // Record trajectory point
        double r_mag = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
        double alt_km = r_mag - EARTH_RADIUS_KM;

        // Downrange distance (great circle from launch site, approximate)
        Vec3 site_eci = site_to_eci(site, gmst + OMEGA_EARTH * elapsed);
        double dx = pos.x - site_eci.x;
        double dy = pos.y - site_eci.y;
        double dz = pos.z - site_eci.z;
        double downrange = std::sqrt(dx * dx + dy * dy + dz * dz);

        TrajectoryPoint tp;
        tp.time_sec      = elapsed;
        tp.position       = pos;
        tp.velocity       = vel;
        tp.mass_kg        = mass;
        tp.current_stage  = current_stage;
        tp.altitude_km    = alt_km;
        tp.downrange_km   = downrange;
        trajectory.push_back(tp);

        // Check closest approach to target
        if (!target_states.empty()) {
            size_t idx = std::min(static_cast<size_t>(elapsed / dt_sec),
                                  target_states.size() - 1);
            const auto& tgt = target_states[idx];
            double tdx = pos.x - tgt.position.x;
            double tdy = pos.y - tgt.position.y;
            double tdz = pos.z - tgt.position.z;
            double dist = std::sqrt(tdx * tdx + tdy * tdy + tdz * tdz);

            if (dist < min_dist) {
                min_dist = dist;
            } else if (dist > min_dist * 1.5 && elapsed > total_burn_time) {
                // Past closest approach and diverging, stop simulation
                break;
            }
        }

        // Check if we've hit the ground
        if (alt_km < 0.0 && elapsed > 10.0) break;

        // ── Compute accelerations ──────────────────────────────────────

        // Gravity: a = -mu * r / |r|^3
        double r3 = r_mag * r_mag * r_mag;
        Vec3 a_grav;
        a_grav.x = -MU_EARTH * pos.x / r3;
        a_grav.y = -MU_EARTH * pos.y / r3;
        a_grav.z = -MU_EARTH * pos.z / r3;

        // Drag
        Vec3 a_drag = compute_drag(vel, alt_km, mass);

        // Thrust
        Vec3 a_thrust = {0.0, 0.0, 0.0};
        bool boosting = (elapsed < total_burn_time && current_stage < params.num_stages);

        if (boosting) {
            // Determine current stage
            while (current_stage < params.num_stages - 1 &&
                   elapsed >= burn_end_times[current_stage]) {
                current_stage++;
                mass = params.stage_mass_kg[current_stage];
            }

            double thrust_kn = params.stage_thrust_kn[current_stage];
            double thrust_n  = thrust_kn * 1000.0; // N

            // Thrust acceleration (km/s^2)
            double a_thrust_mag = thrust_n / (mass * 1000.0); // N / (kg) → m/s^2, then /1000 → km/s^2
            a_thrust_mag = thrust_n / mass / 1000.0;

            // During boost, thrust along launch direction
            a_thrust.x = a_thrust_mag * launch_dir.x;
            a_thrust.y = a_thrust_mag * launch_dir.y;
            a_thrust.z = a_thrust_mag * launch_dir.z;

            // Mass flow rate: dm/dt = thrust / (Isp * g0)
            // Isp from thrust and propellant: Isp = thrust * burn_time / (propellant * g0)
            double burn_time = params.stage_burn_time_sec[current_stage];
            double prop_mass = params.stage_propellant_kg[current_stage];
            double mdot = prop_mass / burn_time; // kg/s
            mass -= mdot * dt_sec;
            if (mass < params.kill_vehicle_mass_kg) {
                mass = params.kill_vehicle_mass_kg;
            }
        } else if (!target_states.empty()) {
            // Post-boost: apply proportional navigation toward target
            size_t idx = std::min(static_cast<size_t>(elapsed / dt_sec),
                                  target_states.size() - 1);
            const auto& tgt = target_states[idx];

            Vec3 los;
            los.x = tgt.position.x - pos.x;
            los.y = tgt.position.y - pos.y;
            los.z = tgt.position.z - pos.z;

            Vec3 rel_vel;
            rel_vel.x = tgt.velocity.x - vel.x;
            rel_vel.y = tgt.velocity.y - vel.y;
            rel_vel.z = tgt.velocity.z - vel.z;

            double closing = -(los.x * rel_vel.x + los.y * rel_vel.y + los.z * rel_vel.z) /
                             std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);

            Vec3 pn = compute_pn_acceleration(los, rel_vel, std::abs(closing));

            // Limit PN acceleration to KV capability
            double max_a = params.max_lateral_accel_g * G0 / 1000.0; // km/s^2
            double pn_mag = std::sqrt(pn.x * pn.x + pn.y * pn.y + pn.z * pn.z);
            if (pn_mag > max_a && pn_mag > 1.0e-10) {
                double scale = max_a / pn_mag;
                pn.x *= scale;
                pn.y *= scale;
                pn.z *= scale;
            }

            a_thrust = pn;
        }

        // Total acceleration
        Vec3 a_total;
        a_total.x = a_grav.x + a_drag.x + a_thrust.x;
        a_total.y = a_grav.y + a_drag.y + a_thrust.y;
        a_total.z = a_grav.z + a_drag.z + a_thrust.z;

        // Euler integration
        vel.x += a_total.x * dt_sec;
        vel.y += a_total.y * dt_sec;
        vel.z += a_total.z * dt_sec;

        pos.x += vel.x * dt_sec;
        pos.y += vel.y * dt_sec;
        pos.z += vel.z * dt_sec;

        elapsed += dt_sec;
    }

    return trajectory;
}

// ─── Solve launch parameters ───────────────────────────────────────────────
//
// Simplified Lambert-like approach: compute required velocity from launch
// site to intercept point for given time-of-flight.

Interceptor::LaunchSolution Interceptor::solve_launch_params(
    const LaunchSite& site,
    const OrbitalState& target_at_intercept,
    const InterceptorParams& params,
    double intercept_time_sec) const {

    LaunchSolution sol;
    sol.converged = false;

    double gmst = Propagator::compute_gmst(target_at_intercept.epoch -
                                            static_cast<Timestamp>(intercept_time_sec));
    Vec3 r_site = site_to_eci(site, gmst);
    Vec3 r_tgt  = target_at_intercept.position;

    // Required displacement
    double dx = r_tgt.x - r_site.x;
    double dy = r_tgt.y - r_site.y;
    double dz = r_tgt.z - r_site.z;

    if (intercept_time_sec <= 0.0) return sol;

    // Simplified velocity requirement: V_req = dr/dt - 0.5*g*t (vertical component)
    double r_site_mag = std::sqrt(r_site.x * r_site.x + r_site.y * r_site.y + r_site.z * r_site.z);

    // Gravity correction along radial direction
    double g_local = MU_EARTH / (r_site_mag * r_site_mag); // km/s^2

    // Required velocity components (km/s)
    double vx_req = dx / intercept_time_sec;
    double vy_req = dy / intercept_time_sec;
    double vz_req = dz / intercept_time_sec;

    // Add gravity loss correction (approximate: half gravity * time along radial)
    double rx_hat = r_site.x / r_site_mag;
    double ry_hat = r_site.y / r_site_mag;
    double rz_hat = r_site.z / r_site_mag;

    double grav_correction = 0.5 * g_local * intercept_time_sec;
    vx_req += grav_correction * rx_hat;
    vy_req += grav_correction * ry_hat;
    vz_req += grav_correction * rz_hat;

    // Subtract Earth rotation velocity at launch site
    double lat_rad = site.lat * DEG_TO_RAD;
    double v_rot = OMEGA_EARTH * r_site_mag * std::cos(lat_rad);
    double theta = gmst + site.lon * DEG_TO_RAD;
    vx_req += v_rot * std::sin(theta);
    vy_req -= v_rot * std::cos(theta);

    // Total delta-V
    double dv = std::sqrt(vx_req * vx_req + vy_req * vy_req + vz_req * vz_req);

    // Compute azimuth and elevation from velocity direction at launch site
    // Transform velocity requirement to local ENU
    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double sin_lon = std::sin(theta);
    double cos_lon = std::cos(theta);

    // ECI to ENU rotation
    double v_east  = -sin_lon * vx_req + cos_lon * vy_req;
    double v_north = -sin_lat * cos_lon * vx_req - sin_lat * sin_lon * vy_req + cos_lat * vz_req;
    double v_up    =  cos_lat * cos_lon * vx_req + cos_lat * sin_lon * vy_req + sin_lat * vz_req;

    sol.azimuth_deg       = std::atan2(v_east, v_north) * RAD_TO_DEG;
    if (sol.azimuth_deg < 0.0) sol.azimuth_deg += 360.0;

    double v_horiz = std::sqrt(v_east * v_east + v_north * v_north);
    sol.elevation_deg     = std::atan2(v_up, v_horiz) * RAD_TO_DEG;
    sol.time_of_flight_sec = intercept_time_sec;
    sol.delta_v_kms        = dv;
    sol.converged          = true;

    return sol;
}

// ─── Compute delta-V to intercept point ────────────────────────────────────

double Interceptor::compute_delta_v(const LaunchSite& site,
                                     const Vec3& intercept_position,
                                     double time_of_flight_sec) const {
    if (time_of_flight_sec <= 0.0) return 1.0e12;

    double r_site_mag = EARTH_RADIUS_KM + site.altitude_km;
    double r_tgt_mag  = std::sqrt(intercept_position.x * intercept_position.x +
                                   intercept_position.y * intercept_position.y +
                                   intercept_position.z * intercept_position.z);

    // Altitude gain (km)
    double alt_gain = r_tgt_mag - r_site_mag;
    if (alt_gain < 0.0) alt_gain = 0.0;

    // Use vis-viva energy approach for direct ascent:
    //   Required velocity to reach altitude h from radius r0:
    //   v = sqrt(2 * mu * (1/r0 - 1/r_tgt))
    double v_vertical = std::sqrt(2.0 * MU_EARTH * (1.0 / r_site_mag - 1.0 / r_tgt_mag));

    // Gravity losses during boost (~15-20% for a direct ascent)
    double dv = v_vertical * 1.15;

    // Earth rotation contribution (subtract for benefit)
    double lat_rad = site.lat * DEG_TO_RAD;
    double v_rot = OMEGA_EARTH * r_site_mag * std::cos(lat_rad);
    dv -= v_rot * 0.3; // Partial benefit

    return std::max(dv, 0.1);
}

} // namespace da_asat
