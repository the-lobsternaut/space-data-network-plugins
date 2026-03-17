#include "ascent_reconstruct/trajectory.h"
#include <algorithm>
#include <cmath>

namespace ascent_reconstruct {

// J2 perturbation coefficient
static constexpr double J2 = 1.08263e-3;

Vec3 TrajectorySimulator::geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m) {
    double lat = lat_deg * DEG2RAD;
    double lon = lon_deg * DEG2RAD;
    double cos_lat = std::cos(lat);
    double sin_lat = std::sin(lat);
    double cos_lon = std::cos(lon);
    double sin_lon = std::sin(lon);
    double r = R_EARTH + alt_m;
    return {
        r * cos_lat * cos_lon,
        r * cos_lat * sin_lon,
        r * sin_lat
    };
}

LatLonAlt TrajectorySimulator::ecef_to_geodetic(const Vec3& ecef) {
    double x = ecef.x, y = ecef.y, z = ecef.z;
    double r = ecef.norm();
    double lon = std::atan2(y, x) * RAD2DEG;
    double lat = std::asin(z / r) * RAD2DEG;
    double alt = r - R_EARTH;
    return {lat, lon, alt};
}

Vec3 TrajectorySimulator::gravity(const Vec3& pos) {
    double r = pos.norm();
    if (r < 1.0) return {0, 0, 0};

    double r2 = r * r;
    double r3 = r2 * r;
    double r5 = r2 * r3;

    // Central gravity
    double mu_r3 = -GM / r3;
    Vec3 g_central = pos * mu_r3;

    // J2 perturbation
    double z2 = pos.z * pos.z;
    double factor = 1.5 * J2 * GM * R_EARTH * R_EARTH / r5;
    double common = 5.0 * z2 / r2;

    Vec3 g_j2;
    g_j2.x = factor * pos.x * (common - 1.0);
    g_j2.y = factor * pos.y * (common - 1.0);
    g_j2.z = factor * pos.z * (common - 3.0);

    return g_central + g_j2;
}

double TrajectorySimulator::atm_density(double alt_m) {
    if (alt_m < 0.0) alt_m = 0.0;
    if (alt_m > 200000.0) return 0.0;

    // Multi-layer exponential atmosphere model
    struct Layer {
        double alt_base;
        double rho_base;
        double scale_height;
    };

    static const Layer layers[] = {
        {0.0,      1.225,      8500.0},
        {25000.0,  3.899e-2,   6500.0},
        {50000.0,  1.027e-3,   7200.0},
        {75000.0,  3.992e-5,   6300.0},
        {100000.0, 5.604e-7,   5900.0},
        {150000.0, 2.076e-9,   7400.0},
    };

    int n_layers = 6;
    int layer_idx = 0;
    for (int i = n_layers - 1; i >= 0; --i) {
        if (alt_m >= layers[i].alt_base) {
            layer_idx = i;
            break;
        }
    }

    const Layer& L = layers[layer_idx];
    return L.rho_base * std::exp(-(alt_m - L.alt_base) / L.scale_height);
}

Vec3 TrajectorySimulator::drag_acceleration(const Vec3& vel, double alt_m,
                                              double cd, double area, double mass) {
    if (mass <= 0.0) return {0, 0, 0};
    double rho = atm_density(alt_m);
    if (rho < 1e-15) return {0, 0, 0};

    double v_mag = vel.norm();
    if (v_mag < 1e-6) return {0, 0, 0};

    double drag_force = 0.5 * rho * v_mag * v_mag * cd * area;
    Vec3 v_hat = vel.normalized();
    return v_hat * (-drag_force / mass);
}

Vec3 TrajectorySimulator::thrust_direction(const Vec3& pos, const Vec3& vel,
                                             double pitch_from_vertical) {
    double v_mag = vel.norm();

    // If velocity is very small, thrust vertically (along position vector from Earth center)
    if (v_mag < 10.0) {
        return pos.normalized();
    }

    // Gravity turn: thrust along velocity vector
    Vec3 v_hat = vel.normalized();
    Vec3 r_hat = pos.normalized();

    // Blend between vertical and velocity direction based on pitch_from_vertical
    // pitch_from_vertical = PI/2 means fully vertical, 0 means along velocity
    double alpha = pitch_from_vertical;
    if (alpha < 0.01) {
        return v_hat;
    }

    // Construct thrust direction as blend of radial and velocity
    // Use the component of velocity perpendicular to radial for the horizontal part
    Vec3 v_radial = r_hat * v_hat.dot(r_hat);
    Vec3 v_horiz = (v_hat - v_radial).normalized();

    double cos_a = std::cos(alpha);
    double sin_a = std::sin(alpha);

    // alpha is angle from horizontal, so sin(alpha) is radial, cos(alpha) is horizontal
    Vec3 dir;
    if (v_horiz.norm() > 0.01) {
        dir = r_hat * sin_a + v_horiz * cos_a;
    } else {
        dir = r_hat;
    }
    return dir.normalized();
}

int TrajectorySimulator::active_stage(const AscentModelParams& params, double t) {
    double t_accum = 0.0;
    for (int i = 0; i < (int)params.stages.size(); ++i) {
        t_accum += params.stages[i].burn_time;
        if (t < t_accum) return i;
    }
    return (int)params.stages.size() - 1;
}

double TrajectorySimulator::stage_start_time(const AscentModelParams& params, int stage) {
    double t = 0.0;
    for (int i = 0; i < stage && i < (int)params.stages.size(); ++i) {
        t += params.stages[i].burn_time;
    }
    return t;
}

TrajectorySimulator::FlightDerivs TrajectorySimulator::derivatives(
        const FlightState& state,
        const AscentModelParams& params,
        double t) const {

    FlightDerivs d;
    d.dpos = state.vel;

    // Gravity
    Vec3 g = gravity(state.pos);

    // Determine active stage
    int stg = active_stage(params, t);
    const auto& sp = params.stages[stg];

    // Time within current stage
    double t_stage_start = stage_start_time(params, stg);
    double t_in_stage = t - t_stage_start;

    // Check if stage is still burning
    bool burning = (t_in_stage < sp.burn_time) && (state.mass > sp.mass_final);

    Vec3 thrust_accel = {0, 0, 0};
    d.dmass = 0.0;

    if (burning && sp.thrust > 0.0 && sp.isp > 0.0) {
        // Mass flow rate
        double mdot = sp.thrust / (sp.isp * G0);
        d.dmass = -mdot;

        // Compute pitch angle for gravity turn
        // Early flight: nearly vertical. Then pitch over gradually.
        double pitch_kick_time = 5.0; // seconds after launch to begin pitch
        double pitch_angle;

        if (t < pitch_kick_time) {
            pitch_angle = params.initial_pitch; // nearly vertical
        } else {
            // Gravity turn: reduce pitch over time
            pitch_angle = params.initial_pitch - sp.pitch_rate * (t - pitch_kick_time);
            if (pitch_angle < 0.0) pitch_angle = 0.0;
        }

        Vec3 t_dir = thrust_direction(state.pos, state.vel, pitch_angle);

        // Apply launch azimuth correction for early flight
        if (t < pitch_kick_time + 10.0) {
            // During early flight, incorporate launch azimuth
            LatLonAlt lla = ecef_to_geodetic(state.pos);
            double lat = lla.lat * DEG2RAD;
            double lon = lla.lon * DEG2RAD;
            double az = params.launch_azimuth;

            // East-North-Up to ECEF rotation
            Vec3 east = {-std::sin(lon), std::cos(lon), 0.0};
            Vec3 north = {-std::sin(lat)*std::cos(lon), -std::sin(lat)*std::sin(lon), std::cos(lat)};
            Vec3 up = {std::cos(lat)*std::cos(lon), std::cos(lat)*std::sin(lon), std::sin(lat)};

            // Horizontal direction from azimuth
            Vec3 horiz = east * std::sin(az) + north * std::cos(az);

            // Blend azimuth into thrust direction
            double blend = std::max(0.0, 1.0 - (t / (pitch_kick_time + 10.0)));
            double cos_p = std::cos(pitch_angle);
            double sin_p = std::sin(pitch_angle);
            Vec3 t_az = up * sin_p + horiz * cos_p;
            t_dir = (t_az * blend + t_dir * (1.0 - blend)).normalized();
        }

        thrust_accel = t_dir * (sp.thrust / state.mass);
    }

    // Drag
    LatLonAlt lla = ecef_to_geodetic(state.pos);
    double cd = 0.3;
    double area = 10.0; // m^2 reference area
    Vec3 drag = drag_acceleration(state.vel, lla.alt, cd, area, state.mass);

    // Coriolis and centrifugal (Earth rotation effects)
    Vec3 omega = {0, 0, OMEGA_EARTH};
    Vec3 coriolis = omega.cross(state.vel) * (-2.0);
    Vec3 centrifugal = omega.cross(omega.cross(state.pos)) * (-1.0);

    d.dvel = g + thrust_accel + drag + coriolis + centrifugal;

    return d;
}

std::vector<StateVector> TrajectorySimulator::simulate(
        const AscentModelParams& params,
        double t_end,
        double dt) const {

    std::vector<StateVector> trajectory;

    if (params.stages.empty()) return trajectory;

    // Initial position at launch site
    Vec3 pos = geodetic_to_ecef(params.launch_lat, params.launch_lon, params.launch_alt);

    // Initial velocity (Earth rotation)
    double lat_rad = params.launch_lat * DEG2RAD;
    double lon_rad = params.launch_lon * DEG2RAD;
    Vec3 omega = {0, 0, OMEGA_EARTH};
    Vec3 vel = omega.cross(pos);

    double mass = params.stages[0].mass_initial;

    int prev_stage = 0;

    for (double t = 0.0; t <= t_end; t += dt) {
        // Check for staging transition
        int cur_stage = active_stage(params, t);
        if (cur_stage != prev_stage && cur_stage < (int)params.stages.size()) {
            mass = params.stages[cur_stage].mass_initial;
            prev_stage = cur_stage;
        }

        // Record state
        LatLonAlt lla = ecef_to_geodetic(pos);
        double speed = vel.norm();

        // Flight path angle: angle between velocity and local horizontal
        Vec3 r_hat = pos.normalized();
        double v_radial = vel.dot(r_hat);
        double v_horiz_sq = speed * speed - v_radial * v_radial;
        double v_horiz = v_horiz_sq > 0 ? std::sqrt(v_horiz_sq) : 0.0;
        double fpa = std::atan2(v_radial, v_horiz) * RAD2DEG;

        // Heading: azimuth of velocity vector projected onto local horizontal
        double lat = lla.lat * DEG2RAD;
        double lon = lla.lon * DEG2RAD;
        Vec3 east = {-std::sin(lon), std::cos(lon), 0.0};
        Vec3 north = {-std::sin(lat)*std::cos(lon), -std::sin(lat)*std::sin(lon), std::cos(lat)};
        double v_east = vel.dot(east);
        double v_north = vel.dot(north);
        double heading = std::atan2(v_east, v_north) * RAD2DEG;
        if (heading < 0) heading += 360.0;

        StateVector sv;
        sv.time = t;
        sv.position = pos;
        sv.velocity = vel;
        sv.mass = mass;
        sv.altitude_m = lla.alt;
        sv.speed_mps = speed;
        sv.flight_path_angle_deg = fpa;
        sv.heading_deg = heading;
        sv.stage = cur_stage + 1;

        // Determine thrust
        double t_stage_start = stage_start_time(params, cur_stage);
        double t_in_stage = t - t_stage_start;
        if (cur_stage < (int)params.stages.size() &&
            t_in_stage < params.stages[cur_stage].burn_time) {
            sv.thrust = params.stages[cur_stage].thrust;
        }

        trajectory.push_back(sv);

        // Abort if below ground (crashed) after initial ascent
        if (t > 10.0 && lla.alt < -1000.0) break;

        // RK4 integration step
        FlightState s0 = {pos, vel, mass};

        FlightDerivs k1 = derivatives(s0, params, t);

        FlightState s1 = {
            s0.pos + k1.dpos * (dt * 0.5),
            s0.vel + k1.dvel * (dt * 0.5),
            s0.mass + k1.dmass * (dt * 0.5)
        };
        FlightDerivs k2 = derivatives(s1, params, t + dt * 0.5);

        FlightState s2 = {
            s0.pos + k2.dpos * (dt * 0.5),
            s0.vel + k2.dvel * (dt * 0.5),
            s0.mass + k2.dmass * (dt * 0.5)
        };
        FlightDerivs k3 = derivatives(s2, params, t + dt * 0.5);

        FlightState s3 = {
            s0.pos + k3.dpos * dt,
            s0.vel + k3.dvel * dt,
            s0.mass + k3.dmass * dt
        };
        FlightDerivs k4 = derivatives(s3, params, t + dt);

        pos = pos + (k1.dpos + k2.dpos * 2.0 + k3.dpos * 2.0 + k4.dpos) * (dt / 6.0);
        vel = vel + (k1.dvel + k2.dvel * 2.0 + k3.dvel * 2.0 + k4.dvel) * (dt / 6.0);
        mass = mass + (k1.dmass + k2.dmass * 2.0 + k3.dmass * 2.0 + k4.dmass) * (dt / 6.0);

        // Enforce minimum mass
        if (cur_stage < (int)params.stages.size()) {
            if (mass < params.stages[cur_stage].mass_final) {
                mass = params.stages[cur_stage].mass_final;
            }
        }
    }

    return trajectory;
}

} // namespace ascent_reconstruct
