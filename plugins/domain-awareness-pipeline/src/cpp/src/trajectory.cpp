#include "da_pipeline/trajectory.h"
#include <cmath>
#include <algorithm>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// geodetic_to_ecef (meters)
// ---------------------------------------------------------------------------
Vec3 TrajectorySimulator::geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m) {
    double lat = lat_deg * DEG2RAD;
    double lon = lon_deg * DEG2RAD;

    double f = 1.0 / 298.257223563;
    double e2 = 2 * f - f * f;
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double N = R_EARTH_M / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

    Vec3 ecef;
    ecef.x = (N + alt_m) * cos_lat * std::cos(lon);
    ecef.y = (N + alt_m) * cos_lat * std::sin(lon);
    ecef.z = (N * (1.0 - e2) + alt_m) * sin_lat;
    return ecef;
}

// ---------------------------------------------------------------------------
// ecef_to_geodetic
// ---------------------------------------------------------------------------
LatLonAlt TrajectorySimulator::ecef_to_geodetic(const Vec3& ecef) {
    double f = 1.0 / 298.257223563;
    double e2 = 2 * f - f * f;

    double p = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    double lon = std::atan2(ecef.y, ecef.x);

    // Iterative lat computation
    double lat = std::atan2(ecef.z, p * (1.0 - e2)); // initial
    for (int i = 0; i < 10; i++) {
        double sin_lat = std::sin(lat);
        double N = R_EARTH_M / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
        lat = std::atan2(ecef.z + N * e2 * sin_lat, p);
    }

    double sin_lat = std::sin(lat);
    double N = R_EARTH_M / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    double alt = p / std::cos(lat) - N;

    // Handle pole singularity
    if (std::abs(lat) > 89.999 * DEG2RAD) {
        alt = std::abs(ecef.z) / std::abs(sin_lat) - N * (1.0 - e2);
    }

    LatLonAlt lla;
    lla.lat = lat * RAD2DEG;
    lla.lon = lon * RAD2DEG;
    lla.alt = alt;
    return lla;
}

// ---------------------------------------------------------------------------
// gravity — inverse-square + J2 perturbation
// ---------------------------------------------------------------------------
Vec3 TrajectorySimulator::gravity(const Vec3& pos) {
    double r = pos.norm();
    if (r < 1.0) return {0, 0, -G0}; // Protect against singularity

    double r2 = r * r;
    double r3 = r * r2;
    double r5 = r2 * r3;
    double mu = MU_EARTH * 1e9; // Convert km^3/s^2 to m^3/s^2

    // Central body gravity
    double gx = -mu * pos.x / r3;
    double gy = -mu * pos.y / r3;
    double gz = -mu * pos.z / r3;

    // J2 perturbation
    double Re = R_EARTH_M;
    double J2_coeff = 1.5 * J2 * mu * Re * Re;
    double z2_r2 = (pos.z * pos.z) / r2;

    gx += J2_coeff * pos.x / r5 * (5.0 * z2_r2 - 1.0);
    gy += J2_coeff * pos.y / r5 * (5.0 * z2_r2 - 1.0);
    gz += J2_coeff * pos.z / r5 * (5.0 * z2_r2 - 3.0);

    return {gx, gy, gz};
}

// ---------------------------------------------------------------------------
// atm_density — exponential model
// ---------------------------------------------------------------------------
double TrajectorySimulator::atm_density(double alt_m) {
    double alt_km = alt_m / 1000.0;
    if (alt_km < 0) alt_km = 0;
    if (alt_km > 1000) return 0;

    // Scale heights and reference densities for altitude bands
    struct AtmLayer { double alt_base; double rho_base; double H; };
    static const AtmLayer layers[] = {
        {0,    1.225,       8.5},
        {25,   3.899e-2,    6.349},
        {30,   1.774e-2,    6.682},
        {40,   3.972e-3,    7.554},
        {50,   1.057e-3,    8.382},
        {60,   3.206e-4,    7.714},
        {70,   8.770e-5,    6.549},
        {80,   1.905e-5,    5.799},
        {90,   3.396e-6,    5.382},
        {100,  5.604e-7,   5.877},
        {110,  9.708e-8,    7.263},
        {120,  2.222e-8,    9.473},
        {130,  8.152e-9,   12.636},
        {140,  3.831e-9,   16.149},
        {150,  2.076e-9,   22.523},
        {180,  5.194e-10,  29.740},
        {200,  2.541e-10,  37.105},
        {250,  6.073e-11,  45.546},
        {300,  1.916e-11,  53.628},
        {350,  7.014e-12,  53.298},
        {400,  2.803e-12,  58.515},
        {450,  1.184e-12,  60.828},
        {500,  5.215e-13,  63.822},
        {600,  1.137e-13,  71.835},
        {700,  3.070e-14,  88.667},
        {800,  1.136e-14, 124.64},
        {900,  5.759e-15, 181.05},
        {1000, 3.561e-15, 268.00},
    };

    // Find appropriate layer
    int idx = 0;
    for (int i = 0; i < 28; i++) {
        if (alt_km >= layers[i].alt_base) idx = i;
        else break;
    }

    double dh = alt_km - layers[idx].alt_base;
    return layers[idx].rho_base * std::exp(-dh / layers[idx].H);
}

// ---------------------------------------------------------------------------
// active_stage
// ---------------------------------------------------------------------------
int TrajectorySimulator::active_stage(const AscentModelParams& params, double t) {
    double t_accum = 0;
    for (int i = 0; i < static_cast<int>(params.stages.size()); i++) {
        t_accum += params.stages[i].burn_time;
        if (t < t_accum) return i;
    }
    return -1; // Coast phase (all stages burnt out)
}

// ---------------------------------------------------------------------------
// simulate — RK4 integration for powered ascent
// ---------------------------------------------------------------------------
std::vector<StateVector> TrajectorySimulator::simulate(
    const AscentModelParams& params, double t_end, double dt) const
{
    std::vector<StateVector> traj;

    // Initial position in ECEF (meters)
    Vec3 pos = geodetic_to_ecef(params.launch_lat, params.launch_lon, params.launch_alt);

    // Initial velocity: Earth rotation
    double lat_r = params.launch_lat * DEG2RAD;
    double lon_r = params.launch_lon * DEG2RAD;
    double v_rot = OMEGA_EARTH * (R_EARTH_M + params.launch_alt) * std::cos(lat_r);
    Vec3 vel = {
        -v_rot * std::sin(lon_r),
         v_rot * std::cos(lon_r),
         0
    };

    // Total initial mass
    double mass = 0;
    for (auto& s : params.stages) mass += s.mass_initial;
    if (mass <= 0) return traj;

    // Burn time accumulation per stage
    std::vector<double> stage_start_times;
    double t_accum = 0;
    for (auto& s : params.stages) {
        stage_start_times.push_back(t_accum);
        t_accum += s.burn_time;
    }

    // Launch azimuth and pitch directions
    double az = params.launch_azimuth * DEG2RAD;

    double t = 0;
    double cd = 0.3;        // drag coefficient
    double A_ref = 10.0;    // reference area m^2

    // Store initial (Earth rotation) velocity to compute inertial-relative velocity
    Vec3 vel_rot_init = vel;

    // Precompute local UP, EAST, NORTH at launch site for thrust direction
    double lat_r2 = params.launch_lat * DEG2RAD;
    double lon_r2 = params.launch_lon * DEG2RAD;
    Vec3 up = pos.normalized();  // radial (up)
    // East: (-sin(lon), cos(lon), 0)
    Vec3 east = {-std::sin(lon_r2), std::cos(lon_r2), 0};
    east = east.normalized();
    // North = up x east ... actually north = cross of east with up? Let's do it right:
    // North in ECEF for geodetic: (-sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat))
    Vec3 north = {
        -std::sin(lat_r2) * std::cos(lon_r2),
        -std::sin(lat_r2) * std::sin(lon_r2),
        std::cos(lat_r2)
    };
    north = north.normalized();

    // Downrange direction from azimuth: north*cos(az) + east*sin(az)
    Vec3 downrange = north * std::cos(az) + east * std::sin(az);

    while (t <= t_end) {
        // Record state
        LatLonAlt lla = ecef_to_geodetic(pos);
        double speed = vel.norm();
        double alt = lla.alt;

        // Flight path angle
        Vec3 pos_n = pos.normalized();
        double v_radial = vel.dot(pos_n);
        double v_horiz = std::sqrt(std::max(0.0, speed * speed - v_radial * v_radial));
        double fpa = std::atan2(v_radial, v_horiz) * RAD2DEG;

        // Heading
        double heading_rad = std::atan2(vel.x * std::cos(lla.lon * DEG2RAD) * (-std::sin(lla.lat * DEG2RAD)) +
                                        vel.y * std::sin(lla.lon * DEG2RAD) * (-std::sin(lla.lat * DEG2RAD)) +
                                        vel.z * std::cos(lla.lat * DEG2RAD),
                                        -vel.x * std::sin(lla.lon * DEG2RAD) +
                                        vel.y * std::cos(lla.lon * DEG2RAD));
        double heading = std::fmod(heading_rad * RAD2DEG + 360.0, 360.0);

        int stg = active_stage(params, t);
        double thrust_val = 0;
        if (stg >= 0 && stg < static_cast<int>(params.stages.size())) {
            thrust_val = params.stages[stg].thrust;
        }

        StateVector sv;
        sv.time = t;
        sv.position = pos;
        sv.velocity = vel;
        sv.mass = mass;
        sv.thrust = thrust_val;
        sv.altitude_m = alt;
        sv.speed_mps = speed;
        sv.flight_path_angle_deg = fpa;
        sv.heading_deg = heading;
        sv.stage = stg >= 0 ? stg : static_cast<int>(params.stages.size()) - 1;
        traj.push_back(sv);

        // Terminate if below ground after liftoff
        if (t > 10 && alt < -1000) break;

        // ---- RK4 integration ----
        auto compute_accel = [&](const Vec3& p, const Vec3& v, double time, double m) -> Vec3 {
            // Gravity
            Vec3 g = gravity(p);

            // Determine stage
            int s = active_stage(params, time);
            Vec3 thrust_vec = {0, 0, 0};

            if (s >= 0 && s < static_cast<int>(params.stages.size())) {
                double T = params.stages[s].thrust;

                if (T > 0 && m > 0) {
                    // Pitch program: start vertical, then gravity turn
                    // pitch_angle = 90 at t=0, decreasing by pitch_rate deg/s
                    double total_time = time;
                    double pitch_rate = params.stages[s].pitch_rate;
                    double pitch_angle_deg = 90.0 - pitch_rate * total_time;
                    if (pitch_angle_deg < 5.0) pitch_angle_deg = 5.0; // Min pitch

                    double pitch_rad = pitch_angle_deg * DEG2RAD;

                    // Thrust direction: combination of up and downrange
                    // up component = sin(pitch), downrange component = cos(pitch)
                    Vec3 local_up = p.normalized();
                    Vec3 thrust_dir = local_up * std::sin(pitch_rad) +
                                      downrange * std::cos(pitch_rad);
                    thrust_dir = thrust_dir.normalized();

                    thrust_vec = thrust_dir * (T / m);
                }
            }

            // Drag
            LatLonAlt lla_local = ecef_to_geodetic(p);
            double rho = atm_density(lla_local.alt);
            double spd = v.norm();
            Vec3 drag_vec = {0, 0, 0};
            if (spd > 0.1 && rho > 0) {
                double drag_mag = 0.5 * rho * spd * spd * cd * A_ref / m;
                // Drag opposes velocity relative to atmosphere (ignore wind)
                drag_vec = v.normalized() * (-drag_mag);
            }

            return g + thrust_vec + drag_vec;
        };

        // Mass flow
        int stg_now = active_stage(params, t);
        double mdot = 0;
        if (stg_now >= 0 && stg_now < static_cast<int>(params.stages.size())) {
            auto& sp = params.stages[stg_now];
            if (sp.isp > 0 && sp.thrust > 0) {
                mdot = sp.thrust / (sp.isp * G0);
            }
        }

        Vec3 k1v = compute_accel(pos, vel, t, mass) * dt;
        Vec3 k1x = vel * dt;

        Vec3 k2v = compute_accel(pos + k1x * 0.5, vel + k1v * 0.5, t + dt * 0.5, mass - mdot * dt * 0.5) * dt;
        Vec3 k2x = (vel + k1v * 0.5) * dt;

        Vec3 k3v = compute_accel(pos + k2x * 0.5, vel + k2v * 0.5, t + dt * 0.5, mass - mdot * dt * 0.5) * dt;
        Vec3 k3x = (vel + k2v * 0.5) * dt;

        Vec3 k4v = compute_accel(pos + k3x, vel + k3v, t + dt, mass - mdot * dt) * dt;
        Vec3 k4x = (vel + k3v) * dt;

        vel = vel + (k1v + k2v * 2 + k3v * 2 + k4v) * (1.0 / 6.0);
        pos = pos + (k1x + k2x * 2 + k3x * 2 + k4x) * (1.0 / 6.0);

        // Update mass
        mass -= mdot * dt;
        if (stg_now >= 0 && stg_now < static_cast<int>(params.stages.size())) {
            if (mass < params.stages[stg_now].mass_final) {
                // Stage separation — drop empty stage mass
                double dropped = params.stages[stg_now].mass_initial - params.stages[stg_now].mass_final;
                // Mass already accounted for by mdot integration
                // Just ensure we don't go below the final mass of remaining stages
                double min_mass = 0;
                for (int si = stg_now + 1; si < static_cast<int>(params.stages.size()); si++) {
                    min_mass += params.stages[si].mass_initial;
                }
                if (mass < min_mass && min_mass > 0) mass = min_mass;
            }
        }

        if (mass <= 0) break;

        t += dt;
    }

    return traj;
}

} // namespace da_pipeline
