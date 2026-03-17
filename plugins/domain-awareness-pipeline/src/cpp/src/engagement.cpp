#include "da_pipeline/engagement.h"
#include "da_pipeline/sgp4.h"
#include "da_pipeline/trajectory.h"
#include "da_pipeline/exclusion_zone.h"
#include <cmath>
#include <algorithm>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// default_interceptor — generic 3-stage DA-ASAT
// ---------------------------------------------------------------------------
InterceptorParams EngagementAnalyzer::default_interceptor() {
    InterceptorParams p;
    p.vehicle_name = "Generic DA-ASAT";
    p.num_stages = 3;

    // Stage 1: solid booster
    p.stage_thrust_kn = {600.0, 300.0, 50.0};
    p.stage_burn_time_sec = {60.0, 45.0, 30.0};
    p.stage_mass_kg = {12000.0, 4000.0, 800.0};
    p.stage_propellant_kg = {10000.0, 3200.0, 600.0};

    p.kill_vehicle_mass_kg = 50.0;
    p.max_lateral_accel_g = 15.0;

    return p;
}

// ---------------------------------------------------------------------------
// max_engagement_altitude
// ---------------------------------------------------------------------------
double EngagementAnalyzer::max_engagement_altitude(const InterceptorParams& params) const {
    // Estimate max altitude from total delta-V (Tsiolkovsky)
    double total_dv = 0;
    double mass_remaining = 0;
    for (int i = static_cast<int>(params.stage_mass_kg.size()) - 1; i >= 0; i--) {
        double m_full = params.stage_mass_kg[i] + mass_remaining + params.kill_vehicle_mass_kg;
        double m_empty = m_full - params.stage_propellant_kg[i];
        if (m_empty <= 0) continue;
        double isp = params.stage_thrust_kn[i] * 1000.0 * params.stage_burn_time_sec[i] /
                     (params.stage_propellant_kg[i] * G0);
        total_dv += isp * G0 * std::log(m_full / m_empty);
        mass_remaining += params.stage_mass_kg[i];
    }

    // Simplified: max altitude ≈ v^2 / (2g) for vertical launch
    double v = total_dv;
    double max_alt_m = v * v / (2.0 * G0);
    return max_alt_m / 1000.0; // km
}

// ---------------------------------------------------------------------------
// evaluate_single — single engagement assessment
// ---------------------------------------------------------------------------
EngagementResult EngagementAnalyzer::evaluate_single(
    const LaunchSite& site, const TLE& target,
    const InterceptorParams& params, Timestamp launch_time) const
{
    EngagementResult result;
    result.target_norad_id = target.norad_id;
    result.target_name = target.name;
    result.launch_site_id = site.site_id;
    result.launch_time = launch_time;

    // Propagate target to launch_time + flight time window
    OrbitalElements elems = SGP4::tle_to_elements(target);
    Timestamp epoch_unix = SGP4::tle_epoch_to_unix(target.epoch_year, target.epoch_day);

    // Target altitude
    double tsince = static_cast<double>(launch_time - epoch_unix) / 60.0;
    Vec3 target_pos, target_vel;
    SGP4::propagate(elems, tsince, target_pos, target_vel);
    double target_alt_km = target_pos.norm() - R_EARTH_KM;

    // Check altitude feasibility
    double max_alt = max_engagement_altitude(params);
    if (target_alt_km > max_alt) {
        result.feasibility = Feasibility::ALTITUDE_TOO_HIGH;
        return result;
    }
    if (target_alt_km < 150) {
        result.feasibility = Feasibility::ALTITUDE_TOO_LOW;
        return result;
    }

    // Find closest approach within a flight time window
    Vec3 site_ecef = TrajectorySimulator::geodetic_to_ecef(site.lat, site.lon, 0);
    Vec3 site_eci_approx = site_ecef * (1.0 / 1000.0); // Rough km conversion (not rotating, simplified)

    double best_range_km = 1e9;
    double best_tof = 0;
    Vec3 best_target_pos, best_target_vel;

    // Scan target overhead passes (within max slant range)
    // Max slant range ~ max_alt for direct ascent
    double max_range_km = max_alt * 1.5;

    for (double dt_sec = 0; dt_sec < 3600; dt_sec += 10.0) {
        double t_min = tsince + dt_sec / 60.0;
        Vec3 tp, tv;
        SGP4::propagate(elems, t_min, tp, tv);

        // Approximate range from site (ignoring Earth rotation precisely)
        // Site ECI position at this time
        double gmst = SGP4::gmst_from_jd(2440587.5 + static_cast<double>(launch_time + dt_sec) / SEC_PER_DAY);
        double site_lon_eci = site.lon * DEG2RAD + gmst;
        double site_lat_r = site.lat * DEG2RAD;
        Vec3 site_eci = {
            R_EARTH_KM * std::cos(site_lat_r) * std::cos(site_lon_eci),
            R_EARTH_KM * std::cos(site_lat_r) * std::sin(site_lon_eci),
            R_EARTH_KM * std::sin(site_lat_r)
        };

        double range = (tp - site_eci).norm();
        if (range < best_range_km) {
            best_range_km = range;
            best_tof = dt_sec;
            best_target_pos = tp;
            best_target_vel = tv;
        }
    }

    if (best_range_km > max_range_km) {
        result.feasibility = Feasibility::GEOMETRY_INFEASIBLE;
        return result;
    }

    // Compute required delta-V for direct ascent intercept
    // Simplified: energy to reach altitude + lateral velocity match
    double alt_km = best_target_pos.norm() - R_EARTH_KM;

    // Vertical delta-V to reach altitude
    double dv_vertical = std::sqrt(2.0 * MU_EARTH * (1.0 / R_EARTH_KM - 1.0 / (R_EARTH_KM + alt_km)));

    // Lateral delta-V (need to match some target velocity component)
    double dv_lateral = best_range_km * 0.01; // Simplified approximation

    double total_dv = std::sqrt(dv_vertical * dv_vertical + dv_lateral * dv_lateral);

    // Check if interceptor has enough delta-V
    double interceptor_dv = 0;
    double mass_above = params.kill_vehicle_mass_kg;
    for (int i = params.num_stages - 1; i >= 0; i--) {
        if (i >= static_cast<int>(params.stage_mass_kg.size())) continue;
        double m_full = params.stage_mass_kg[i] + mass_above;
        double m_dry = m_full - params.stage_propellant_kg[i];
        if (m_dry <= 0) continue;
        double isp_eff = params.stage_thrust_kn[i] * 1000.0 * params.stage_burn_time_sec[i] /
                         (params.stage_propellant_kg[i] * G0);
        interceptor_dv += isp_eff * G0 * std::log(m_full / m_dry) / 1000.0; // km/s
        mass_above = m_full;
    }

    if (total_dv > interceptor_dv) {
        result.feasibility = Feasibility::INSUFFICIENT_ENERGY;
        result.total_delta_v_kms = total_dv;
        return result;
    }

    // Feasible engagement
    result.feasibility = Feasibility::FEASIBLE;
    result.total_delta_v_kms = total_dv;
    result.time_of_flight_sec = best_tof;
    result.intercept_time = launch_time + static_cast<Timestamp>(best_tof);

    // Launch geometry
    // Azimuth to target
    ExclusionZoneAnalyzer ez;
    double jd = 2440587.5 + static_cast<double>(launch_time + best_tof) / SEC_PER_DAY;
    double gmst = SGP4::gmst_from_jd(jd);
    LatLon target_ground = SGP4::eci_to_geodetic(best_target_pos, gmst);
    LatLon site_ll = {site.lat, site.lon};
    result.launch_azimuth_deg = ez.bearing_deg(site_ll, target_ground);

    // Elevation angle
    double elev = std::atan2(alt_km, best_range_km - alt_km) * RAD2DEG;
    result.launch_elevation_deg = std::max(10.0, elev);

    // Intercept point
    result.intercept.position.lat = target_ground.lat;
    result.intercept.position.lon = target_ground.lon;
    result.intercept.position.alt = alt_km * 1000.0;
    result.intercept.altitude_km = alt_km;
    result.intercept.range_km = best_range_km;
    result.intercept.time_of_flight_sec = best_tof;
    result.intercept.target_vel = best_target_vel;
    result.intercept.closing_velocity_kms = best_target_vel.norm() + dv_vertical * 0.5;
    result.intercept.miss_distance_km = 0.05; // Optimistic

    // Confidence
    result.confidence = std::max(0.0, std::min(1.0, 1.0 - total_dv / interceptor_dv));

    return result;
}

// ---------------------------------------------------------------------------
// evaluate — multiple targets
// ---------------------------------------------------------------------------
std::vector<EngagementResult> EngagementAnalyzer::evaluate(
    const LaunchSite& site, const std::vector<TLE>& targets,
    const InterceptorParams& params,
    Timestamp window_start, Timestamp window_end, double step_sec) const
{
    std::vector<EngagementResult> results;

    for (auto& target : targets) {
        // Sample launch times across window
        EngagementResult best;
        best.feasibility = Feasibility::GEOMETRY_INFEASIBLE;
        double best_dv = 1e9;

        for (Timestamp t = window_start; t <= window_end; t += static_cast<Timestamp>(step_sec)) {
            auto er = evaluate_single(site, target, params, t);
            if (er.feasibility == Feasibility::FEASIBLE && er.total_delta_v_kms < best_dv) {
                best = er;
                best_dv = er.total_delta_v_kms;
            }
        }

        results.push_back(best);
    }

    return results;
}

// ---------------------------------------------------------------------------
// find_windows
// ---------------------------------------------------------------------------
std::vector<EngagementWindow> EngagementAnalyzer::find_windows(
    const LaunchSite& site, const TLE& target,
    const InterceptorParams& params,
    Timestamp start, Timestamp end) const
{
    std::vector<EngagementWindow> windows;

    bool in_window = false;
    EngagementWindow current;
    current.min_delta_v_kms = 1e9;

    Timestamp step = 60; // 1-minute steps
    for (Timestamp t = start; t <= end; t += step) {
        auto er = evaluate_single(site, target, params, t);

        if (er.feasibility == Feasibility::FEASIBLE) {
            if (!in_window) {
                in_window = true;
                current.window_start = t;
                current.min_delta_v_kms = 1e9;
            }
            current.window_end = t;
            if (er.total_delta_v_kms < current.min_delta_v_kms) {
                current.min_delta_v_kms = er.total_delta_v_kms;
                current.best_launch_azimuth_deg = er.launch_azimuth_deg;
                current.best_engagement = er;
            }
        } else {
            if (in_window) {
                windows.push_back(current);
                in_window = false;
                current = EngagementWindow();
                current.min_delta_v_kms = 1e9;
            }
        }
    }

    if (in_window) {
        windows.push_back(current);
    }

    return windows;
}

// ---------------------------------------------------------------------------
// assess_threat
// ---------------------------------------------------------------------------
ThreatAssessment EngagementAnalyzer::assess_threat(
    const LaunchSite& site, const std::vector<TLE>& catalog,
    const InterceptorParams& params,
    Timestamp window_start, Timestamp window_end) const
{
    ThreatAssessment ta;
    ta.site_id = site.site_id;

    auto results = evaluate(site, catalog, params, window_start, window_end, 300.0);

    int feasible_count = 0;
    for (auto& er : results) {
        if (er.feasibility == Feasibility::FEASIBLE) {
            ta.feasible_engagements.push_back(er);
            ta.threatened_norad_ids.push_back(er.target_norad_id);
            feasible_count++;
        }
    }

    // Determine threat level
    if (feasible_count == 0) {
        ta.level = ThreatLevel::LOW;
        ta.rationale = "No feasible engagements from " + site.name;
    } else if (feasible_count <= 2) {
        ta.level = ThreatLevel::MODERATE;
        ta.rationale = std::to_string(feasible_count) +
                       " target(s) within engagement envelope from " + site.name;
    } else if (feasible_count <= 5) {
        ta.level = ThreatLevel::HIGH;
        ta.rationale = std::to_string(feasible_count) +
                       " targets within engagement envelope from " + site.name +
                       "; multiple engagement windows available";
    } else {
        ta.level = ThreatLevel::CRITICAL;
        ta.rationale = std::to_string(feasible_count) +
                       " targets threatened from " + site.name +
                       "; broad engagement capability";
    }

    return ta;
}

} // namespace da_pipeline
