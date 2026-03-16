/**
 * @file types.h
 * @brief Core data types for all DRAMA modules
 *
 * Types follow ESA DRAMA nomenclature and the Space Data Standards.
 */
#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace drama {

// ════════════════════════════════════════════════════════════════════════════
// Common orbital types
// ════════════════════════════════════════════════════════════════════════════

struct KeplerianOrbit {
    double semi_major_axis_m;       // a [m]
    double eccentricity;            // e
    double inclination_rad;         // i [rad]
    double raan_rad;                // Ω [rad]
    double arg_perigee_rad;         // ω [rad]
    double true_anomaly_rad;        // ν [rad]

    // Convenience: altitude above Earth surface
    double perigee_altitude_km() const;
    double apogee_altitude_km() const;
    double mean_altitude_km() const;
    double period_seconds() const;
};

struct SatelliteParams {
    double mass_kg;                 // spacecraft dry mass
    double cross_section_m2;        // effective cross-section for drag
    double cd;                      // drag coefficient (typically 2.2)
    double cr;                      // reflectivity coefficient (typically 1.3)

    double ballistic_coefficient() const { return mass_kg / (cd * cross_section_m2); }
};

enum class SolarActivityModel {
    CONSTANT,          // fixed F10.7 and Ap
    MEAN_CYCLE,        // 11-year sinusoidal mean cycle
    LOW_ACTIVITY,      // solar minimum throughout
    HIGH_ACTIVITY      // solar maximum throughout
};

struct SolarActivity {
    double f107;       // 10.7 cm radio flux [sfu]
    double f107a;      // 81-day average F10.7
    double ap;         // daily geomagnetic Ap index
};

struct EpochTime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    double second;

    double to_mjd() const;
    double to_jd() const;
    static EpochTime from_mjd(double mjd);
    double fractional_year() const;
};

// ════════════════════════════════════════════════════════════════════════════
// OSCAR — Orbital Lifetime
// ════════════════════════════════════════════════════════════════════════════

struct LifetimeConfig {
    KeplerianOrbit initial_orbit;
    SatelliteParams satellite;
    SolarActivityModel solar_model;
    SolarActivity solar_override;    // used when solar_model == CONSTANT
    EpochTime epoch;
    double max_lifetime_years;       // simulation cap (default 200)
    double time_step_days;           // integration step (default 1.0)
};

struct AltitudeRecord {
    double time_years;
    double perigee_km;
    double apogee_km;
    double eccentricity;
};

struct LifetimeResult {
    double lifetime_years;
    EpochTime decay_date;
    bool compliant_25yr;
    std::vector<AltitudeRecord> history;
    std::string notes;
};

// ════════════════════════════════════════════════════════════════════════════
// ARES — Collision Risk / Flux
// ════════════════════════════════════════════════════════════════════════════

struct FluxConfig {
    KeplerianOrbit orbit;
    EpochTime epoch;
    double duration_years;
    double min_diameter_m;           // minimum debris size threshold
};

struct FluxBySize {
    double diameter_m;
    double flux_per_m2_yr;           // impacts/m²/yr
};

struct FluxResult {
    double total_debris_flux;        // total debris impacts/m²/yr
    double total_meteoroid_flux;     // total meteoroid impacts/m²/yr
    std::vector<FluxBySize> debris_flux_by_size;
    std::vector<FluxBySize> meteoroid_flux_by_size;
    double collision_probability_per_year;
    double cumulative_collision_probability;
};

// ════════════════════════════════════════════════════════════════════════════
// CROC — Cross-Section
// ════════════════════════════════════════════════════════════════════════════

enum class PrimitiveType {
    BOX,
    CYLINDER,
    SPHERE,
    FLAT_PANEL
};

struct GeometricPrimitive {
    PrimitiveType type;
    double dim_x_m;                  // length / diameter
    double dim_y_m;                  // width / height
    double dim_z_m;                  // depth (box only)
    double offset_x_m;              // position offset from body center
    double offset_y_m;
    double offset_z_m;
};

struct SpacecraftGeometry {
    std::vector<GeometricPrimitive> primitives;
    std::string description;
};

struct AttitudeDirection {
    double az_rad;                   // azimuth
    double el_rad;                   // elevation
};

struct CrossSectionAtAttitude {
    AttitudeDirection attitude;
    double area_m2;
};

struct CrossSectionResult {
    double average_area_m2;          // random tumbling average
    double max_area_m2;
    double min_area_m2;
    std::vector<CrossSectionAtAttitude> area_vs_attitude;
};

// ════════════════════════════════════════════════════════════════════════════
// SARA — Re-entry Survival
// ════════════════════════════════════════════════════════════════════════════

enum class MaterialType {
    ALUMINUM,
    STEEL,
    TITANIUM,
    CFRP,           // carbon fiber reinforced polymer
    COPPER,
    GLASS,
    INCONEL,
    BERYLLIUM,
    STAINLESS_STEEL,
    GENERIC
};

struct MaterialProperties {
    MaterialType type;
    double density_kg_m3;
    double specific_heat_j_kg_k;
    double melting_point_k;
    double heat_of_fusion_j_kg;
    double emissivity;
    std::string name;

    static MaterialProperties get(MaterialType type);
};

struct SpacecraftComponent {
    std::string name;
    MaterialType material;
    double mass_kg;
    double characteristic_length_m;  // diameter or equivalent
    GeometricPrimitive shape;
};

struct ReentryConfig {
    KeplerianOrbit initial_orbit;
    double total_mass_kg;
    std::vector<SpacecraftComponent> components;
    SpacecraftGeometry geometry;
    EpochTime epoch;
    double breakup_altitude_km;      // default 78 km
    double ground_impact_velocity_threshold_ms; // min to count as surviving
};

struct SurvivingFragment {
    std::string component_name;
    MaterialType material;
    double mass_kg;
    double area_m2;                  // casualty area
    double impact_energy_j;
    double impact_velocity_ms;
    bool demised;                    // true if fragment demised during reentry
    double demise_altitude_km;       // altitude where demise occurred (if demised)
    double peak_heating_kw_m2;
};

struct ReentryResult {
    std::vector<SurvivingFragment> fragments;
    double total_surviving_mass_kg;
    double casualty_area_m2;
    double casualty_expectation;
    bool compliant_1e4;              // casualty_expectation < 1e-4
    double ground_footprint_km2;
    std::string notes;
};

// ════════════════════════════════════════════════════════════════════════════
// MASTER — Space Debris Environment
// ════════════════════════════════════════════════════════════════════════════

struct EnvironmentConfig {
    EpochTime epoch;
    double altitude_min_km;
    double altitude_max_km;
    double altitude_step_km;
    double inclination_min_deg;
    double inclination_max_deg;
    double size_min_m;               // minimum debris diameter
    double size_max_m;
};

struct PopulationBin {
    double altitude_km;
    double spatial_density;          // objects/km³
    double flux;                     // impacts/m²/yr
};

struct SizeDistribution {
    double diameter_m;
    double cumulative_count;         // objects with d >= diameter
};

struct EnvironmentResult {
    std::vector<PopulationBin> debris_by_altitude;
    std::vector<PopulationBin> meteoroid_by_altitude;
    std::vector<SizeDistribution> debris_size_distribution;
    std::vector<SizeDistribution> meteoroid_size_distribution;
    double total_debris_objects;
    double total_meteoroid_flux;     // at reference altitude
    EpochTime epoch;
};

// ════════════════════════════════════════════════════════════════════════════
// Compliance
// ════════════════════════════════════════════════════════════════════════════

enum class ComplianceGuideline {
    IADC_DEBRIS_MITIGATION,
    ISO_24113,
    ESA_DEBRIS_POLICY,
    FCC_25_YEAR,
    FRENCH_SPACE_OPS_ACT
};

struct GuidelineResult {
    ComplianceGuideline guideline;
    std::string name;
    bool passed;
    std::string details;
    std::vector<std::string> findings;
};

struct ComplianceConfig {
    KeplerianOrbit orbit;
    SatelliteParams satellite;
    SpacecraftGeometry geometry;
    std::vector<SpacecraftComponent> components;
    EpochTime epoch;
    SolarActivityModel solar_model;
    double mission_duration_years;
    double post_mission_disposal_years; // planned PMD duration
};

struct ComplianceReport {
    bool all_passed;
    std::vector<GuidelineResult> results;
    LifetimeResult lifetime;
    FluxResult flux;
    CrossSectionResult cross_section;
    ReentryResult reentry;
    std::string summary;
};

} // namespace drama
