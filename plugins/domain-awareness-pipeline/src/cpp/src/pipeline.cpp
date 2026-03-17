#include "da_pipeline/pipeline.h"
#include <chrono>
#include <sstream>
#include <algorithm>

namespace da_pipeline {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
DomainAwarenessPipeline::DomainAwarenessPipeline() {
    interceptor_params_ = EngagementAnalyzer::default_interceptor();

    // Default analysis window: now +/- 24 hours
    auto now = std::chrono::system_clock::now();
    auto epoch = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
    window_start_ = static_cast<Timestamp>(seconds) - 86400;
    window_end_ = static_cast<Timestamp>(seconds) + 86400;
}

// ---------------------------------------------------------------------------
// Step 1-2: Ingest NOTAMs
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::ingest_notams(const std::string& raw_text) {
    parsed_notams_ = notam_parser_.parse_batch(raw_text);
}

// ---------------------------------------------------------------------------
// Step 3: Ingest NTMs
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::ingest_ntms(const std::string& raw_text) {
    parsed_ntms_ = ntm_parser_.parse_batch(raw_text);
}

// ---------------------------------------------------------------------------
// Step 4: Ingest TLE catalog
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::ingest_tle_catalog(const std::string& catalog_text) {
    tle_catalog_ = SGP4::parse_catalog(catalog_text);
}

// ---------------------------------------------------------------------------
// Step 5: Analyze exclusion zones
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::analyze_exclusion_zones() {
    // Convert NOTAMs to exclusion zones
    std::vector<ExclusionZone> notam_zones;
    for (auto& n : parsed_notams_) {
        if (notam_parser_.is_launch_related(n)) {
            notam_zones.push_back(notam_parser_.notam_to_zone(n));
        }
    }

    // Collect NTM exclusion zones
    std::vector<ExclusionZone> ntm_zones;
    for (auto& m : parsed_ntms_) {
        if (m.zone.type != GeometryType::UNKNOWN) {
            ntm_zones.push_back(m.zone);
        }
    }

    // Group zones near known launch sites
    zone_groups_ = zone_analyzer_.group_zones(notam_zones, ntm_zones, {});
}

// ---------------------------------------------------------------------------
// Step 6: Correlate to TLEs
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::correlate_tle() {
    std::vector<ExclusionZone> all_zones;
    for (auto& group : zone_groups_) {
        for (auto& z : group.notam_zones) all_zones.push_back(z);
        for (auto& z : group.ntm_zones) all_zones.push_back(z);
    }

    correlations_ = tle_correlator_.batch_correlate(all_zones, tle_catalog_);
}

// ---------------------------------------------------------------------------
// Step 7: Generate trajectory families
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::generate_trajectory_families() {
    trajectory_families_.clear();

    for (auto& group : zone_groups_) {
        if (group.activity == ActivityType::LAUNCH_PREPARATION ||
            group.activity == ActivityType::LAUNCH_DETECTED) {
            auto family = ascent_estimator_.generate_family(
                group.associated_site, group.azimuth_deg);
            trajectory_families_.push_back(family);
        }
    }
}

// ---------------------------------------------------------------------------
// Step 8: Ingest tracklet (right-of-launch)
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::ingest_tracklet(const Tracklet& tracklet) {
    AscentModelParams guess;
    // Try to determine a reasonable guess from tracklet sensor location
    guess.launch_lat = tracklet.sensor_location.lat;
    guess.launch_lon = tracklet.sensor_location.lon;
    guess.launch_azimuth = 90.0; // Default eastward
    guess.initial_pitch = 90.0;

    // Use default vehicle
    guess = AscentEstimator::default_vehicle(
        tracklet.sensor_location.lat,
        tracklet.sensor_location.lon,
        90.0);

    auto result = ascent_estimator_.reconstruct(tracklet, guess);
    reconstructions_.push_back(result);
}

// ---------------------------------------------------------------------------
// Step 9: ASAT threat assessment
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::assess_asat_threats() {
    threat_assessments_.clear();

    for (auto& group : zone_groups_) {
        if (group.activity == ActivityType::ASAT_CONCERN ||
            group.activity == ActivityType::LAUNCH_DETECTED ||
            group.activity == ActivityType::LAUNCH_PREPARATION) {
            auto ta = engagement_analyzer_.assess_threat(
                group.associated_site, tle_catalog_,
                interceptor_params_, window_start_, window_end_);
            threat_assessments_.push_back(ta);
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
void DomainAwarenessPipeline::set_analysis_window(Timestamp start, Timestamp end) {
    window_start_ = start;
    window_end_ = end;
}

void DomainAwarenessPipeline::set_asat_interceptor(const InterceptorParams& params) {
    interceptor_params_ = params;
}

// ---------------------------------------------------------------------------
// generate_summary
// ---------------------------------------------------------------------------
std::string DomainAwarenessPipeline::generate_summary() const {
    std::ostringstream ss;
    ss << "=== Domain Awareness Situational Report ===\n";
    ss << "NOTAMs parsed: " << parsed_notams_.size() << "\n";
    ss << "NTMs parsed: " << parsed_ntms_.size() << "\n";
    ss << "TLEs in catalog: " << tle_catalog_.size() << "\n";
    ss << "Active zone groups: " << zone_groups_.size() << "\n";
    ss << "TLE correlations: " << correlations_.size() << "\n";
    ss << "Trajectory families: " << trajectory_families_.size() << "\n";
    ss << "Reconstructions: " << reconstructions_.size() << "\n";

    int total_threats = 0;
    ThreatLevel max_threat = ThreatLevel::LOW;
    for (auto& ta : threat_assessments_) {
        total_threats += static_cast<int>(ta.feasible_engagements.size());
        if (ta.level > max_threat) max_threat = ta.level;
    }
    ss << "ASAT threat assessments: " << threat_assessments_.size() << "\n";
    ss << "Total feasible engagements: " << total_threats << "\n";

    const char* threat_str[] = {"LOW", "MODERATE", "HIGH", "CRITICAL"};
    ss << "Overall threat level: " << threat_str[static_cast<int>(max_threat)] << "\n";

    // Active sites
    for (auto& group : zone_groups_) {
        const char* activity_str[] = {
            "LAUNCH_PREP", "LAUNCH_DETECTED", "REENTRY",
            "ASAT_CONCERN", "ROUTINE", "UNKNOWN"
        };
        ss << "\n  Site: " << group.associated_site.name
           << " (" << group.associated_site.site_id << ")"
           << " | Activity: " << activity_str[static_cast<int>(group.activity)]
           << " | Azimuth: " << group.azimuth_deg << " deg"
           << " | NOTAM zones: " << group.notam_zones.size()
           << " | NTM zones: " << group.ntm_zones.size();
    }

    // Top correlations
    if (!correlations_.empty()) {
        ss << "\n\nTop TLE Correlations:";
        int n = std::min(5, static_cast<int>(correlations_.size()));
        for (int i = 0; i < n; i++) {
            auto& c = correlations_[i];
            ss << "\n  " << c.object_name << " (NORAD " << c.norad_id << ")"
               << " | Zone: " << c.zone_id
               << " | Confidence: " << c.confidence
               << " | Min dist: " << c.min_distance_km << " km";
        }
    }

    ss << "\n=== End Report ===\n";
    return ss.str();
}

// ---------------------------------------------------------------------------
// generate_report
// ---------------------------------------------------------------------------
SituationalAwareness DomainAwarenessPipeline::generate_report() const {
    SituationalAwareness sa;

    auto now = std::chrono::system_clock::now();
    sa.generated_at = static_cast<Timestamp>(
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());
    sa.analysis_window_start = window_start_;
    sa.analysis_window_end = window_end_;

    sa.parsed_notams = parsed_notams_;
    sa.parsed_ntms = parsed_ntms_;
    sa.tle_catalog = tle_catalog_;
    sa.zone_groups = zone_groups_;
    sa.correlations = correlations_;
    sa.trajectory_families = trajectory_families_;
    sa.reconstructions = reconstructions_;
    sa.threat_assessments = threat_assessments_;

    // Summary stats
    sa.total_active_zones = 0;
    for (auto& g : zone_groups_) {
        sa.total_active_zones += static_cast<int>(g.notam_zones.size() + g.ntm_zones.size());
    }
    sa.launch_sites_active = static_cast<int>(zone_groups_.size());
    sa.tle_correlations_found = static_cast<int>(correlations_.size());

    sa.asat_threats_identified = 0;
    sa.overall_threat = ThreatLevel::LOW;
    for (auto& ta : threat_assessments_) {
        sa.asat_threats_identified += static_cast<int>(ta.feasible_engagements.size());
        if (ta.level > sa.overall_threat) sa.overall_threat = ta.level;
    }

    sa.summary_text = generate_summary();

    return sa;
}

// ---------------------------------------------------------------------------
// run_full_pipeline
// ---------------------------------------------------------------------------
SituationalAwareness DomainAwarenessPipeline::run_full_pipeline(
    const std::string& notam_text,
    const std::string& ntm_text,
    const std::string& tle_catalog)
{
    ingest_notams(notam_text);
    ingest_ntms(ntm_text);
    ingest_tle_catalog(tle_catalog);
    analyze_exclusion_zones();
    correlate_tle();
    generate_trajectory_families();
    assess_asat_threats();
    return generate_report();
}

} // namespace da_pipeline
