#pragma once
#include "da_pipeline/types.h"
#include "da_pipeline/notam_parser.h"
#include "da_pipeline/ntm_parser.h"
#include "da_pipeline/exclusion_zone.h"
#include "da_pipeline/sgp4.h"
#include "da_pipeline/tle_correlator.h"
#include "da_pipeline/trajectory.h"
#include "da_pipeline/ascent_estimator.h"
#include "da_pipeline/engagement.h"
#include <string>
#include <vector>

namespace da_pipeline {

class DomainAwarenessPipeline {
public:
    DomainAwarenessPipeline();

    // Step 1-2: Ingest and parse NOTAMs
    void ingest_notams(const std::string& raw_text);

    // Step 3: Ingest and parse NTMs
    void ingest_ntms(const std::string& raw_text);

    // Step 4: Ingest TLE catalog
    void ingest_tle_catalog(const std::string& catalog_text);

    // Step 5: Extract and group exclusion zones
    void analyze_exclusion_zones();

    // Step 6: Correlate zones to TLEs
    void correlate_tle();

    // Step 7: Generate ascent trajectory families (left-of-launch)
    void generate_trajectory_families();

    // Step 8: Accept tracklet, reconstruct (right-of-launch)
    void ingest_tracklet(const Tracklet& tracklet);

    // Step 9: Compute DA-ASAT engagement zones
    void assess_asat_threats();

    // Step 10: Generate report
    SituationalAwareness generate_report() const;

    // Run full pipeline
    SituationalAwareness run_full_pipeline(
        const std::string& notam_text,
        const std::string& ntm_text,
        const std::string& tle_catalog);

    // Configuration
    void set_analysis_window(Timestamp start, Timestamp end);
    void set_asat_interceptor(const InterceptorParams& params);

private:
    NotamParser notam_parser_;
    NtmParser ntm_parser_;
    ExclusionZoneAnalyzer zone_analyzer_;
    TLECorrelator tle_correlator_;
    TrajectorySimulator trajectory_sim_;
    AscentEstimator ascent_estimator_;
    EngagementAnalyzer engagement_analyzer_;

    // State
    std::vector<NOTAM> parsed_notams_;
    std::vector<MaritimeNotice> parsed_ntms_;
    std::vector<TLE> tle_catalog_;
    std::vector<ExclusionZoneGroup> zone_groups_;
    std::vector<Correlation> correlations_;
    std::vector<AscentTrajectoryFamily> trajectory_families_;
    std::vector<ReconstructionResult> reconstructions_;
    std::vector<ThreatAssessment> threat_assessments_;
    Timestamp window_start_ = 0;
    Timestamp window_end_ = 0;
    InterceptorParams interceptor_params_;

    std::string generate_summary() const;
};

} // namespace da_pipeline
