#pragma once
#include "notam_archive/archive.h"
#include <utility>

namespace notam_archive {

class NotamAnalyzer {
public:
    explicit NotamAnalyzer(const NotamArchive& archive);

    // Statistics
    ArchiveStatistics compute_statistics() const;
    SiteStatistics compute_site_stats(const std::string& site_id) const;
    VehicleStatistics compute_vehicle_stats(const std::string& vehicle) const;
    OrbitStatistics compute_orbit_stats(OrbitType orbit) const;

    // Analysis
    double average_lead_time(const std::vector<LeadTimeRecord>& records) const;
    double median_lead_time(const std::vector<LeadTimeRecord>& records) const;
    double stddev_lead_time(const std::vector<LeadTimeRecord>& records) const;

    // Trend analysis
    std::vector<std::pair<Timestamp, double>> lead_time_trend(
        const std::string& site_id, int bucket_days = 30) const;

    // Activity correlation
    int count_active_notams_at(Timestamp time) const;
    std::vector<std::pair<Timestamp, int>> activity_timeline(
        Timestamp start, Timestamp end, int step_hours = 1) const;

    // Anomaly detection
    struct Anomaly {
        std::string notam_id;
        std::string description;
        double severity = 0;  // 0-1
    };
    std::vector<Anomaly> detect_anomalies() const;

private:
    const NotamArchive& archive_;

    double percentile(std::vector<double>& values, double p) const;
};

} // namespace notam_archive
