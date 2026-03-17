#include "notam_archive/analyzer.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <numeric>

namespace notam_archive {

NotamAnalyzer::NotamAnalyzer(const NotamArchive& archive) : archive_(archive) {}

// --- Statistical Helpers ---

double NotamAnalyzer::average_lead_time(const std::vector<LeadTimeRecord>& records) const {
    if (records.empty()) return 0.0;
    double sum = 0.0;
    for (const auto& r : records) {
        sum += r.lead_time_hours;
    }
    return sum / static_cast<double>(records.size());
}

double NotamAnalyzer::median_lead_time(const std::vector<LeadTimeRecord>& records) const {
    if (records.empty()) return 0.0;
    std::vector<double> values;
    values.reserve(records.size());
    for (const auto& r : records) {
        values.push_back(r.lead_time_hours);
    }
    std::sort(values.begin(), values.end());
    size_t n = values.size();
    if (n % 2 == 1) {
        return values[n / 2];
    } else {
        return (values[n / 2 - 1] + values[n / 2]) / 2.0;
    }
}

double NotamAnalyzer::stddev_lead_time(const std::vector<LeadTimeRecord>& records) const {
    if (records.size() < 2) return 0.0;
    double mean = average_lead_time(records);
    double sum_sq = 0.0;
    for (const auto& r : records) {
        double diff = r.lead_time_hours - mean;
        sum_sq += diff * diff;
    }
    return std::sqrt(sum_sq / static_cast<double>(records.size() - 1));
}

double NotamAnalyzer::percentile(std::vector<double>& values, double p) const {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    double index = p * static_cast<double>(values.size() - 1);
    size_t lo = static_cast<size_t>(std::floor(index));
    size_t hi = static_cast<size_t>(std::ceil(index));
    if (lo == hi) return values[lo];
    double frac = index - static_cast<double>(lo);
    return values[lo] * (1.0 - frac) + values[hi] * frac;
}

// --- Statistics ---

SiteStatistics NotamAnalyzer::compute_site_stats(const std::string& site_id) const {
    SiteStatistics stats;
    stats.site_id = site_id;

    // Count NOTAMs for this site
    const auto& all_notams = archive_.get_all();
    for (const auto& n : all_notams) {
        if (n.location_id == site_id) {
            stats.total_notams++;
            if (n.classification == NotamClassification::LAUNCH) stats.launch_notams++;
            if (n.classification == NotamClassification::HAZARD) stats.hazard_notams++;
        }
    }

    // Count launches and scrubs for this site
    const auto& launches = archive_.get_launches();
    for (const auto& l : launches) {
        if (l.site_id == site_id) {
            stats.total_launches++;
            if (l.scrubbed) stats.scrubbed_launches++;
        }
    }
    if (stats.total_launches > 0) {
        stats.scrub_rate = static_cast<double>(stats.scrubbed_launches) /
                           static_cast<double>(stats.total_launches);
    }

    // Lead time statistics
    auto lead_times = archive_.get_lead_times_for_site(site_id);
    if (!lead_times.empty()) {
        stats.avg_lead_time_hours = average_lead_time(lead_times);
        stats.median_lead_time_hours = median_lead_time(lead_times);
        stats.stddev_lead_time_hours = stddev_lead_time(lead_times);

        std::vector<double> values;
        for (const auto& lt : lead_times) values.push_back(lt.lead_time_hours);
        std::sort(values.begin(), values.end());
        stats.min_lead_time_hours = values.front();
        stats.max_lead_time_hours = values.back();
    }

    return stats;
}

VehicleStatistics NotamAnalyzer::compute_vehicle_stats(const std::string& vehicle) const {
    VehicleStatistics stats;
    stats.vehicle = vehicle;

    const auto& launches = archive_.get_launches();
    int total_notam_count = 0;
    for (const auto& l : launches) {
        if (l.vehicle == vehicle) {
            stats.total_launches++;
            total_notam_count += static_cast<int>(l.notam_ids.size());
            if (l.scrubbed) stats.scrubbed++;
        }
    }

    if (stats.total_launches > 0) {
        stats.scrub_rate = static_cast<double>(stats.scrubbed) /
                           static_cast<double>(stats.total_launches);
        stats.avg_notams_per_launch = static_cast<double>(total_notam_count) /
                                      static_cast<double>(stats.total_launches);
    }

    // Lead time stats for this vehicle
    auto all_lead_times = archive_.get_lead_times();
    std::vector<LeadTimeRecord> vehicle_lts;
    for (const auto& lt : all_lead_times) {
        if (lt.vehicle == vehicle) vehicle_lts.push_back(lt);
    }

    if (!vehicle_lts.empty()) {
        stats.avg_lead_time_hours = average_lead_time(vehicle_lts);
        stats.median_lead_time_hours = median_lead_time(vehicle_lts);
    }

    return stats;
}

OrbitStatistics NotamAnalyzer::compute_orbit_stats(OrbitType orbit) const {
    OrbitStatistics stats;
    stats.orbit = orbit;

    const auto& launches = archive_.get_launches();
    for (const auto& l : launches) {
        if (l.orbit == orbit) stats.total_launches++;
    }

    auto all_lead_times = archive_.get_lead_times();
    std::vector<LeadTimeRecord> orbit_lts;
    for (const auto& lt : all_lead_times) {
        if (lt.orbit == orbit) orbit_lts.push_back(lt);
    }

    if (!orbit_lts.empty()) {
        stats.avg_lead_time_hours = average_lead_time(orbit_lts);
        stats.median_lead_time_hours = median_lead_time(orbit_lts);
    }

    return stats;
}

ArchiveStatistics NotamAnalyzer::compute_statistics() const {
    ArchiveStatistics stats;
    const auto& all_notams = archive_.get_all();
    const auto& launches = archive_.get_launches();
    auto changes = archive_.detect_changes();
    auto all_lead_times = archive_.get_lead_times();

    stats.total_notams = static_cast<int>(all_notams.size());
    stats.total_launches = static_cast<int>(launches.size());
    stats.total_changes = static_cast<int>(changes.size());

    // Time range
    if (!all_notams.empty()) {
        stats.earliest_notam = all_notams[0].issued;
        stats.latest_notam = all_notams[0].issued;
        for (const auto& n : all_notams) {
            if (n.issued < stats.earliest_notam) stats.earliest_notam = n.issued;
            if (n.issued > stats.latest_notam) stats.latest_notam = n.issued;
        }
    }

    // Overall lead time
    if (!all_lead_times.empty()) {
        stats.overall_avg_lead_time_hours = average_lead_time(all_lead_times);
    }

    // Overall scrub rate
    int total_scrubs = 0;
    for (const auto& l : launches) {
        if (l.scrubbed) total_scrubs++;
    }
    if (!launches.empty()) {
        stats.overall_scrub_rate = static_cast<double>(total_scrubs) /
                                   static_cast<double>(launches.size());
    }

    // Collect unique sites, vehicles, orbit types
    std::set<std::string> sites;
    std::set<std::string> vehicles;
    std::set<OrbitType> orbits;

    for (const auto& n : all_notams) {
        sites.insert(n.location_id);
    }
    for (const auto& l : launches) {
        vehicles.insert(l.vehicle);
        orbits.insert(l.orbit);
    }

    for (const auto& s : sites) {
        stats.by_site.push_back(compute_site_stats(s));
    }
    for (const auto& v : vehicles) {
        stats.by_vehicle.push_back(compute_vehicle_stats(v));
    }
    for (const auto& o : orbits) {
        stats.by_orbit.push_back(compute_orbit_stats(o));
    }

    return stats;
}

// --- Trend Analysis ---

std::vector<std::pair<Timestamp, double>> NotamAnalyzer::lead_time_trend(
    const std::string& site_id, int bucket_days) const {

    auto lead_times = archive_.get_lead_times_for_site(site_id);
    if (lead_times.empty()) return {};

    // Sort by launch time
    std::sort(lead_times.begin(), lead_times.end(),
        [](const LeadTimeRecord& a, const LeadTimeRecord& b) {
            return a.launch_time < b.launch_time;
        });

    Timestamp bucket_size = static_cast<Timestamp>(bucket_days) * 86400;
    Timestamp bucket_start = lead_times.front().launch_time;

    std::vector<std::pair<Timestamp, double>> trend;
    std::vector<LeadTimeRecord> current_bucket;

    for (const auto& lt : lead_times) {
        if (lt.launch_time >= bucket_start + bucket_size) {
            // Emit current bucket
            if (!current_bucket.empty()) {
                trend.push_back({bucket_start, average_lead_time(current_bucket)});
            }
            bucket_start = lt.launch_time;
            current_bucket.clear();
        }
        current_bucket.push_back(lt);
    }
    // Emit last bucket
    if (!current_bucket.empty()) {
        trend.push_back({bucket_start, average_lead_time(current_bucket)});
    }

    return trend;
}

// --- Activity Correlation ---

int NotamAnalyzer::count_active_notams_at(Timestamp time) const {
    int count = 0;
    for (const auto& n : archive_.get_all()) {
        if (n.effective_start <= time && time <= n.effective_end) {
            count++;
        }
    }
    return count;
}

std::vector<std::pair<Timestamp, int>> NotamAnalyzer::activity_timeline(
    Timestamp start, Timestamp end, int step_hours) const {

    std::vector<std::pair<Timestamp, int>> timeline;
    Timestamp step = static_cast<Timestamp>(step_hours) * 3600;

    for (Timestamp t = start; t <= end; t += step) {
        timeline.push_back({t, count_active_notams_at(t)});
    }

    return timeline;
}

// --- Anomaly Detection ---

std::vector<NotamAnalyzer::Anomaly> NotamAnalyzer::detect_anomalies() const {
    std::vector<Anomaly> anomalies;
    auto all_lead_times = archive_.get_lead_times();

    if (all_lead_times.size() >= 3) {
        // Detect unusual lead times (>3 sigma from mean)
        double mean = average_lead_time(all_lead_times);
        double sd = stddev_lead_time(all_lead_times);

        if (sd > 0.0) {
            for (const auto& lt : all_lead_times) {
                double z = std::abs(lt.lead_time_hours - mean) / sd;
                if (z > 3.0) {
                    Anomaly a;
                    a.notam_id = lt.notam_id;
                    a.description = "Unusual lead time: " +
                        std::to_string(lt.lead_time_hours) + "h (z-score=" +
                        std::to_string(z) + ")";
                    a.severity = std::min(1.0, z / 5.0);
                    anomalies.push_back(a);
                }
            }
        }
    }

    // Detect unusual NOTAM counts per launch
    const auto& launches = archive_.get_launches();
    if (launches.size() >= 3) {
        double sum = 0.0;
        for (const auto& l : launches) {
            sum += static_cast<double>(l.notam_ids.size());
        }
        double mean_count = sum / static_cast<double>(launches.size());

        double sq_sum = 0.0;
        for (const auto& l : launches) {
            double diff = static_cast<double>(l.notam_ids.size()) - mean_count;
            sq_sum += diff * diff;
        }
        double sd_count = std::sqrt(sq_sum / static_cast<double>(launches.size() - 1));

        if (sd_count > 0.0) {
            for (const auto& l : launches) {
                double z = std::abs(static_cast<double>(l.notam_ids.size()) - mean_count) / sd_count;
                if (z > 3.0) {
                    Anomaly a;
                    a.notam_id = l.launch_id;
                    a.description = "Unusual NOTAM count for launch: " +
                        std::to_string(l.notam_ids.size()) + " NOTAMs";
                    a.severity = std::min(1.0, z / 5.0);
                    anomalies.push_back(a);
                }
            }
        }
    }

    // Detect overlapping NOTAMs for same site (potential conflicts)
    const auto& all_notams = archive_.get_all();
    std::map<std::string, std::vector<size_t>> by_site;
    for (size_t i = 0; i < all_notams.size(); ++i) {
        by_site[all_notams[i].location_id].push_back(i);
    }

    for (const auto& [site, indices] : by_site) {
        for (size_t i = 0; i < indices.size(); ++i) {
            int overlap_count = 0;
            for (size_t j = 0; j < indices.size(); ++j) {
                if (i == j) continue;
                const auto& a = all_notams[indices[i]];
                const auto& b = all_notams[indices[j]];
                if (a.effective_start <= b.effective_end &&
                    b.effective_start <= a.effective_end) {
                    overlap_count++;
                }
            }
            if (overlap_count >= 3) {
                Anomaly anom;
                anom.notam_id = all_notams[indices[i]].notam_id;
                anom.description = "NOTAM overlaps with " +
                    std::to_string(overlap_count) + " other NOTAMs at site " + site;
                anom.severity = std::min(1.0, static_cast<double>(overlap_count) / 5.0);
                anomalies.push_back(anom);
            }
        }
    }

    return anomalies;
}

} // namespace notam_archive
