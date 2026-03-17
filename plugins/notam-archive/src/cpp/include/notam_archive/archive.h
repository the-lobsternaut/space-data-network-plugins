#pragma once
#include "notam_archive/types.h"
#include <optional>

namespace notam_archive {

class NotamArchive {
public:
    // Storage
    void add(const NOTAM& notam);
    void add_batch(const std::vector<NOTAM>& notams);
    void add_launch(const LaunchRecord& launch);

    // Queries
    std::optional<NOTAM> find_by_id(const std::string& id) const;
    std::vector<NOTAM> query_by_time(Timestamp start, Timestamp end) const;
    std::vector<NOTAM> query_by_location(double lat, double lon, double radius_nm) const;
    std::vector<NOTAM> query_by_classification(NotamClassification cls) const;
    std::vector<NOTAM> query_by_site(const std::string& location_id) const;
    std::vector<NOTAM> query_active_at(Timestamp time) const;

    // Pattern detection
    std::vector<NotamChange> detect_changes() const;
    NotamChange compare_notams(const NOTAM& old_n, const NOTAM& new_n) const;
    std::vector<NOTAM> find_replacements(const NOTAM& notam) const;
    bool is_scrub_indicator(const NotamChange& change) const;
    bool is_reschedule_indicator(const NotamChange& change) const;

    // Lead time tracking
    void record_lead_time(const std::string& notam_id, const LaunchRecord& launch);
    std::vector<LeadTimeRecord> get_lead_times() const;
    std::vector<LeadTimeRecord> get_lead_times_for_site(const std::string& site_id) const;

    // Accessors
    const std::vector<NOTAM>& get_all() const;
    const std::vector<LaunchRecord>& get_launches() const;
    size_t size() const;
    void clear();

private:
    std::vector<NOTAM> notams_;
    std::vector<LaunchRecord> launches_;
    std::vector<LeadTimeRecord> lead_times_;

    double haversine_nm(const LatLon& a, const LatLon& b) const;
    bool overlaps_time(const NOTAM& n, Timestamp start, Timestamp end) const;
    bool overlaps_location(const NOTAM& n, double lat, double lon, double radius_nm) const;
};

} // namespace notam_archive
