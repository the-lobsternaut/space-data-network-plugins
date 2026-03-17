#include "notam_archive/archive.h"
#include <algorithm>
#include <cmath>

namespace notam_archive {

// --- Storage ---

void NotamArchive::add(const NOTAM& notam) {
    notams_.push_back(notam);
}

void NotamArchive::add_batch(const std::vector<NOTAM>& notams) {
    notams_.insert(notams_.end(), notams.begin(), notams.end());
}

void NotamArchive::add_launch(const LaunchRecord& launch) {
    launches_.push_back(launch);
}

// --- Queries ---

std::optional<NOTAM> NotamArchive::find_by_id(const std::string& id) const {
    for (const auto& n : notams_) {
        if (n.notam_id == id) return n;
    }
    return std::nullopt;
}

std::vector<NOTAM> NotamArchive::query_by_time(Timestamp start, Timestamp end) const {
    std::vector<NOTAM> result;
    for (const auto& n : notams_) {
        if (overlaps_time(n, start, end)) {
            result.push_back(n);
        }
    }
    return result;
}

std::vector<NOTAM> NotamArchive::query_by_location(double lat, double lon, double radius_nm) const {
    std::vector<NOTAM> result;
    for (const auto& n : notams_) {
        if (overlaps_location(n, lat, lon, radius_nm)) {
            result.push_back(n);
        }
    }
    return result;
}

std::vector<NOTAM> NotamArchive::query_by_classification(NotamClassification cls) const {
    std::vector<NOTAM> result;
    for (const auto& n : notams_) {
        if (n.classification == cls) {
            result.push_back(n);
        }
    }
    return result;
}

std::vector<NOTAM> NotamArchive::query_by_site(const std::string& location_id) const {
    std::vector<NOTAM> result;
    for (const auto& n : notams_) {
        if (n.location_id == location_id) {
            result.push_back(n);
        }
    }
    return result;
}

std::vector<NOTAM> NotamArchive::query_active_at(Timestamp time) const {
    std::vector<NOTAM> result;
    for (const auto& n : notams_) {
        if (n.effective_start <= time && time <= n.effective_end) {
            result.push_back(n);
        }
    }
    return result;
}

// --- Pattern Detection ---

std::vector<NotamChange> NotamArchive::detect_changes() const {
    std::vector<NotamChange> changes;

    // Group NOTAMs by location_id
    std::map<std::string, std::vector<size_t>> by_location;
    for (size_t i = 0; i < notams_.size(); ++i) {
        by_location[notams_[i].location_id].push_back(i);
    }

    for (const auto& [loc, indices] : by_location) {
        if (indices.size() < 2) continue;

        // Sort by issued time
        std::vector<size_t> sorted_indices = indices;
        std::sort(sorted_indices.begin(), sorted_indices.end(),
            [this](size_t a, size_t b) {
                return notams_[a].issued < notams_[b].issued;
            });

        // Compare consecutive NOTAMs for same location
        for (size_t i = 0; i + 1 < sorted_indices.size(); ++i) {
            const auto& old_n = notams_[sorted_indices[i]];
            const auto& new_n = notams_[sorted_indices[i + 1]];

            // Check if time windows overlap or are close (within 7 days)
            bool time_related =
                (old_n.effective_start <= new_n.effective_end &&
                 new_n.effective_start <= old_n.effective_end) ||
                (std::abs(old_n.effective_start - new_n.effective_start) < 7 * 86400);

            if (!time_related) continue;

            // Check if geometry is similar (centers within 50nm)
            LatLon a{old_n.center_lat, old_n.center_lon};
            LatLon b{new_n.center_lat, new_n.center_lon};
            double dist = haversine_nm(a, b);
            if (dist > 50.0) continue;

            NotamChange change = compare_notams(old_n, new_n);
            changes.push_back(change);
        }
    }

    return changes;
}

NotamChange NotamArchive::compare_notams(const NOTAM& old_n, const NOTAM& new_n) const {
    NotamChange change;
    change.old_notam_id = old_n.notam_id;
    change.new_notam_id = new_n.notam_id;
    change.detected_at = new_n.issued;
    change.old_start = old_n.effective_start;
    change.old_end = old_n.effective_end;
    change.new_start = new_n.effective_start;
    change.new_end = new_n.effective_end;

    Timestamp old_duration = old_n.effective_end - old_n.effective_start;
    Timestamp new_duration = new_n.effective_end - new_n.effective_start;

    // If new NOTAM has zero or negative duration, it is a cancellation
    if (new_duration <= 0) {
        change.change_type = ChangeType::CANCELLATION;
        change.description = "NOTAM cancelled (zero/negative effective window)";
        return change;
    }

    // If new end is later than old end and start is similar
    if (new_n.effective_end > old_n.effective_end &&
        std::abs(new_n.effective_start - old_n.effective_start) < 3600) {
        change.change_type = ChangeType::EXTENSION;
        change.description = "Effective window extended by " +
            std::to_string((new_n.effective_end - old_n.effective_end) / 3600) + " hours";
        return change;
    }

    // If new window is shorter (narrowed)
    if (new_duration < old_duration && new_duration < old_duration * 0.8) {
        change.change_type = ChangeType::NARROWING;
        change.description = "Effective window narrowed from " +
            std::to_string(old_duration / 3600) + "h to " +
            std::to_string(new_duration / 3600) + "h";
        return change;
    }

    // Otherwise it is a replacement
    change.change_type = ChangeType::REPLACEMENT;
    change.description = "NOTAM replaced with new time window";
    return change;
}

std::vector<NOTAM> NotamArchive::find_replacements(const NOTAM& notam) const {
    std::vector<NOTAM> result;
    for (const auto& n : notams_) {
        if (n.notam_id == notam.notam_id) continue;
        if (n.location_id != notam.location_id) continue;
        // Check overlapping time windows
        if (n.effective_start <= notam.effective_end &&
            notam.effective_start <= n.effective_end) {
            result.push_back(n);
        }
    }
    return result;
}

bool NotamArchive::is_scrub_indicator(const NotamChange& change) const {
    return change.change_type == ChangeType::NARROWING ||
           change.change_type == ChangeType::CANCELLATION;
}

bool NotamArchive::is_reschedule_indicator(const NotamChange& change) const {
    if (change.change_type == ChangeType::EXTENSION) return true;
    // Time shift > 12 hours
    Timestamp start_shift = std::abs(change.new_start - change.old_start);
    if (start_shift > 12 * 3600) return true;
    return false;
}

// --- Lead Time Tracking ---

void NotamArchive::record_lead_time(const std::string& notam_id, const LaunchRecord& launch) {
    auto notam_opt = find_by_id(notam_id);
    if (!notam_opt.has_value()) return;

    const auto& notam = notam_opt.value();
    LeadTimeRecord rec;
    rec.notam_id = notam_id;
    rec.site_id = launch.site_id;
    rec.vehicle = launch.vehicle;
    rec.orbit = launch.orbit;
    rec.notam_issued = notam.issued;
    rec.launch_time = launch.launch_time;
    rec.lead_time_hours = static_cast<double>(launch.launch_time - notam.issued) / 3600.0;

    lead_times_.push_back(rec);
}

std::vector<LeadTimeRecord> NotamArchive::get_lead_times() const {
    return lead_times_;
}

std::vector<LeadTimeRecord> NotamArchive::get_lead_times_for_site(const std::string& site_id) const {
    std::vector<LeadTimeRecord> result;
    for (const auto& lt : lead_times_) {
        if (lt.site_id == site_id) {
            result.push_back(lt);
        }
    }
    return result;
}

// --- Accessors ---

const std::vector<NOTAM>& NotamArchive::get_all() const {
    return notams_;
}

const std::vector<LaunchRecord>& NotamArchive::get_launches() const {
    return launches_;
}

size_t NotamArchive::size() const {
    return notams_.size();
}

void NotamArchive::clear() {
    notams_.clear();
    launches_.clear();
    lead_times_.clear();
}

// --- Private Helpers ---

double NotamArchive::haversine_nm(const LatLon& a, const LatLon& b) const {
    double lat1 = a.lat * DEG2RAD;
    double lat2 = b.lat * DEG2RAD;
    double dlat = (b.lat - a.lat) * DEG2RAD;
    double dlon = (b.lon - a.lon) * DEG2RAD;

    double h = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(h), std::sqrt(1.0 - h));
    double d_km = R_EARTH_KM * c;
    return d_km / 1.852;  // km to nautical miles
}

bool NotamArchive::overlaps_time(const NOTAM& n, Timestamp start, Timestamp end) const {
    return n.effective_start <= end && start <= n.effective_end;
}

bool NotamArchive::overlaps_location(const NOTAM& n, double lat, double lon, double radius_nm) const {
    LatLon center{n.center_lat, n.center_lon};
    LatLon query{lat, lon};
    double dist = haversine_nm(center, query);
    return dist <= radius_nm;
}

} // namespace notam_archive
