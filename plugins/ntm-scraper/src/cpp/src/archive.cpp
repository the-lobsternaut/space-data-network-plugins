#include "ntm_scraper/archive.h"
#include <algorithm>

namespace ntm_scraper {

void NoticeArchive::add(const MaritimeNotice& notice) {
    notices_.push_back(notice);
}

void NoticeArchive::add_batch(const std::vector<MaritimeNotice>& notices) {
    notices_.insert(notices_.end(), notices.begin(), notices.end());
}

std::optional<MaritimeNotice> NoticeArchive::find_by_id(const std::string& id) const {
    for (const auto& n : notices_) {
        if (n.notice_id == id) return n;
    }
    return std::nullopt;
}

std::vector<MaritimeNotice> NoticeArchive::query_by_time(int64_t start, int64_t end) const {
    std::vector<MaritimeNotice> results;
    for (const auto& n : notices_) {
        // Notice overlaps with the query range
        if (n.effective_end >= start && n.effective_start <= end) {
            results.push_back(n);
        }
    }
    return results;
}

std::vector<MaritimeNotice> NoticeArchive::query_by_classification(HazardClassification cls) const {
    std::vector<MaritimeNotice> results;
    for (const auto& n : notices_) {
        if (n.classification == cls) {
            results.push_back(n);
        }
    }
    return results;
}

std::vector<MaritimeNotice> NoticeArchive::query_by_region(double min_lat, double max_lat,
                                                             double min_lon, double max_lon) const {
    std::vector<MaritimeNotice> results;
    for (const auto& n : notices_) {
        if (n.zone.type == GeometryType::UNKNOWN) continue;

        // Check bounding box overlap
        if (n.zone.max_lat >= min_lat && n.zone.min_lat <= max_lat &&
            n.zone.max_lon >= min_lon && n.zone.min_lon <= max_lon) {
            results.push_back(n);
        }
    }
    return results;
}

void NoticeArchive::process_cancellations() {
    // Build a set of IDs that are cancelled
    for (const auto& n : notices_) {
        if (!n.cancel_notice_id.empty()) {
            // Find the referenced notice and mark it cancelled
            for (auto& target : notices_) {
                if (target.notice_id == n.cancel_notice_id) {
                    target.is_cancelled = true;
                }
            }
        }
    }
}

std::vector<MaritimeNotice> NoticeArchive::get_active_notices() const {
    std::vector<MaritimeNotice> results;
    for (const auto& n : notices_) {
        if (!n.is_cancelled) {
            results.push_back(n);
        }
    }
    return results;
}

const std::vector<MaritimeNotice>& NoticeArchive::get_all() const {
    return notices_;
}

size_t NoticeArchive::size() const {
    return notices_.size();
}

void NoticeArchive::clear() {
    notices_.clear();
}

} // namespace ntm_scraper
