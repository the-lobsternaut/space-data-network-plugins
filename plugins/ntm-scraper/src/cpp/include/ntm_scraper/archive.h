#pragma once
#include "ntm_scraper/types.h"
#include <vector>
#include <string>
#include <optional>

namespace ntm_scraper {

class NoticeArchive {
public:
    void add(const MaritimeNotice& notice);
    void add_batch(const std::vector<MaritimeNotice>& notices);

    // Query by ID
    std::optional<MaritimeNotice> find_by_id(const std::string& id) const;

    // Query by time range
    std::vector<MaritimeNotice> query_by_time(int64_t start, int64_t end) const;

    // Query by classification
    std::vector<MaritimeNotice> query_by_classification(HazardClassification cls) const;

    // Query by geographic bounding box
    std::vector<MaritimeNotice> query_by_region(double min_lat, double max_lat,
                                                  double min_lon, double max_lon) const;

    // Process cancellation notices
    void process_cancellations();

    // Get active (non-cancelled) notices
    std::vector<MaritimeNotice> get_active_notices() const;

    // Get all notices
    const std::vector<MaritimeNotice>& get_all() const;

    size_t size() const;
    void clear();

private:
    std::vector<MaritimeNotice> notices_;
};

} // namespace ntm_scraper
