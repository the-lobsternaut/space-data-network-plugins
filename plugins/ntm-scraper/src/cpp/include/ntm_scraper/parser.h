#pragma once
#include "ntm_scraper/types.h"
#include <optional>
#include <string>
#include <vector>

namespace ntm_scraper {

class NtmParser {
public:
    // Parse a single NGA broadcast warning text
    std::optional<MaritimeNotice> parse(const std::string& text) const;

    // Parse multiple notices from a concatenated text block
    std::vector<MaritimeNotice> parse_batch(const std::string& text) const;

    // Classify a notice based on its text
    HazardClassification classify(const std::string& text) const;

    // Extract exclusion zone geometry from notice text
    ExclusionZone extract_geometry(const std::string& text) const;

    // Parse a DMS coordinate string like "28-25.0N" to decimal degrees
    static double parse_lat(const std::string& s);
    static double parse_lon(const std::string& s);

    // Parse NGA date format like "271400Z JUL 2024"
    static int64_t parse_nga_datetime(const std::string& s);

    // Extract broadcast area from header
    static BroadcastArea parse_broadcast_area(const std::string& text);

    // Extract notice ID
    static std::string parse_notice_id(const std::string& text);
};

} // namespace ntm_scraper
