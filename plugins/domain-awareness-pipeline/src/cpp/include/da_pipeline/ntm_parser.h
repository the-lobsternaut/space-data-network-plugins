#pragma once
#include "da_pipeline/types.h"
#include <optional>
#include <string>
#include <vector>

namespace da_pipeline {

class NtmParser {
public:
    NtmParser() = default;

    /// Parse a single NTM from raw text
    std::optional<MaritimeNotice> parse(const std::string& text) const;

    /// Parse multiple NTMs separated by double newlines or "NNNN"
    std::vector<MaritimeNotice> parse_batch(const std::string& text) const;

    /// Classify hazard type from text
    HazardClassification classify(const std::string& text) const;

    /// Extract geometry (circle or polygon) from NTM text
    ExclusionZone extract_geometry(const std::string& text) const;

    /// Parse latitude "28-25.0N" format
    static double parse_lat(const std::string& s);

    /// Parse longitude "080-20.0W" format
    static double parse_lon(const std::string& s);

    /// Parse NGA datetime "DDHHMMZ MON YYYY" format
    static Timestamp parse_nga_datetime(const std::string& s);
};

} // namespace da_pipeline
