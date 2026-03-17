#pragma once
#include "da_pipeline/types.h"
#include <optional>
#include <string>
#include <vector>

namespace da_pipeline {

class NotamParser {
public:
    NotamParser() = default;

    /// Parse an FAA-format NOTAM from raw text
    std::optional<NOTAM> parse(const std::string& text) const;

    /// Parse an ICAO-format NOTAM
    std::optional<NOTAM> parse_icao(const std::string& text) const;

    /// Classify a parsed NOTAM
    NotamClassification classify(const NOTAM& n) const;

    /// Quick check whether NOTAM is launch-related
    bool is_launch_related(const NOTAM& n) const;

    /// Convert NOTAM geometry into an ExclusionZone
    ExclusionZone notam_to_zone(const NOTAM& n) const;

    /// Parse multiple NOTAMs from a block of text (separated by blank lines)
    std::vector<NOTAM> parse_batch(const std::string& text) const;

private:
    /// Parse DMS coordinate "DDMMSSN" or "DDDMMSSW"
    static double parse_dms_lat(const std::string& s);
    static double parse_dms_lon(const std::string& s);

    /// Parse YYMMDDHHMM timestamp
    static Timestamp parse_notam_datetime(const std::string& s);

    /// Check text for keywords
    static bool contains_any(const std::string& text, const std::vector<std::string>& keywords);
};

} // namespace da_pipeline
