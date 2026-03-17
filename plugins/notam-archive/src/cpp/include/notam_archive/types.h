#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include <map>
#include <optional>

namespace notam_archive {

using Timestamp = int64_t;

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double R_EARTH_KM = 6378.137;

struct LatLon { double lat = 0, lon = 0; };
struct Polygon { std::vector<LatLon> vertices; };

enum class NotamClassification { LAUNCH, REENTRY, HAZARD, AIRSPACE, ROUTINE };
enum class OrbitType { LEO, GTO, SSO, POLAR, ISS, MEO, HEO, ESCAPE, UNKNOWN };
enum class ChangeType { NEW, REPLACEMENT, EXTENSION, CANCELLATION, NARROWING };

struct NOTAM {
    std::string notam_id;
    std::string location_id;
    Timestamp issued = 0;
    Timestamp effective_start = 0;
    Timestamp effective_end = 0;
    double altitude_floor_ft = 0;
    double altitude_ceiling_ft = 0;  // -1.0 = unlimited
    double center_lat = 0, center_lon = 0;
    double radius_nm = 0;
    Polygon polygon;
    std::string notam_text;
    NotamClassification classification = NotamClassification::ROUTINE;
};

struct LaunchRecord {
    std::string launch_id;
    std::string site_id;
    std::string vehicle;
    OrbitType orbit = OrbitType::UNKNOWN;
    Timestamp launch_time = 0;
    std::vector<std::string> notam_ids;
    bool scrubbed = false;
};

struct NotamChange {
    std::string old_notam_id;
    std::string new_notam_id;
    ChangeType change_type = ChangeType::NEW;
    Timestamp detected_at = 0;
    Timestamp old_start = 0, old_end = 0;
    Timestamp new_start = 0, new_end = 0;
    std::string description;
};

struct LeadTimeRecord {
    std::string notam_id;
    std::string site_id;
    std::string vehicle;
    OrbitType orbit = OrbitType::UNKNOWN;
    double lead_time_hours = 0;  // hours between NOTAM issuance and effective_start
    Timestamp notam_issued = 0;
    Timestamp launch_time = 0;
};

struct SiteStatistics {
    std::string site_id;
    int total_notams = 0;
    int launch_notams = 0;
    int hazard_notams = 0;
    double avg_lead_time_hours = 0;
    double median_lead_time_hours = 0;
    double min_lead_time_hours = 0;
    double max_lead_time_hours = 0;
    double stddev_lead_time_hours = 0;
    int total_launches = 0;
    int scrubbed_launches = 0;
    double scrub_rate = 0;  // fraction
};

struct VehicleStatistics {
    std::string vehicle;
    int total_launches = 0;
    double avg_lead_time_hours = 0;
    double median_lead_time_hours = 0;
    double avg_notams_per_launch = 0;
    int scrubbed = 0;
    double scrub_rate = 0;
};

struct OrbitStatistics {
    OrbitType orbit = OrbitType::UNKNOWN;
    int total_launches = 0;
    double avg_lead_time_hours = 0;
    double median_lead_time_hours = 0;
};

struct ArchiveStatistics {
    int total_notams = 0;
    int total_launches = 0;
    int total_changes = 0;
    Timestamp earliest_notam = 0;
    Timestamp latest_notam = 0;
    std::vector<SiteStatistics> by_site;
    std::vector<VehicleStatistics> by_vehicle;
    std::vector<OrbitStatistics> by_orbit;
    double overall_avg_lead_time_hours = 0;
    double overall_scrub_rate = 0;
};

} // namespace notam_archive
