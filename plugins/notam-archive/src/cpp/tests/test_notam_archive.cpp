#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>
#include "notam_archive/types.h"
#include "notam_archive/archive.h"
#include "notam_archive/analyzer.h"

using namespace notam_archive;

static int tests_run = 0, tests_passed = 0;
#define TEST(name) void name(); struct name##_reg { name##_reg() { tests_run++; std::cout << "  " #name "... "; try { name(); tests_passed++; std::cout << "PASS\n"; } catch (const std::exception& e) { std::cout << "FAIL: " << e.what() << "\n"; } catch (...) { std::cout << "FAIL\n"; } } } name##_instance; void name()
#define ASSERT_TRUE(x) do { if (!(x)) throw std::runtime_error("ASSERT_TRUE failed: " #x); } while(0)
#define ASSERT_EQ(a, b) do { if ((a) != (b)) throw std::runtime_error("ASSERT_EQ failed: " #a " != " #b); } while(0)
#define ASSERT_NEAR(a, b, tol) do { if (std::abs((a)-(b)) > (tol)) throw std::runtime_error("ASSERT_NEAR failed: " + std::to_string(a) + " vs " + std::to_string(b)); } while(0)

// Helper: create a Cape Canaveral launch NOTAM
static NOTAM make_cape_notam(const std::string& id, Timestamp issued,
                              Timestamp start, Timestamp end,
                              NotamClassification cls = NotamClassification::LAUNCH) {
    NOTAM n;
    n.notam_id = id;
    n.location_id = "KXMR";
    n.issued = issued;
    n.effective_start = start;
    n.effective_end = end;
    n.altitude_floor_ft = 0;
    n.altitude_ceiling_ft = -1.0;
    n.center_lat = 28.396837;
    n.center_lon = -80.605659;
    n.radius_nm = 30.0;
    n.notam_text = "SPACE OPERATIONS - LAUNCH";
    n.classification = cls;
    return n;
}

// Helper: create a Vandenberg NOTAM
static NOTAM make_vafb_notam(const std::string& id, Timestamp issued,
                              Timestamp start, Timestamp end,
                              NotamClassification cls = NotamClassification::LAUNCH) {
    NOTAM n;
    n.notam_id = id;
    n.location_id = "KVBG";
    n.issued = issued;
    n.effective_start = start;
    n.effective_end = end;
    n.altitude_floor_ft = 0;
    n.altitude_ceiling_ft = -1.0;
    n.center_lat = 34.7420;
    n.center_lon = -120.5724;
    n.radius_nm = 25.0;
    n.notam_text = "SPACE OPERATIONS - POLAR LAUNCH";
    n.classification = cls;
    return n;
}

static LaunchRecord make_launch(const std::string& id, const std::string& site,
                                 const std::string& vehicle, OrbitType orbit,
                                 Timestamp time, const std::vector<std::string>& notam_ids,
                                 bool scrubbed = false) {
    LaunchRecord l;
    l.launch_id = id;
    l.site_id = site;
    l.vehicle = vehicle;
    l.orbit = orbit;
    l.launch_time = time;
    l.notam_ids = notam_ids;
    l.scrubbed = scrubbed;
    return l;
}

// Timestamps: base = 1710000000 (2024-03-09 ~16:00 UTC)
static constexpr Timestamp BASE = 1710000000;
static constexpr Timestamp HOUR = 3600;
static constexpr Timestamp DAY = 86400;

// ==================== Archive Tests ====================

TEST(test_add_and_find) {
    NotamArchive archive;
    auto n = make_cape_notam("A0001/24", BASE, BASE + 48*HOUR, BASE + 52*HOUR);
    archive.add(n);
    ASSERT_EQ(archive.size(), 1u);

    auto found = archive.find_by_id("A0001/24");
    ASSERT_TRUE(found.has_value());
    ASSERT_EQ(found->notam_id, "A0001/24");
    ASSERT_EQ(found->location_id, "KXMR");

    auto missing = archive.find_by_id("NONEXISTENT");
    ASSERT_TRUE(!missing.has_value());
}

TEST(test_query_by_time) {
    NotamArchive archive;
    // NOTAM active from BASE+48h to BASE+52h
    archive.add(make_cape_notam("T1", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    // NOTAM active from BASE+100h to BASE+104h
    archive.add(make_cape_notam("T2", BASE, BASE + 100*HOUR, BASE + 104*HOUR));

    // Query overlapping first NOTAM only
    auto r1 = archive.query_by_time(BASE + 49*HOUR, BASE + 51*HOUR);
    ASSERT_EQ(r1.size(), 1u);
    ASSERT_EQ(r1[0].notam_id, "T1");

    // Query overlapping both
    auto r2 = archive.query_by_time(BASE + 40*HOUR, BASE + 110*HOUR);
    ASSERT_EQ(r2.size(), 2u);

    // Query overlapping none
    auto r3 = archive.query_by_time(BASE + 60*HOUR, BASE + 90*HOUR);
    ASSERT_EQ(r3.size(), 0u);
}

TEST(test_query_by_location) {
    NotamArchive archive;
    // Cape Canaveral: 28.396837, -80.605659
    archive.add(make_cape_notam("LOC1", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    // Vandenberg: 34.7420, -120.5724
    archive.add(make_vafb_notam("LOC2", BASE, BASE + 48*HOUR, BASE + 52*HOUR));

    // Query near Cape Canaveral (within 50nm)
    auto r1 = archive.query_by_location(28.5, -80.5, 50.0);
    ASSERT_EQ(r1.size(), 1u);
    ASSERT_EQ(r1[0].notam_id, "LOC1");

    // Query near Vandenberg (within 50nm)
    auto r2 = archive.query_by_location(34.7, -120.5, 50.0);
    ASSERT_EQ(r2.size(), 1u);
    ASSERT_EQ(r2[0].notam_id, "LOC2");

    // Query in the middle of nowhere
    auto r3 = archive.query_by_location(0.0, 0.0, 10.0);
    ASSERT_EQ(r3.size(), 0u);

    // Query with huge radius gets both
    auto r4 = archive.query_by_location(30.0, -100.0, 2000.0);
    ASSERT_EQ(r4.size(), 2u);
}

TEST(test_query_by_classification) {
    NotamArchive archive;
    archive.add(make_cape_notam("CL1", BASE, BASE + 48*HOUR, BASE + 52*HOUR, NotamClassification::LAUNCH));
    archive.add(make_cape_notam("CL2", BASE, BASE + 48*HOUR, BASE + 52*HOUR, NotamClassification::HAZARD));
    archive.add(make_cape_notam("CL3", BASE, BASE + 48*HOUR, BASE + 52*HOUR, NotamClassification::ROUTINE));

    auto launches = archive.query_by_classification(NotamClassification::LAUNCH);
    ASSERT_EQ(launches.size(), 1u);
    ASSERT_EQ(launches[0].notam_id, "CL1");

    auto hazards = archive.query_by_classification(NotamClassification::HAZARD);
    ASSERT_EQ(hazards.size(), 1u);
    ASSERT_EQ(hazards[0].notam_id, "CL2");
}

TEST(test_query_by_site) {
    NotamArchive archive;
    archive.add(make_cape_notam("S1", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    archive.add(make_vafb_notam("S2", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    archive.add(make_cape_notam("S3", BASE + HOUR, BASE + 49*HOUR, BASE + 53*HOUR));

    auto cape = archive.query_by_site("KXMR");
    ASSERT_EQ(cape.size(), 2u);

    auto vafb = archive.query_by_site("KVBG");
    ASSERT_EQ(vafb.size(), 1u);
}

TEST(test_query_active_at) {
    NotamArchive archive;
    archive.add(make_cape_notam("ACT1", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    archive.add(make_cape_notam("ACT2", BASE, BASE + 50*HOUR, BASE + 54*HOUR));
    archive.add(make_cape_notam("ACT3", BASE, BASE + 60*HOUR, BASE + 64*HOUR));

    // At BASE+51h: ACT1 and ACT2 are active
    auto r = archive.query_active_at(BASE + 51*HOUR);
    ASSERT_EQ(r.size(), 2u);

    // At BASE+53h: only ACT2 is active
    auto r2 = archive.query_active_at(BASE + 53*HOUR);
    ASSERT_EQ(r2.size(), 1u);
    ASSERT_EQ(r2[0].notam_id, "ACT2");

    // At BASE+62h: only ACT3
    auto r3 = archive.query_active_at(BASE + 62*HOUR);
    ASSERT_EQ(r3.size(), 1u);
    ASSERT_EQ(r3[0].notam_id, "ACT3");
}

TEST(test_add_batch) {
    NotamArchive archive;
    std::vector<NOTAM> batch;
    for (int i = 0; i < 10; ++i) {
        batch.push_back(make_cape_notam("BATCH" + std::to_string(i),
            BASE + i * HOUR, BASE + 48*HOUR + i*HOUR, BASE + 52*HOUR + i*HOUR));
    }
    archive.add_batch(batch);
    ASSERT_EQ(archive.size(), 10u);
    ASSERT_EQ(archive.get_all().size(), 10u);
}

// ==================== Pattern Detection Tests ====================

TEST(test_detect_replacement) {
    NotamArchive archive;
    // Original NOTAM
    archive.add(make_cape_notam("REP_OLD", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    // Replacement with shifted time window
    archive.add(make_cape_notam("REP_NEW", BASE + 2*HOUR, BASE + 50*HOUR, BASE + 54*HOUR));

    auto changes = archive.detect_changes();
    ASSERT_EQ(changes.size(), 1u);
    ASSERT_EQ(changes[0].old_notam_id, "REP_OLD");
    ASSERT_EQ(changes[0].new_notam_id, "REP_NEW");
    ASSERT_EQ(static_cast<int>(changes[0].change_type), static_cast<int>(ChangeType::REPLACEMENT));
}

TEST(test_detect_extension) {
    NotamArchive archive;
    // Original: 48h to 52h
    archive.add(make_cape_notam("EXT_OLD", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    // Extended: 48h to 60h (same start, later end)
    archive.add(make_cape_notam("EXT_NEW", BASE + HOUR, BASE + 48*HOUR, BASE + 60*HOUR));

    auto changes = archive.detect_changes();
    ASSERT_EQ(changes.size(), 1u);
    ASSERT_EQ(static_cast<int>(changes[0].change_type), static_cast<int>(ChangeType::EXTENSION));
}

TEST(test_detect_narrowing) {
    NotamArchive archive;
    // Original: 48h to 60h (12 hour window)
    archive.add(make_cape_notam("NAR_OLD", BASE, BASE + 48*HOUR, BASE + 60*HOUR));
    // Narrowed: 48h to 50h (2 hour window, < 80% of original)
    archive.add(make_cape_notam("NAR_NEW", BASE + HOUR, BASE + 50*HOUR, BASE + 52*HOUR));

    auto changes = archive.detect_changes();
    ASSERT_EQ(changes.size(), 1u);
    ASSERT_EQ(static_cast<int>(changes[0].change_type), static_cast<int>(ChangeType::NARROWING));
}

TEST(test_detect_cancellation) {
    NotamArchive archive;
    // Original: normal window
    archive.add(make_cape_notam("CAN_OLD", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    // Cancelled: zero-length window
    archive.add(make_cape_notam("CAN_NEW", BASE + HOUR, BASE + 48*HOUR, BASE + 48*HOUR));

    auto changes = archive.detect_changes();
    ASSERT_EQ(changes.size(), 1u);
    ASSERT_EQ(static_cast<int>(changes[0].change_type), static_cast<int>(ChangeType::CANCELLATION));
}

TEST(test_scrub_indicator) {
    NotamArchive archive;
    NotamChange narrowing;
    narrowing.change_type = ChangeType::NARROWING;
    ASSERT_TRUE(archive.is_scrub_indicator(narrowing));

    NotamChange cancellation;
    cancellation.change_type = ChangeType::CANCELLATION;
    ASSERT_TRUE(archive.is_scrub_indicator(cancellation));

    NotamChange extension;
    extension.change_type = ChangeType::EXTENSION;
    ASSERT_TRUE(!archive.is_scrub_indicator(extension));

    NotamChange replacement;
    replacement.change_type = ChangeType::REPLACEMENT;
    ASSERT_TRUE(!archive.is_scrub_indicator(replacement));
}

TEST(test_reschedule_indicator) {
    NotamArchive archive;

    NotamChange extension;
    extension.change_type = ChangeType::EXTENSION;
    extension.old_start = BASE;
    extension.new_start = BASE;
    ASSERT_TRUE(archive.is_reschedule_indicator(extension));

    // Large time shift (>12 hours)
    NotamChange shift;
    shift.change_type = ChangeType::REPLACEMENT;
    shift.old_start = BASE;
    shift.new_start = BASE + 24*HOUR;
    ASSERT_TRUE(archive.is_reschedule_indicator(shift));

    // Small time shift (<12 hours)
    NotamChange small_shift;
    small_shift.change_type = ChangeType::REPLACEMENT;
    small_shift.old_start = BASE;
    small_shift.new_start = BASE + 2*HOUR;
    ASSERT_TRUE(!archive.is_reschedule_indicator(small_shift));
}

TEST(test_find_replacements) {
    NotamArchive archive;
    auto n1 = make_cape_notam("FR1", BASE, BASE + 48*HOUR, BASE + 52*HOUR);
    auto n2 = make_cape_notam("FR2", BASE + HOUR, BASE + 50*HOUR, BASE + 54*HOUR);
    auto n3 = make_vafb_notam("FR3", BASE, BASE + 48*HOUR, BASE + 52*HOUR);  // different site
    auto n4 = make_cape_notam("FR4", BASE, BASE + 100*HOUR, BASE + 104*HOUR); // no time overlap

    archive.add(n1);
    archive.add(n2);
    archive.add(n3);
    archive.add(n4);

    auto replacements = archive.find_replacements(n1);
    ASSERT_EQ(replacements.size(), 1u);
    ASSERT_EQ(replacements[0].notam_id, "FR2");
}

// ==================== Lead Time Tests ====================

TEST(test_record_lead_time) {
    NotamArchive archive;
    // NOTAM issued at BASE, launch at BASE+72h => lead time = 72 hours
    auto n = make_cape_notam("LT1", BASE, BASE + 70*HOUR, BASE + 74*HOUR);
    archive.add(n);

    auto launch = make_launch("L1", "KXMR", "Falcon 9", OrbitType::LEO,
                               BASE + 72*HOUR, {"LT1"});
    archive.add_launch(launch);
    archive.record_lead_time("LT1", launch);

    auto lts = archive.get_lead_times();
    ASSERT_EQ(lts.size(), 1u);
    ASSERT_NEAR(lts[0].lead_time_hours, 72.0, 0.01);
    ASSERT_EQ(lts[0].site_id, "KXMR");
    ASSERT_EQ(lts[0].vehicle, "Falcon 9");
}

TEST(test_lead_times_by_site) {
    NotamArchive archive;
    archive.add(make_cape_notam("LTS1", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    archive.add(make_vafb_notam("LTS2", BASE, BASE + 48*HOUR, BASE + 52*HOUR));

    auto l1 = make_launch("LL1", "KXMR", "Falcon 9", OrbitType::LEO, BASE + 50*HOUR, {"LTS1"});
    auto l2 = make_launch("LL2", "KVBG", "Falcon 9", OrbitType::SSO, BASE + 50*HOUR, {"LTS2"});

    archive.add_launch(l1);
    archive.add_launch(l2);
    archive.record_lead_time("LTS1", l1);
    archive.record_lead_time("LTS2", l2);

    auto cape_lts = archive.get_lead_times_for_site("KXMR");
    ASSERT_EQ(cape_lts.size(), 1u);
    ASSERT_EQ(cape_lts[0].notam_id, "LTS1");

    auto vafb_lts = archive.get_lead_times_for_site("KVBG");
    ASSERT_EQ(vafb_lts.size(), 1u);
    ASSERT_EQ(vafb_lts[0].notam_id, "LTS2");
}

// ==================== Statistics Tests ====================

TEST(test_average_lead_time) {
    NotamArchive archive;
    NotamAnalyzer analyzer(archive);

    std::vector<LeadTimeRecord> records;
    LeadTimeRecord r1; r1.lead_time_hours = 24.0;
    LeadTimeRecord r2; r2.lead_time_hours = 48.0;
    LeadTimeRecord r3; r3.lead_time_hours = 72.0;
    records = {r1, r2, r3};

    double avg = analyzer.average_lead_time(records);
    ASSERT_NEAR(avg, 48.0, 0.01);
}

TEST(test_median_lead_time) {
    NotamArchive archive;
    NotamAnalyzer analyzer(archive);

    // Odd count
    std::vector<LeadTimeRecord> odd_records;
    LeadTimeRecord r1; r1.lead_time_hours = 10.0;
    LeadTimeRecord r2; r2.lead_time_hours = 30.0;
    LeadTimeRecord r3; r3.lead_time_hours = 50.0;
    odd_records = {r3, r1, r2};  // unsorted on purpose
    ASSERT_NEAR(analyzer.median_lead_time(odd_records), 30.0, 0.01);

    // Even count
    LeadTimeRecord r4; r4.lead_time_hours = 70.0;
    std::vector<LeadTimeRecord> even_records = {r3, r1, r4, r2};
    // sorted: 10, 30, 50, 70 => median = (30+50)/2 = 40
    ASSERT_NEAR(analyzer.median_lead_time(even_records), 40.0, 0.01);
}

TEST(test_stddev_lead_time) {
    NotamArchive archive;
    NotamAnalyzer analyzer(archive);

    std::vector<LeadTimeRecord> records;
    LeadTimeRecord r1; r1.lead_time_hours = 2.0;
    LeadTimeRecord r2; r2.lead_time_hours = 4.0;
    LeadTimeRecord r3; r3.lead_time_hours = 4.0;
    LeadTimeRecord r4; r4.lead_time_hours = 4.0;
    LeadTimeRecord r5; r5.lead_time_hours = 5.0;
    LeadTimeRecord r6; r6.lead_time_hours = 5.0;
    LeadTimeRecord r7; r7.lead_time_hours = 7.0;
    LeadTimeRecord r8; r8.lead_time_hours = 9.0;
    records = {r1, r2, r3, r4, r5, r6, r7, r8};
    // mean = 40/8 = 5.0
    // sum of sq diffs = 9+1+1+1+0+0+4+16 = 32
    // variance = 32/7 = 4.571428...
    // stddev = sqrt(4.571428) = 2.13809...
    ASSERT_NEAR(analyzer.stddev_lead_time(records), 2.13809, 0.001);
}

TEST(test_site_statistics) {
    NotamArchive archive;

    // Add NOTAMs for Cape
    archive.add(make_cape_notam("SS1", BASE, BASE + 48*HOUR, BASE + 52*HOUR, NotamClassification::LAUNCH));
    archive.add(make_cape_notam("SS2", BASE + HOUR, BASE + 96*HOUR, BASE + 100*HOUR, NotamClassification::LAUNCH));
    archive.add(make_cape_notam("SS3", BASE + 2*HOUR, BASE + 48*HOUR, BASE + 52*HOUR, NotamClassification::HAZARD));

    // Add launches
    auto l1 = make_launch("SL1", "KXMR", "Falcon 9", OrbitType::LEO, BASE + 50*HOUR, {"SS1"});
    auto l2 = make_launch("SL2", "KXMR", "Falcon 9", OrbitType::GTO, BASE + 98*HOUR, {"SS2"}, true);
    archive.add_launch(l1);
    archive.add_launch(l2);
    archive.record_lead_time("SS1", l1);
    archive.record_lead_time("SS2", l2);

    NotamAnalyzer analyzer(archive);
    auto stats = analyzer.compute_site_stats("KXMR");

    ASSERT_EQ(stats.total_notams, 3);
    ASSERT_EQ(stats.launch_notams, 2);
    ASSERT_EQ(stats.hazard_notams, 1);
    ASSERT_EQ(stats.total_launches, 2);
    ASSERT_EQ(stats.scrubbed_launches, 1);
    ASSERT_NEAR(stats.scrub_rate, 0.5, 0.01);
    ASSERT_TRUE(stats.avg_lead_time_hours > 0);
}

TEST(test_vehicle_statistics) {
    NotamArchive archive;

    archive.add(make_cape_notam("VS1", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    archive.add(make_cape_notam("VS2", BASE + HOUR, BASE + 96*HOUR, BASE + 100*HOUR));

    auto l1 = make_launch("VL1", "KXMR", "Falcon 9", OrbitType::LEO, BASE + 50*HOUR, {"VS1"});
    auto l2 = make_launch("VL2", "KXMR", "Falcon 9", OrbitType::GTO, BASE + 98*HOUR, {"VS2"}, true);
    archive.add_launch(l1);
    archive.add_launch(l2);
    archive.record_lead_time("VS1", l1);
    archive.record_lead_time("VS2", l2);

    NotamAnalyzer analyzer(archive);
    auto stats = analyzer.compute_vehicle_stats("Falcon 9");

    ASSERT_EQ(stats.total_launches, 2);
    ASSERT_EQ(stats.scrubbed, 1);
    ASSERT_NEAR(stats.scrub_rate, 0.5, 0.01);
    ASSERT_NEAR(stats.avg_notams_per_launch, 1.0, 0.01);
    ASSERT_TRUE(stats.avg_lead_time_hours > 0);
}

TEST(test_compute_statistics) {
    NotamArchive archive;

    archive.add(make_cape_notam("CS1", BASE, BASE + 48*HOUR, BASE + 52*HOUR));
    archive.add(make_vafb_notam("CS2", BASE + DAY, BASE + 96*HOUR, BASE + 100*HOUR));

    auto l1 = make_launch("CL1", "KXMR", "Falcon 9", OrbitType::LEO, BASE + 50*HOUR, {"CS1"});
    auto l2 = make_launch("CL2", "KVBG", "Falcon 9", OrbitType::SSO, BASE + 98*HOUR, {"CS2"});
    archive.add_launch(l1);
    archive.add_launch(l2);
    archive.record_lead_time("CS1", l1);
    archive.record_lead_time("CS2", l2);

    NotamAnalyzer analyzer(archive);
    auto stats = analyzer.compute_statistics();

    ASSERT_EQ(stats.total_notams, 2);
    ASSERT_EQ(stats.total_launches, 2);
    ASSERT_TRUE(stats.earliest_notam == BASE);
    ASSERT_TRUE(stats.latest_notam == BASE + DAY);
    ASSERT_TRUE(stats.by_site.size() >= 2u);
    ASSERT_TRUE(stats.by_vehicle.size() >= 1u);
    ASSERT_TRUE(stats.overall_avg_lead_time_hours > 0);
    ASSERT_NEAR(stats.overall_scrub_rate, 0.0, 0.01);
}

// ==================== Analysis Tests ====================

TEST(test_activity_timeline) {
    NotamArchive archive;
    // NOTAM active from BASE to BASE+4h
    archive.add(make_cape_notam("AT1", BASE - DAY, BASE, BASE + 4*HOUR));
    // NOTAM active from BASE+2h to BASE+6h
    archive.add(make_cape_notam("AT2", BASE - DAY, BASE + 2*HOUR, BASE + 6*HOUR));

    NotamAnalyzer analyzer(archive);
    auto timeline = analyzer.activity_timeline(BASE, BASE + 6*HOUR, 1);

    // At BASE: 1 active (AT1)
    ASSERT_EQ(timeline[0].second, 1);
    // At BASE+2h: 2 active (AT1, AT2)
    ASSERT_EQ(timeline[2].second, 2);
    // At BASE+3h: 2 active
    ASSERT_EQ(timeline[3].second, 2);
    // At BASE+5h: 1 active (AT2 only)
    ASSERT_EQ(timeline[5].second, 1);
}

TEST(test_detect_anomalies) {
    NotamArchive archive;

    // Create several NOTAMs and launches with normal lead times
    for (int i = 0; i < 10; ++i) {
        std::string nid = "AN" + std::to_string(i);
        Timestamp issued = BASE + i * DAY;
        archive.add(make_cape_notam(nid, issued,
            issued + 46*HOUR, issued + 50*HOUR));

        auto launch = make_launch("AL" + std::to_string(i), "KXMR", "Falcon 9",
            OrbitType::LEO, issued + 48*HOUR, {nid});
        archive.add_launch(launch);
        archive.record_lead_time(nid, launch);
    }

    // Add one with an extreme lead time (anomalous)
    std::string anom_id = "AN_EXTREME";
    archive.add(make_cape_notam(anom_id, BASE - 30*DAY,
        BASE + 200*HOUR, BASE + 204*HOUR));
    auto anom_launch = make_launch("AL_EXTREME", "KXMR", "Falcon 9",
        OrbitType::LEO, BASE + 202*HOUR, {anom_id});
    archive.add_launch(anom_launch);
    archive.record_lead_time(anom_id, anom_launch);

    NotamAnalyzer analyzer(archive);
    auto anomalies = analyzer.detect_anomalies();

    // Should detect the extreme lead time as anomalous
    bool found_extreme = false;
    for (const auto& a : anomalies) {
        if (a.notam_id == anom_id) {
            found_extreme = true;
            ASSERT_TRUE(a.severity > 0.0);
        }
    }
    ASSERT_TRUE(found_extreme);
}

TEST(test_lead_time_trend) {
    NotamArchive archive;

    // Create lead times spanning 90 days, with increasing trend
    for (int i = 0; i < 6; ++i) {
        std::string nid = "TR" + std::to_string(i);
        Timestamp issued = BASE + i * 15 * DAY;
        double desired_lead_hours = 24.0 + i * 8.0;  // 24, 32, 40, 48, 56, 64
        Timestamp launch_time = issued + static_cast<Timestamp>(desired_lead_hours * 3600);

        archive.add(make_cape_notam(nid, issued,
            launch_time - 2*HOUR, launch_time + 2*HOUR));
        auto launch = make_launch("TRL" + std::to_string(i), "KXMR", "Falcon 9",
            OrbitType::LEO, launch_time, {nid});
        archive.add_launch(launch);
        archive.record_lead_time(nid, launch);
    }

    NotamAnalyzer analyzer(archive);
    auto trend = analyzer.lead_time_trend("KXMR", 30);

    ASSERT_TRUE(trend.size() >= 2u);
    // Later buckets should have higher average lead times
    ASSERT_TRUE(trend.back().second > trend.front().second);
}

// ==================== Haversine Validation ====================

TEST(test_haversine_known_distance) {
    // Verify haversine with known distance:
    // Cape Canaveral (28.3968, -80.6057) to Vandenberg (34.7420, -120.5724)
    // Approx distance: ~2000 nm
    NotamArchive archive;
    archive.add(make_cape_notam("HAV1", BASE, BASE + 48*HOUR, BASE + 52*HOUR));

    // Query from Vandenberg with radius 2100nm should find Cape
    auto r1 = archive.query_by_location(34.7420, -120.5724, 2100.0);
    ASSERT_EQ(r1.size(), 1u);

    // But not with radius 1800nm
    auto r2 = archive.query_by_location(34.7420, -120.5724, 1800.0);
    ASSERT_EQ(r2.size(), 0u);
}

int main() {
    std::cout << "\n=== NOTAM Archive Test Suite ===\n\n";
    // Tests are auto-registered and run before main
    std::cout << "\n" << tests_passed << "/" << tests_run << " tests passed\n";
    return tests_passed == tests_run ? 0 : 1;
}
