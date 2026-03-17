#include "ntm_scraper/parser.h"
#include "ntm_scraper/archive.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using namespace ntm_scraper;

// ─── Test helpers ───────────────────────────────────────────────────────────

static int tests_run    = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { \
        std::cout << "  TEST: " << #name << "... "; \
        tests_run++; \
    } while (0)

#define PASS() \
    do { \
        tests_passed++; \
        std::cout << "PASS" << std::endl; \
    } while (0)

#define ASSERT_TRUE(expr) \
    do { \
        if (!(expr)) { \
            std::cout << "FAIL at line " << __LINE__ << ": " << #expr << std::endl; \
            return; \
        } \
    } while (0)

#define ASSERT_EQ(a, b) \
    do { \
        if ((a) != (b)) { \
            std::cout << "FAIL at line " << __LINE__ << ": " << #a << " != " << #b << std::endl; \
            return; \
        } \
    } while (0)

#define ASSERT_NEAR(a, b, tol) \
    do { \
        if (std::abs((a) - (b)) > (tol)) { \
            std::cout << "FAIL at line " << __LINE__ << ": " << (a) << " != " << (b) \
                      << " (tol=" << (tol) << ")" << std::endl; \
            return; \
        } \
    } while (0)

// ─── Sample NGA texts ──────────────────────────────────────────────────────

static const std::string hydrolant_polygon =
    "HYDROLANT 1234/2024(56)\n"
    "NORTH ATLANTIC.\n"
    "UNITED STATES.\n"
    "CAPE CANAVERAL FLORIDA.\n"
    "SPACE OPERATIONS\n"
    "ROCKET DEBRIS\n"
    "1. HAZARDOUS OPERATIONS WILL BE CONDUCTED FROM\n"
    "   271400Z JUL 2024 TO 271800Z JUL 2024\n"
    "2. IN AN AREA BOUNDED BY\n"
    "   28-25.0N 080-20.0W\n"
    "   28-25.0N 079-50.0W\n"
    "   28-50.0N 079-50.0W\n"
    "   28-50.0N 080-20.0W\n"
    "3. CANCEL THIS MSG 271900Z JUL 2024\n"
    "4. AUTHORITY: EASTERN RANGE 45 SW/SEL\n";

static const std::string hydropac_circle =
    "HYDROPAC 5678/2024(12)\n"
    "NORTH PACIFIC.\n"
    "UNITED STATES.\n"
    "VANDENBERG CALIFORNIA.\n"
    "SPACE OPERATIONS\n"
    "FAIRING RECOVERY\n"
    "1. HAZARDOUS OPERATIONS FROM 151200Z DEC 2024 TO 151800Z DEC 2024\n"
    "2. IN AN AREA WITHIN 30NM RADIUS OF 34-45.0N 120-30.0W\n"
    "3. CANCEL THIS MSG 151900Z DEC 2024\n"
    "4. AUTHORITY: WESTERN RANGE 30 SW/SEL\n";

// ─── Parser Tests ───────────────────────────────────────────────────────────

void test_parse_hydrolant_polygon() {
    TEST(parse_hydrolant_polygon);

    NtmParser parser;
    auto result = parser.parse(hydrolant_polygon);
    ASSERT_TRUE(result.has_value());

    auto& notice = result.value();
    ASSERT_EQ(notice.notice_id, "HYDROLANT 1234/2024");
    ASSERT_EQ(notice.area, BroadcastArea::HYDROLANT);
    ASSERT_EQ(notice.classification, HazardClassification::ROCKET_DEBRIS);
    ASSERT_TRUE(notice.effective_start > 0);
    ASSERT_TRUE(notice.effective_end > notice.effective_start);
    ASSERT_EQ(notice.zone.type, GeometryType::POLYGON);
    ASSERT_EQ(notice.zone.polygon.vertices.size(), 4u);

    // First vertex: 28-25.0N = 28 + 25/60 = 28.41667
    ASSERT_NEAR(notice.zone.polygon.vertices[0].lat, 28.41667, 0.01);
    // 080-20.0W = -(80 + 20/60) = -80.33333
    ASSERT_NEAR(notice.zone.polygon.vertices[0].lon, -80.33333, 0.01);

    ASSERT_EQ(notice.authority, "EASTERN RANGE 45 SW/SEL");
    ASSERT_TRUE(notice.confidence > 0.8);

    PASS();
}

void test_parse_hydropac_circle() {
    TEST(parse_hydropac_circle);

    NtmParser parser;
    auto result = parser.parse(hydropac_circle);
    ASSERT_TRUE(result.has_value());

    auto& notice = result.value();
    ASSERT_EQ(notice.notice_id, "HYDROPAC 5678/2024");
    ASSERT_EQ(notice.area, BroadcastArea::HYDROPAC);
    ASSERT_EQ(notice.classification, HazardClassification::FAIRING_RECOVERY);
    ASSERT_TRUE(notice.effective_start > 0);
    ASSERT_TRUE(notice.effective_end > notice.effective_start);
    ASSERT_EQ(notice.zone.type, GeometryType::CIRCLE);
    ASSERT_NEAR(notice.zone.circle.radius_nm, 30.0, 0.1);

    // Center: 34-45.0N = 34 + 45/60 = 34.75
    ASSERT_NEAR(notice.zone.circle.center.lat, 34.75, 0.01);
    // 120-30.0W = -(120 + 30/60) = -120.5
    ASSERT_NEAR(notice.zone.circle.center.lon, -120.5, 0.01);

    ASSERT_EQ(notice.authority, "WESTERN RANGE 30 SW/SEL");

    PASS();
}

void test_parse_lat_lon() {
    TEST(parse_lat_lon);

    // Decimal minutes: DD-MM.MN
    ASSERT_NEAR(NtmParser::parse_lat("28-25.0N"), 28.41667, 0.01);
    ASSERT_NEAR(NtmParser::parse_lat("28-50.0N"), 28.83333, 0.01);
    ASSERT_NEAR(NtmParser::parse_lat("34-45.0N"), 34.75, 0.01);
    ASSERT_NEAR(NtmParser::parse_lat("10-30.0S"), -10.5, 0.01);

    ASSERT_NEAR(NtmParser::parse_lon("080-20.0W"), -80.33333, 0.01);
    ASSERT_NEAR(NtmParser::parse_lon("079-50.0W"), -79.83333, 0.01);
    ASSERT_NEAR(NtmParser::parse_lon("120-30.0W"), -120.5, 0.01);
    ASSERT_NEAR(NtmParser::parse_lon("045-15.0E"), 45.25, 0.01);

    // DMS: DD-MM-SSN
    ASSERT_NEAR(NtmParser::parse_lat("28-30-00N"), 28.5, 0.01);
    ASSERT_NEAR(NtmParser::parse_lon("079-30-00W"), -79.5, 0.01);

    PASS();
}

void test_parse_nga_datetime() {
    TEST(parse_nga_datetime);

    // 271400Z JUL 2024 = 27 Jul 2024 14:00 UTC
    int64_t ts1 = NtmParser::parse_nga_datetime("271400Z JUL 2024");
    ASSERT_TRUE(ts1 > 0);

    // 271800Z JUL 2024 = 27 Jul 2024 18:00 UTC
    int64_t ts2 = NtmParser::parse_nga_datetime("271800Z JUL 2024");
    ASSERT_TRUE(ts2 > ts1);

    // Difference should be 4 hours = 14400 seconds
    ASSERT_EQ(ts2 - ts1, 14400);

    // 151200Z DEC 2024 = 15 Dec 2024 12:00 UTC
    int64_t ts3 = NtmParser::parse_nga_datetime("151200Z DEC 2024");
    ASSERT_TRUE(ts3 > ts2);

    // Invalid should return 0
    ASSERT_EQ(NtmParser::parse_nga_datetime("garbage"), 0);
    ASSERT_EQ(NtmParser::parse_nga_datetime(""), 0);

    PASS();
}

void test_classify_rocket_debris() {
    TEST(classify_rocket_debris);

    NtmParser parser;
    ASSERT_EQ(parser.classify("ROCKET DEBRIS FALLING"), HazardClassification::ROCKET_DEBRIS);
    ASSERT_EQ(parser.classify("FALLING DEBRIS EXPECTED IN AREA"), HazardClassification::ROCKET_DEBRIS);

    PASS();
}

void test_classify_splashdown() {
    TEST(classify_splashdown);

    NtmParser parser;
    ASSERT_EQ(parser.classify("SPLASHDOWN AREA"), HazardClassification::SPLASHDOWN);
    ASSERT_EQ(parser.classify("SPACECRAFT SPLASHDOWN OPERATIONS"), HazardClassification::SPLASHDOWN);

    PASS();
}

void test_classify_booster_recovery() {
    TEST(classify_booster_recovery);

    NtmParser parser;
    ASSERT_EQ(parser.classify("BOOSTER RECOVERY OPERATIONS"), HazardClassification::BOOSTER_RECOVERY);
    ASSERT_EQ(parser.classify("BOOSTER LANDING ZONE"), HazardClassification::BOOSTER_RECOVERY);

    PASS();
}

void test_classify_fairing_recovery() {
    TEST(classify_fairing_recovery);

    NtmParser parser;
    ASSERT_EQ(parser.classify("FAIRING RECOVERY ZONE"), HazardClassification::FAIRING_RECOVERY);
    ASSERT_EQ(parser.classify("FAIRING OPERATIONS"), HazardClassification::FAIRING_RECOVERY);

    PASS();
}

void test_extract_polygon_geometry() {
    TEST(extract_polygon_geometry);

    NtmParser parser;
    std::string text =
        "IN AN AREA BOUNDED BY\n"
        "28-25.0N 080-20.0W\n"
        "28-25.0N 079-50.0W\n"
        "28-50.0N 079-50.0W\n"
        "28-50.0N 080-20.0W\n";

    auto zone = parser.extract_geometry(text);
    ASSERT_EQ(zone.type, GeometryType::POLYGON);
    ASSERT_EQ(zone.polygon.vertices.size(), 4u);

    ASSERT_NEAR(zone.polygon.vertices[0].lat, 28.41667, 0.01);
    ASSERT_NEAR(zone.polygon.vertices[0].lon, -80.33333, 0.01);
    ASSERT_NEAR(zone.polygon.vertices[2].lat, 28.83333, 0.01);
    ASSERT_NEAR(zone.polygon.vertices[2].lon, -79.83333, 0.01);

    // Bounds should be computed
    ASSERT_TRUE(zone.min_lat < zone.max_lat);
    ASSERT_TRUE(zone.min_lon < zone.max_lon);

    PASS();
}

void test_extract_circle_geometry() {
    TEST(extract_circle_geometry);

    NtmParser parser;
    std::string text = "IN AN AREA WITHIN 30NM RADIUS OF 34-45.0N 120-30.0W\n";

    auto zone = parser.extract_geometry(text);
    ASSERT_EQ(zone.type, GeometryType::CIRCLE);
    ASSERT_NEAR(zone.circle.radius_nm, 30.0, 0.1);
    ASSERT_NEAR(zone.circle.center.lat, 34.75, 0.01);
    ASSERT_NEAR(zone.circle.center.lon, -120.5, 0.01);

    // Bounds should be computed
    ASSERT_TRUE(zone.min_lat < zone.max_lat);
    ASSERT_TRUE(zone.min_lon < zone.max_lon);

    PASS();
}

// ─── Archive Tests ──────────────────────────────────────────────────────────

void test_archive_add_query() {
    TEST(archive_add_query);

    NtmParser parser;
    NoticeArchive archive;

    auto n1 = parser.parse(hydrolant_polygon);
    auto n2 = parser.parse(hydropac_circle);
    ASSERT_TRUE(n1.has_value());
    ASSERT_TRUE(n2.has_value());

    archive.add(n1.value());
    archive.add(n2.value());
    ASSERT_EQ(archive.size(), 2u);

    auto found = archive.find_by_id("HYDROLANT 1234/2024");
    ASSERT_TRUE(found.has_value());
    ASSERT_EQ(found.value().notice_id, "HYDROLANT 1234/2024");

    auto not_found = archive.find_by_id("NONEXISTENT 9999/2024");
    ASSERT_TRUE(!not_found.has_value());

    PASS();
}

void test_archive_time_query() {
    TEST(archive_time_query);

    NtmParser parser;
    NoticeArchive archive;

    auto n1 = parser.parse(hydrolant_polygon);
    auto n2 = parser.parse(hydropac_circle);
    ASSERT_TRUE(n1.has_value());
    ASSERT_TRUE(n2.has_value());

    archive.add(n1.value());
    archive.add(n2.value());

    // Query for Jul 2024 (should find hydrolant_polygon)
    int64_t jul_start = NtmParser::parse_nga_datetime("010000Z JUL 2024");
    int64_t jul_end = NtmParser::parse_nga_datetime("312359Z JUL 2024");
    auto jul_results = archive.query_by_time(jul_start, jul_end);
    ASSERT_EQ(jul_results.size(), 1u);
    ASSERT_EQ(jul_results[0].notice_id, "HYDROLANT 1234/2024");

    // Query for Dec 2024 (should find hydropac_circle)
    int64_t dec_start = NtmParser::parse_nga_datetime("010000Z DEC 2024");
    int64_t dec_end = NtmParser::parse_nga_datetime("312359Z DEC 2024");
    auto dec_results = archive.query_by_time(dec_start, dec_end);
    ASSERT_EQ(dec_results.size(), 1u);
    ASSERT_EQ(dec_results[0].notice_id, "HYDROPAC 5678/2024");

    // Query spanning both (should find both)
    auto all_results = archive.query_by_time(jul_start, dec_end);
    ASSERT_EQ(all_results.size(), 2u);

    PASS();
}

void test_archive_region_query() {
    TEST(archive_region_query);

    NtmParser parser;
    NoticeArchive archive;

    auto n1 = parser.parse(hydrolant_polygon);
    auto n2 = parser.parse(hydropac_circle);
    ASSERT_TRUE(n1.has_value());
    ASSERT_TRUE(n2.has_value());

    archive.add(n1.value());
    archive.add(n2.value());

    // Query around Cape Canaveral area (should find hydrolant_polygon)
    auto cape_results = archive.query_by_region(28.0, 29.0, -81.0, -79.0);
    ASSERT_EQ(cape_results.size(), 1u);
    ASSERT_EQ(cape_results[0].notice_id, "HYDROLANT 1234/2024");

    // Query around Vandenberg area (should find hydropac_circle)
    auto vafb_results = archive.query_by_region(34.0, 35.5, -121.5, -119.5);
    ASSERT_EQ(vafb_results.size(), 1u);
    ASSERT_EQ(vafb_results[0].notice_id, "HYDROPAC 5678/2024");

    // Query in middle of nowhere (should find nothing)
    auto empty_results = archive.query_by_region(0.0, 1.0, 0.0, 1.0);
    ASSERT_EQ(empty_results.size(), 0u);

    PASS();
}

void test_archive_cancellation() {
    TEST(archive_cancellation);

    NoticeArchive archive;

    // Create a notice
    MaritimeNotice original;
    original.notice_id = "HYDROLANT 1000/2024";
    original.area = BroadcastArea::HYDROLANT;
    original.classification = HazardClassification::ROCKET_DEBRIS;
    original.effective_start = 1000;
    original.effective_end = 2000;
    original.is_cancelled = false;

    // Create a cancellation notice
    MaritimeNotice cancel;
    cancel.notice_id = "HYDROLANT 1001/2024";
    cancel.area = BroadcastArea::HYDROLANT;
    cancel.cancel_notice_id = "HYDROLANT 1000/2024";
    cancel.classification = HazardClassification::UNKNOWN;
    cancel.effective_start = 1500;
    cancel.effective_end = 1500;
    cancel.is_cancelled = false;

    archive.add(original);
    archive.add(cancel);
    ASSERT_EQ(archive.size(), 2u);

    // Before processing cancellations, both are active
    auto active_before = archive.get_active_notices();
    ASSERT_EQ(active_before.size(), 2u);

    // Process cancellations
    archive.process_cancellations();

    // After processing, original should be cancelled
    auto active_after = archive.get_active_notices();
    ASSERT_EQ(active_after.size(), 1u);
    ASSERT_EQ(active_after[0].notice_id, "HYDROLANT 1001/2024");

    // Verify the original is marked cancelled
    auto found = archive.find_by_id("HYDROLANT 1000/2024");
    ASSERT_TRUE(found.has_value());
    ASSERT_TRUE(found.value().is_cancelled);

    PASS();
}

void test_parse_batch() {
    TEST(parse_batch);

    NtmParser parser;
    std::string batch = hydrolant_polygon + "\n" + hydropac_circle;

    auto notices = parser.parse_batch(batch);
    ASSERT_EQ(notices.size(), 2u);

    // First should be the HYDROLANT
    ASSERT_EQ(notices[0].notice_id, "HYDROLANT 1234/2024");
    ASSERT_EQ(notices[0].area, BroadcastArea::HYDROLANT);
    ASSERT_EQ(notices[0].classification, HazardClassification::ROCKET_DEBRIS);

    // Second should be the HYDROPAC
    ASSERT_EQ(notices[1].notice_id, "HYDROPAC 5678/2024");
    ASSERT_EQ(notices[1].area, BroadcastArea::HYDROPAC);
    ASSERT_EQ(notices[1].classification, HazardClassification::FAIRING_RECOVERY);

    PASS();
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== NTM Scraper Plugin Tests ===" << std::endl;
    std::cout << std::endl;

    std::cout << "[NTM Parser]" << std::endl;
    test_parse_hydrolant_polygon();
    test_parse_hydropac_circle();
    test_parse_lat_lon();
    test_parse_nga_datetime();
    test_classify_rocket_debris();
    test_classify_splashdown();
    test_classify_booster_recovery();
    test_classify_fairing_recovery();
    test_extract_polygon_geometry();
    test_extract_circle_geometry();

    std::cout << std::endl;
    std::cout << "[Notice Archive]" << std::endl;
    test_archive_add_query();
    test_archive_time_query();
    test_archive_region_query();
    test_archive_cancellation();
    test_parse_batch();

    std::cout << std::endl;
    std::cout << "=== Results: " << tests_passed << "/" << tests_run
              << " passed ===" << std::endl;

    return (tests_passed == tests_run) ? 0 : 1;
}
