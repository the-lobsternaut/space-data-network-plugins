# NOTAM Historical Archive Plugin

Historical NOTAM/TFR archive with temporal queries, pattern change detection, launch lead time tracking, and statistical analysis by site, vehicle, and orbit type.

## Features

### Archive Storage and Queries
- Store parsed NOTAMs with full metadata (timestamps, location, altitude, classification)
- Query by time range (overlapping effective windows)
- Query by geographic location using haversine distance
- Query by classification (LAUNCH, REENTRY, HAZARD, AIRSPACE, ROUTINE)
- Query by site location ID
- Query for NOTAMs active at a specific timestamp

### Pattern Change Detection
- Detect when a new NOTAM replaces an existing one for the same site
- Categorize changes: REPLACEMENT, EXTENSION, NARROWING, CANCELLATION
- Identify scrub indicators (NARROWING or CANCELLATION of launch NOTAMs)
- Identify reschedule indicators (EXTENSION or large time shifts)
- Track the full change history for each site

### Launch Lead Time Tracking
- Record the time between NOTAM issuance and the associated launch
- Filter lead times by site, vehicle, or orbit type
- Detect trends in lead time over time

### Statistical Analysis
- Per-site statistics: total NOTAMs, launch NOTAMs, average/median/stddev lead time, scrub rate
- Per-vehicle statistics: average lead time, average NOTAMs per launch, scrub rate
- Per-orbit-type statistics: average and median lead times
- Overall archive statistics with aggregated metrics
- Activity timeline: count of active NOTAMs over a time period
- Anomaly detection: unusual lead times (>3 sigma), unusual NOTAM counts per launch

## Query API (WASM)

### Archive Operations
- `wasm_add_notam(json)` -- Add a NOTAM record (JSON string)
- `wasm_query_by_time(start, end)` -- Query NOTAMs overlapping a time range
- `wasm_query_by_location(lat, lon, radius_nm)` -- Query NOTAMs within radius
- `wasm_query_by_classification(classification)` -- Filter by classification string
- `wasm_query_active_at(time)` -- NOTAMs active at a specific timestamp
- `wasm_detect_changes()` -- Detect pattern changes across all NOTAMs

### Lead Time Operations
- `wasm_add_launch(json)` -- Add a launch record and compute lead times
- `wasm_get_lead_times()` -- Retrieve all lead time records

### Statistics
- `wasm_compute_statistics()` -- Full archive statistics
- `wasm_site_statistics(site_id)` -- Statistics for a specific site
- `wasm_vehicle_statistics(vehicle)` -- Statistics for a specific vehicle
- `wasm_activity_timeline(start, end, step_hours)` -- Activity over time
- `wasm_detect_anomalies()` -- Find anomalous patterns

### Utility
- `wasm_archive_size()` -- Number of stored NOTAMs
- `wasm_clear()` -- Clear the archive

## Build

Requires [Emscripten SDK](https://emscripten.org/).

```bash
./build.sh          # Build WASM module
./build.sh --clean  # Clean and rebuild
```

### Native Build (for testing)

```bash
cd src/cpp
mkdir -p build && cd build
cmake ..
make
./test_notam_archive
```

## Example Usage

### Adding NOTAMs
```json
{
  "notam_id": "A0001/26",
  "location_id": "KXMR",
  "issued": 1710500000,
  "effective_start": 1710600000,
  "effective_end": 1710614400,
  "altitude_floor_ft": 0,
  "altitude_ceiling_ft": -1.0,
  "center_lat": 28.396837,
  "center_lon": -80.605659,
  "radius_nm": 30,
  "notam_text": "SPACE OPERATIONS - LAUNCH",
  "classification": "LAUNCH"
}
```

### Querying by Location
```
wasm_query_by_location(28.4, -80.6, 50.0)
```
Returns all NOTAMs whose center is within 50 nautical miles of the given point.

### Detecting Scrubs
When a NOTAM for a launch site is narrowed or cancelled, `detect_changes()` flags it as a potential scrub. The `is_scrub_indicator` field in the change record will be true.

## Data Sources

This plugin archives NOTAMs from any upstream parser. Typical sources include:
- FAA NOTAM API
- ICAO NOTAM database
- TFR feeds (temporary flight restrictions associated with launches)

## License

MIT
