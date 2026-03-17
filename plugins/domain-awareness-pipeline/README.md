# Domain Awareness Pipeline

Unified intelligence pipeline that orchestrates five domain awareness capabilities into a single situational awareness product. This plugin ties together NOTAM parsing, maritime notice (NTM) parsing, TLE catalog correlation, ascent trajectory reconstruction, and DA-ASAT engagement zone prediction.

## Architecture

The pipeline operates as a 10-step sequential process:

```
Raw NOTAM Text ──┐
                 ├──► Parse NOTAMs ──┐
                 │                   │
Raw NTM Text ────┤                   ├──► Group Exclusion Zones ──► Correlate TLEs
                 ├──► Parse NTMs ────┘         │                        │
                 │                             │                        │
TLE Catalog ─────┴──► Parse TLEs ──────────────┘              ┌────────┘
                                                               │
                 ┌─────────────────────────────────────────────┘
                 │
                 ├──► Generate Trajectory Families (left-of-launch)
                 │
Tracklet ────────┼──► Reconstruct Ascent (right-of-launch)
                 │
                 └──► Assess DA-ASAT Threats
                              │
                              ▼
                   SituationalAwareness Report
```

### Components

| Module | Source Plugin | Purpose |
|--------|-------------|---------|
| `NotamParser` | launch-predict | FAA/ICAO NOTAM parsing and classification |
| `NtmParser` | ntm-scraper | NGA maritime notice parsing |
| `ExclusionZoneAnalyzer` | (combined) | Zone geometry ops, grouping near launch sites |
| `SGP4` | tle-correlator | TLE parsing and orbital propagation |
| `TLECorrelator` | tle-correlator | Zone-to-TLE ground track correlation |
| `TrajectorySimulator` | ascent-reconstruct | RK4-integrated powered flight simulation |
| `AscentEstimator` | ascent-reconstruct | Tracklet-based ascent reconstruction |
| `EngagementAnalyzer` | da-asat-predictor | DA-ASAT feasibility and threat assessment |

### Built-in Launch Site Database

The `ExclusionZoneAnalyzer` includes 12 sites: Cape Canaveral, Kennedy Space Center, Vandenberg, Boca Chica, Wallops, Kodiak, Baikonur, Plesetsk, Kourou, Jiuquan, Wenchang, and Sriharikota.

## Pipeline Steps

1. **Ingest NOTAMs** — parse raw FAA/ICAO NOTAM text, extract coordinates (DMS), radius, altitude, time window. Classify as LAUNCH/REENTRY/HAZARD/AIRSPACE/ROUTINE.
2. **Ingest NTMs** — parse NGA maritime warnings (HYDROLANT/HYDROPAC), extract polygon or circle geometry, classify hazard type.
3. **Ingest TLE Catalog** — parse 3-line TLE format into structured records.
4. **Group Exclusion Zones** — cluster NOTAM and NTM zones near known launch sites within 2000 km.
5. **Estimate Launch Azimuth** — compute bearing from launch site through zone chain centroids.
6. **Correlate TLEs** — propagate catalog TLEs via SGP4, check ground track intersection with exclusion zones.
7. **Generate Trajectory Families** — for active launch sites, produce nominal + dispersed ascent trajectories using a Falcon-9-like vehicle model.
8. **Reconstruct Ascent** — given radar/optical tracklets, estimate launch site, time, azimuth via iterative trajectory fitting.
9. **Assess DA-ASAT Threats** — evaluate direct-ascent intercept feasibility against catalog objects from active launch sites.
10. **Generate Report** — compile all results into a `SituationalAwareness` structure.

## Build Instructions

### Native (testing)

```bash
cd src/cpp
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./test_da_pipeline
```

### WASM

```bash
./build.sh          # first run installs emsdk
./build.sh --clean  # clean rebuild
```

Output lands in `wasm/node/`.

## API Usage

### C++ (Native)

```cpp
#include "da_pipeline/pipeline.h"

da_pipeline::DomainAwarenessPipeline pipeline;
pipeline.set_analysis_window(start_ts, end_ts);

auto report = pipeline.run_full_pipeline(notam_text, ntm_text, tle_catalog);

std::cout << report.summary_text << std::endl;
std::cout << "Active zones: " << report.total_active_zones << std::endl;
std::cout << "Threat level: " << static_cast<int>(report.overall_threat) << std::endl;
```

### Step-by-step usage

```cpp
da_pipeline::DomainAwarenessPipeline pipeline;
pipeline.set_analysis_window(start, end);

pipeline.ingest_notams(notam_text);
pipeline.ingest_ntms(ntm_text);
pipeline.ingest_tle_catalog(catalog);
pipeline.analyze_exclusion_zones();
pipeline.correlate_tle();
pipeline.generate_trajectory_families();

// Right-of-launch (optional)
pipeline.ingest_tracklet(tracklet);

pipeline.assess_asat_threats();
auto report = pipeline.generate_report();
```

### WASM (JavaScript)

```javascript
const module = await require('./wasm/node/da_pipeline.js')();

// Full pipeline
const report = JSON.parse(module.runPipeline(notamText, ntmText, tleCatalog));

// Individual functions
const notams = JSON.parse(module.parseNotams(notamText));
const ntms = JSON.parse(module.parseNtms(ntmText));
const tles = JSON.parse(module.parseTleCatalog(catalogText));
const correlations = JSON.parse(module.correlateZones("28.5,-80.5,100", catalogText));
const family = JSON.parse(module.generateTrajectoryFamily(28.5, -80.5, 90.0));
const threat = JSON.parse(module.assessAsatThreat(28.5, -80.5, catalogText, startTs, endTs));
```

## Data Formats

### NOTAM Input

Standard FAA FDC format or ICAO format. Multiple NOTAMs separated by blank lines.

### NTM Input

NGA HYDROLANT/HYDROPAC format. Multiple notices separated by `NNNN`.

### TLE Catalog Input

Standard 3-line format (name, line1, line2), one object per 3 lines.

### Output

`SituationalAwareness` structure containing parsed inputs, zone groups, TLE correlations, trajectory families, reconstruction results, threat assessments, and a human-readable summary.

## License

MIT
