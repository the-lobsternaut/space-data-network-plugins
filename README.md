# Space Data Network Plugins

Data source plugins for the [Space Data Network](https://github.com/the-lobsternaut/space-data-network).

Each plugin is a standalone C++ WASM module that converts external data sources into
[spacedatastandards.org](https://spacedatastandards.org) FlatBuffers aligned binary format.

**65 plugins** — all managed as git submodules.

## Plugins

### Space Domain Awareness

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [conjunction-assessment](./plugins/conjunction-assessment/) | Satellite conjunction screening and collision probability | CDM |
| [da-asat-predictor](./plugins/da-asat-predictor/) | DA-ASAT engagement zone prediction — intercept trajectories and launch windows | EngagementResult |
| [domain-awareness-pipeline](./plugins/domain-awareness-pipeline/) | Unified intel pipeline: NOTAM/NTM parsing, exclusion zones, TLE correlation, ASAT prediction | SituationalAwareness |
| [drama](./plugins/drama/) | ESA DRAMA-equivalent debris mitigation analysis (OSCAR, ARES, CROC, SARA, MASTER) | LIFETIME, FLUX, XSEC, REENTRY, ENV, COMPLIANCE |
| [exclusion-zone-tle-correlator](./plugins/exclusion-zone-tle-correlator/) | Correlate airspace/maritime exclusion zones with TLE catalog via SGP4 | Correlation |
| [gps-jamming-detection](./plugins/gps-jamming-detection/) | Multi-layer GPS jamming and spoofing detection via signal-level analysis | EPM |
| [satobs](./plugins/satobs/) | Citizen space surveillance observations → orbit determination | OEM |

### Orbit Propagation & Dynamics

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [hpop](./plugins/hpop/) | High-precision orbit propagation — VCM to OCM, full force models | OCM |
| [numerical-propagator](./plugins/numerical-propagator/) | High-fidelity numerical propagator powered by Tudat-WASM | OEM, STM |
| [sgp4-propagator](./plugins/sgp4-propagator/) | SGP4/SDP4 propagator — GP/OMM elements to OEM ephemeris | OEM |
| [od](./plugins/od/) | Orbit determination from observations — batch least squares, EKF | OEM, CDM, TDM |
| [maneuver](./plugins/maneuver/) | Classical orbital maneuver planner (Hohmann, bi-elliptic, patched conic) | OEM, MNV |
| [cislunar](./plugins/cislunar/) | Multi-body CR3BP propagation and cislunar trajectory design | OEM, MNV |
| [small-bodies](./plugins/small-bodies/) | Comets, asteroids, NEOs, PHAs from NASA/ESA/JAXA databases | OEM, OMM |

### Satellite Data & Tracking

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [sdn-starlink](./plugins/sdn-starlink/) | Starlink ephemeris → CCSDS OEM FlatBuffers | OEM |
| [iridium](./plugins/iridium/) | Iridium L-band SIGINT — satellite burst data processing | EPM |
| [satfoot](./plugins/satfoot/) | Satellite RF coverage footprints and sensor swath estimation | REC |
| [linkanalysis](./plugins/linkanalysis/) | Entity relationship mapping and network graph analysis | REC |
| [starfield](./plugins/starfield/) | Star catalog rendering for CesiumJS (magnitude-correct GLSL shaders) | StarCatalog, Attitude |

### Atmospheric & Space Weather

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [space-weather-forecast](./plugins/space-weather-forecast/) | NOAA SWPC 45-day AP and F10.7 forecasts → SPW FlatBuffers | SPW |
| [atmosphere](./plugins/atmosphere/) | US Standard Atmosphere 1976 and NRLMSISE-00 models | ATM |
| [atmospheric-wind](./plugins/atmospheric-wind/) | Open-Meteo GFS wind data at multiple pressure levels | EPM |
| [stereo-wind](./plugins/stereo-wind/) | Dense 3D atmospheric wind from geostationary satellite stereo pairs | EPM |
| [gencast](./plugins/gencast/) | GenCast probabilistic weather forecasting (DeepMind diffusion model) | EPM |
| [bolide](./plugins/bolide/) | Bolide/fireball detection from JPL Fireball API and NASA SkyWatch | OMM |

### Launch & Trajectory

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [launch-predict](./plugins/launch-predict/) | Launch prediction from NOTAMs, TFRs, and maritime notices | LaunchPrediction, TrajectoryEstimate |
| [ascent-reconstruct](./plugins/ascent-reconstruct/) | Reconstruct launch vehicle ascent trajectories from tracklet observations | Trajectory |
| [missile](./plugins/missile/) | General missile simulation (MANPADS to ICBMs) — 3DOF trajectory | OEM, MNV |
| [notam-archive](./plugins/notam-archive/) | Historical NOTAM/TFR archive with pattern detection and launch lead-time tracking | ArchiveResult, Statistics |
| [ntm-scraper](./plugins/ntm-scraper/) | NGA HYDROLANT/HYDROPAC broadcast warnings → structured maritime notices | MaritimeNotice |

### ML & AI

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [ml-inference](./plugins/ml-inference/) | BitNet-style models for real-time satellite telemetry anomaly detection | ANM |
| [ml-train](./plugins/ml-train/) | Train small BitNet neural networks for satellite telemetry anomaly detection | SDNM |
| [autoresearch-astro](./plugins/autoresearch-astro/) | Autonomous astrodynamics research — AI experiment loops for orbit/maneuver/debris | EPM |

### Aircraft & Maritime

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [adsb](./plugins/adsb/) | ADS-B aircraft tracking from OpenSky Network and raw Mode-S data | EPM |
| [ais](./plugins/ais/) | AIS vessel tracking from AISStream, MarineTraffic, Global Fishing Watch | EPM |

### Weather & Environment

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [nws](./plugins/nws/) | NOAA National Weather Service alerts and warnings | EPM |
| [nexrad](./plugins/nexrad/) | NEXRAD WSR-88D Doppler weather radar — Level-II/III volume scans | EPM |
| [tornado](./plugins/tornado/) | Tornado and severe convective weather tracking — NWS/SPC data | EPM |
| [firms](./plugins/firms/) | NASA FIRMS satellite fire/thermal anomaly detection | EPM |
| [radnet](./plugins/radnet/) | EPA RadNet radiation monitoring network | EPM |
| [safecast](./plugins/safecast/) | Citizen-science radiation monitoring (Safecast network) | EPM |

### Geopolitical & Security

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [acled](./plugins/acled/) | Armed Conflict Location & Event Data — battles, protests, violence | EPM |
| [gdelt](./plugins/gdelt/) | GDELT global news events and conflict mapping (100+ languages) | EPM |
| [reliefweb](./plugins/reliefweb/) | UN OCHA ReliefWeb humanitarian crisis tracking | EPM |
| [who_health](./plugins/who_health/) | WHO Disease Outbreak News — outbreaks and health emergencies | EPM |
| [ofac](./plugins/ofac/) | US Treasury OFAC Sanctions (SDN List) | EPM |
| [opensanctions](./plugins/opensanctions/) | Aggregated global sanctions from 30+ sources | EPM |
| [socmint](./plugins/socmint/) | Social Media Intelligence — X/Twitter, TikTok, Telegram, Reddit | EPM |
| [mediameta](./plugins/mediameta/) | Media metadata intelligence — EXIF/IPTC/XMP extraction with geolocation | EPM |
| [netrecon](./plugins/netrecon/) | Network reconnaissance — passive DNS, WHOIS, Certificate Transparency | EPM |
| [webarchive](./plugins/webarchive/) | Web archive intelligence — Wayback Machine change tracking | EPM |
| [photogrammetry](./plugins/photogrammetry/) | Structure-from-Motion for space object 3D reconstruction | EPM |

### Economic & Financial

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [fred](./plugins/fred/) | Federal Reserve Economic Data (22 key indicators) | EPM |
| [treasury](./plugins/treasury/) | US Treasury fiscal data (debt, yields) | EPM |
| [bls](./plugins/bls/) | Bureau of Labor Statistics (CPI, unemployment, payrolls) | EPM |
| [eia](./plugins/eia/) | US Energy Information Administration (crude, gas, inventories) | EPM |
| [gscpi](./plugins/gscpi/) | NY Fed Global Supply Chain Pressure Index | EPM |
| [usaspending](./plugins/usaspending/) | Federal spending and defense contracts | EPM |
| [comtrade](./plugins/comtrade/) | UN Comtrade international trade data | EPM |
| [yfinance](./plugins/yfinance/) | Yahoo Finance live market prices (indexes, crypto, energy) | EPM |

### Telecom & Spectrum

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [fcc](./plugins/fcc/) | FCC filings, spectrum allocations, satellite licenses | EPM |
| [kiwisdr](./plugins/kiwisdr/) | KiwiSDR global HF radio receiver network | EPM |
| [flipper](./plugins/flipper/) | Flipper Zero edge sensor — Sub-GHz RF, NFC/RFID, IR, BLE | EPM |

### Integration

| Plugin | Description | Output Schema |
|--------|-------------|---------------|
| [dimos-bridge](./plugins/dimos-bridge/) | DimOS ↔ SDN bridge — connects plugins to DimOS robot modules | EPM |
| [patents](./plugins/patents/) | USPTO patent filings in strategic tech areas | EPM |

## Architecture

Each plugin is self-contained with:
- Its own `deps/emsdk/` git submodule (Emscripten toolchain)
- CMake build that auto-installs emsdk on first run
- FlatBuffers aligned binary output (not JSON)
- SDN Plugin ABI: `parse()` and `convert()` via `extern "C"` + `EMSCRIPTEN_KEEPALIVE`
- Embind API for Node.js/browser usage

## Building a Plugin

```bash
cd plugins/<name>
# CMake handles emsdk installation automatically on first build
cmake -B build -S src/cpp
cmake --build build
```

Or manually:
```bash
cd plugins/<name>
cd deps/emsdk && ./emsdk install latest && ./emsdk activate latest && cd ../..
source deps/emsdk/emsdk_env.sh
cd src/cpp && emcmake cmake -B build -S . && emmake make -C build
```

## Plugin SDK Compliance

All plugins follow the [SDN Plugin SDK](https://github.com/the-lobsternaut/space-data-network/tree/main/packages/plugin-sdk) contract:

- `plugin-manifest.json` with `schemaVersion: 1`
- FlatBuffers wire format with file identifiers
- WASM exports: `parse`, `convert`, `malloc`, `free`

## Adding a New Plugin

1. Create a new repo at `the-lobsternaut/<plugin-name>-sdn-plugin`
2. Use the plugin structure template (see any existing plugin)
3. Add as submodule: `git submodule add https://github.com/the-lobsternaut/<plugin-name>-sdn-plugin.git plugins/<plugin-name>`
4. Update this README

## License

MIT
