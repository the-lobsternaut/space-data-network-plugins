# NTM Scraper Plugin

Parse NGA HYDROLANT/HYDROPAC broadcast warnings into structured maritime notices with exclusion zone geometry and hazard classification.

## Overview

This plugin ingests raw NGA (National Geospatial-Intelligence Agency) maritime broadcast warnings — HYDROLANT (Atlantic) and HYDROPAC (Pacific) — and extracts structured data including exclusion zone geometry (polygons and circles), time windows, hazard classifications, and cancellation references. Designed for correlating maritime closures with space launch operations.

## Inputs

| Port | Schema | Required | Description |
|------|--------|----------|-------------|
| `ntm_text` | NTMText | Yes | Raw NGA broadcast warning text |

## Outputs

| Port | Schema | Description |
|------|--------|-------------|
| `maritime_notices` | MaritimeNotice | Structured maritime notices with geometry and classification |

## Building

```bash
# Native build (for testing)
cd src/cpp && mkdir -p build && cd build && cmake .. && make -j4
./test_ntm_scraper

# WASM build
./build.sh
```

## License

MIT
