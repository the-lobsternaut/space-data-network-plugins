# Space Data Network Plugins

Data source plugins for the [Space Data Network](https://github.com/the-lobsternaut/space-data-network).

Each plugin is a standalone C++ WASM module that converts external data sources into
[spacedatastandards.org](https://spacedatastandards.org) FlatBuffers aligned binary format.

## Plugins

| Plugin | Schema | Data Source | Status |
|--------|--------|-------------|--------|
| [space-weather-forecast](./plugins/space-weather-forecast/) | SPW | NOAA SWPC 45-day forecast | ✅ Working |
| [sdn-starlink](./plugins/sdn-starlink/) | OEM | Starlink ephemeris | 🔧 In progress |
| [starlink-oem-wasm](./plugins/starlink-oem-wasm/) | OEM | Starlink → CCSDS OEM | 🔧 In progress |

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
# Install emsdk if needed
cd deps/emsdk && ./emsdk install latest && ./emsdk activate latest && cd ../..
# Build
source deps/emsdk/emsdk_env.sh
cd src/cpp && emcmake cmake -B build -S . && emmake make -C build
```

## Plugin SDK Compliance

All plugins follow the [SDN Plugin SDK](https://github.com/the-lobsternaut/space-data-network/tree/main/packages/plugin-sdk) contract:

- `plugin-manifest.json` with `schemaVersion: 1`
- FlatBuffers wire format with file identifiers
- WASM exports: `parse`, `convert`, `malloc`, `free`

## Adding a New Plugin

1. Create a new repo at `the-lobsternaut/<plugin-name>`
2. Use the plugin structure template (see any existing plugin)
3. Add as submodule: `git submodule add https://github.com/the-lobsternaut/<plugin-name>.git plugins/<plugin-name>`

## License

MIT
