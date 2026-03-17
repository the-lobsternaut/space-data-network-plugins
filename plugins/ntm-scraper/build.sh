#!/usr/bin/env bash
set -euo pipefail
PLUGIN_DIR="$(cd "$(dirname "$0")" && pwd)"
EMSDK_DIR="${PLUGIN_DIR}/deps/emsdk"
CPP_DIR="${PLUGIN_DIR}/src/cpp"
BUILD_DIR="${CPP_DIR}/build"
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log() { echo -e "${GREEN}[build]${NC} $1"; }
warn() { echo -e "${YELLOW}[build]${NC} $1"; }
err() { echo -e "${RED}[build]${NC} $1" >&2; }
if [[ "${1:-}" == "--clean" ]]; then log "Cleaning build directory..."; rm -rf "${BUILD_DIR}"; fi
if [[ ! -f "${EMSDK_DIR}/emsdk" ]]; then log "Initializing emsdk submodule..."; cd "${PLUGIN_DIR}"; git submodule update --init deps/emsdk; fi
if [[ ! -d "${EMSDK_DIR}/upstream/emscripten" ]]; then log "Installing emsdk (first run)..."; cd "${EMSDK_DIR}"; ./emsdk install latest; ./emsdk activate latest; log "emsdk installed successfully."; else log "emsdk already installed."; fi
log "Sourcing emsdk environment..."; export EMSDK_QUIET=1; source "${EMSDK_DIR}/emsdk_env.sh"
SDS_ROOT=""
for candidate in "${PLUGIN_DIR}/../../space-data-network-project" "${HOME}/software/space-data-network-project" "${PLUGIN_DIR}/../space-data-network-project"; do
    if [[ -d "${candidate}/packages/flatbuffers/include" ]]; then SDS_ROOT="$(cd "${candidate}" && pwd)"; break; fi
done
if [[ -z "${SDS_ROOT}" ]]; then warn "space-data-network-project not found."; fi
log "Configuring with emcmake..."
mkdir -p "${BUILD_DIR}"
CMAKE_ARGS=(-B "${BUILD_DIR}" -S "${CPP_DIR}")
if [[ -n "${SDS_ROOT}" ]]; then CMAKE_ARGS+=(-DSDS_ROOT="${SDS_ROOT}"); fi
emcmake cmake "${CMAKE_ARGS[@]}"
log "Building with emmake..."
emmake make -C "${BUILD_DIR}" -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"
WASM_DIR="${PLUGIN_DIR}/wasm/node"
WASM_FILES=($(find "${WASM_DIR}" -name "*.wasm" 2>/dev/null))
if [[ ${#WASM_FILES[@]} -eq 0 ]]; then err "BUILD FAILED: No .wasm files found in ${WASM_DIR}"; exit 1; fi
for f in "${WASM_FILES[@]}"; do SIZE=$(wc -c < "$f" | tr -d ' '); log "Built: $(basename "$f") (${SIZE} bytes)"; done
log "BUILD_SUCCESS"
