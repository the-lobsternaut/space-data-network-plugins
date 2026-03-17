# Ascent Reconstruct Plugin

Reconstruct launch vehicle ascent trajectories from radar/optical tracklet observations using powered flight model fitting and staging event detection.

## Overview

This plugin takes time-tagged position observations (tracklets) from radar or optical sensors and reconstructs the full ascent trajectory of a launch vehicle. It includes:

- **Trajectory Simulation**: RK4-integrated powered flight model with gravity turn, atmospheric drag, J2 perturbation, and Earth rotation effects
- **Staging Detection**: Identifies staging events from acceleration profile discontinuities
- **Launch Site Estimation**: Extrapolates early observations backward to estimate the launch site
- **Launch Time Estimation**: Projects altitude backward to estimate when the vehicle left the pad
- **Trajectory Reconstruction**: Iterative Gauss-Newton optimization to fit model parameters to observations

## Building

### Native (for testing)

```bash
cd src/cpp
mkdir -p build && cd build
cmake .. && make -j$(nproc)
./test_ascent_reconstruct
```

### WASM (for deployment)

```bash
./build.sh
```

## Ports

### Inputs

| Port | Schema | Description |
|------|--------|-------------|
| `tracklet` | `Tracklet` | Time-tagged position observations from radar/optical sensors |

### Outputs

| Port | Schema | Description |
|------|--------|-------------|
| `trajectory` | `Trajectory` | Reconstructed ascent trajectory with state vectors |
