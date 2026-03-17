# DA-ASAT Engagement Zone Predictor

Model direct-ascent anti-satellite engagement zones. Given a launch site and target satellite TLEs, compute intercept geometry, launch azimuths, delta-v requirements, and feasibility windows.

## Build

```bash
cd src/cpp
mkdir -p build && cd build
cmake ..
make -j4
./test_da_asat
```

## Architecture

- **Propagator** — Simplified SGP4-style orbit propagation with J2 perturbations and Bstar drag
- **Interceptor** — 3DOF DA-ASAT trajectory model with staged boost, atmospheric drag, and proportional navigation guidance
- **EngagementCalculator** — Engagement zone computation, feasibility analysis, and window finding

## License

MIT
