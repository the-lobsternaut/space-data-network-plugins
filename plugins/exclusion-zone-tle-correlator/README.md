# Exclusion Zone TLE Correlator Plugin

Correlate airspace/maritime exclusion zones with TLE catalog objects. Determines which orbital planes intersect exclusion zone corridors and predicts launch times from ascending node geometry.

## Build

```bash
cd src/cpp && mkdir -p build && cd build && cmake .. && make -j4
```

## Test

```bash
./test_ez_correlator
```
