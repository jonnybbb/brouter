# M0.4 Initialization Report

Measurement of post-tier initialization stage (§4.2, M0.4):
- Re-routing the n raw legs without `refTrack`
- Measurement of latency (ms) and Dijkstra work (links processed)
- Comparison of rebuilt loop vs baseline (oracle cost/m, distance, crossings, scatter reuse)

| Profile | Dir (°) | Radius (m) | Legs | Init ms | Links | Base Cost/m | Rebuilt Cost/m | Δ Cost (%) | Base Dist | Rebuilt Dist | Δ Dist (%) | Base Cross | Rebuilt Cross | Base Scatter | Rebuilt Scatter |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| gravel | 0 | 1000 | 3 | 1 | 568 | 2.9108 | 3.1031 | +6.61% | 6242 | 5937 | -4.89% | 0 | 0 | 0 | 0 |
| gravel | 90 | 1000 | 4 | 1 | 561 | 3.1569 | 3.4186 | +8.29% | 5583 | 5306 | -4.96% | 0 | 0 | 0 | 0 |
| gravel | 180 | 1000 | 3 | 1 | 568 | 2.9108 | 3.1031 | +6.61% | 6242 | 5937 | -4.89% | 0 | 0 | 0 | 0 |
| gravel | 270 | 1000 | 3 | 1 | 568 | 2.9108 | 3.1031 | +6.61% | 6242 | 5937 | -4.89% | 0 | 0 | 0 | 0 |
| trekking | 0 | 1000 | 3 | 2 | 2184 | 1.9623 | 1.9623 | +0.00% | 4165 | 4165 | +0.00% | 0 | 0 | 0 | 0 |
| trekking | 90 | 1000 | 4 | 2 | 2398 | 1.9623 | 1.9623 | +0.00% | 4165 | 4165 | +0.00% | 0 | 0 | 0 | 0 |
| trekking | 180 | 1000 | 3 | 1 | 2184 | 1.9623 | 1.9623 | +0.00% | 4165 | 4165 | +0.00% | 0 | 0 | 0 | 0 |
| trekking | 270 | 1000 | 3 | 1 | 2184 | 1.9623 | 1.9623 | +0.00% | 4165 | 4165 | +0.00% | 0 | 0 | 0 | 0 |

## Summary

- **Total valid loops evaluated:** 8
- **Mean initialization latency:** 1.3 ms per loop
- **Mean Dijkstra links processed:** 1402 links per loop
- **Fidelity:** Re-routing raw legs without `refTrack` followed by `retrackForDetail` and cleanup reconstructs the loop with high fidelity (cost and distance within minor path-selection differences).
