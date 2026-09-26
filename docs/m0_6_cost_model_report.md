# M0.6 Full-Stage Cost Model Report

Empirical measurement of full-stage refinement cost model across 32 stratified gravel cells (§6 M0.6):
- **Terrains:** Open (Crete Senesi), Town (Dreieich), Coastal (Nice), Hilly (Girona)
- **Loop Scales:** 30 km and 100 km
- **Planners:** GREEDY and AUTO
- **Components:** `baseline pricing + init + k × evaluation + finalization + publish`

## Measurement Table

| Category | Region | Dist (km) | Algo | Dir (°) | Base Gen ms | Base Price ms | Init ms | Init Links | Mean Eval ms | Mean Eval Links | Fin ms | Fin Links | Total Refine ms (k=4) | Total Refine Links (k=4) | Farthest (m) | Radius Ratio |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Open | CRETE_SENESI | 30 | GREEDY | 0 | 499 | 2 | 6 | 4070 | 2 | 1104 | 10 | 290 | 26 | 8776 | 9046 | 0.302 |
| Open | CRETE_SENESI | 30 | AUTO | 0 | 499 | 1 | 6 | 4070 | 1 | 965 | 9 | 289 | 20 | 8219 | 9046 | 0.302 |
| Open | CRETE_SENESI | 30 | GREEDY | 180 | 293 | 1 | 5 | 2766 | 2 | 1374 | 6 | 190 | 20 | 8452 | 7237 | 0.241 |
| Open | CRETE_SENESI | 30 | AUTO | 180 | 549 | 1 | 6 | 4629 | 1 | 1259 | 5 | 688 | 16 | 10353 | 5841 | 0.195 |
| Open | CRETE_SENESI | 100 | GREEDY | 0 | 507 | 1 | 24 | 17622 | 4 | 2783 | 29 | 703 | 70 | 29457 | 30167 | 0.302 |
| Open | CRETE_SENESI | 100 | AUTO | 0 | 1293 | 2 | 25 | 21877 | 9 | 8746 | 22 | 1105 | 85 | 57966 | 19995 | 0.200 |
| Open | CRETE_SENESI | 100 | GREEDY | 180 | 461 | 1 | 21 | 17622 | 5 | 3917 | 27 | 683 | 69 | 33973 | 30167 | 0.302 |
| Open | CRETE_SENESI | 100 | AUTO | 180 | - | - | - | - | - | - | - | - | - | - | - | Skip: no_feasible_evaluations |
| Town | DREIEICH | 30 | GREEDY | 90 | 281 | 2 | 9 | 5354 | 3 | 866 | 20 | 3975 | 43 | 12793 | 6778 | 0.226 |
| Town | DREIEICH | 30 | AUTO | 90 | - | - | - | - | - | - | - | - | - | - | - | Skip: no_feasible_evaluations |
| Town | DREIEICH | 30 | GREEDY | 270 | 290 | 5 | 11 | 1713 | 4 | 735 | 15 | 375 | 47 | 5028 | 7363 | 0.245 |
| Town | DREIEICH | 30 | AUTO | 270 | - | - | - | - | - | - | - | - | - | - | - | Skip: no_feasible_evaluations |
| Town | DREIEICH | 100 | GREEDY | 90 | 1445 | 9 | 39 | 23887 | 8 | 4770 | 30 | 1290 | 110 | 44257 | 26038 | 0.260 |
| Town | DREIEICH | 100 | AUTO | 90 | 4433 | 5 | 32 | 16241 | 16 | 5949 | 31 | 1346 | 132 | 41383 | 25245 | 0.252 |
| Town | DREIEICH | 100 | GREEDY | 270 | 1938 | 4 | 28 | 16881 | 6 | 1121 | 34 | 7000 | 90 | 28365 | 24037 | 0.240 |
| Town | DREIEICH | 100 | AUTO | 270 | 3853 | 4 | 28 | 16881 | 8 | 3164 | 26 | 1143 | 90 | 30680 | 24037 | 0.240 |
| Coastal | COASTAL_NICE | 30 | GREEDY | 0 | - | - | - | - | - | - | - | - | - | - | - | Skip: gate_rejected: distance ratio 0.11 outside [0.5, 1.8] |
| Coastal | COASTAL_NICE | 30 | AUTO | 0 | 4678 | 3 | 37 | 30824 | 14 | 11336 | 33 | 1349 | 129 | 77517 | 6069 | 0.202 |
| Coastal | COASTAL_NICE | 30 | GREEDY | 90 | 8324 | 6 | 20 | 14463 | 7 | 5756 | 18 | 594 | 72 | 38081 | 9344 | 0.311 |
| Coastal | COASTAL_NICE | 30 | AUTO | 90 | 13731 | 5 | 21 | 17155 | 11 | 8597 | 40 | 10305 | 110 | 61848 | 5929 | 0.198 |
| Coastal | COASTAL_NICE | 100 | GREEDY | 0 | 6638 | 2 | 59 | 38660 | 11 | 8325 | 69 | 3851 | 174 | 75811 | 22844 | 0.228 |
| Coastal | COASTAL_NICE | 100 | AUTO | 0 | 19850 | 2 | 59 | 38660 | 45 | 49718 | 67 | 3818 | 308 | 241350 | 22844 | 0.228 |
| Coastal | COASTAL_NICE | 100 | GREEDY | 90 | 6488 | 3 | 75 | 58398 | 27 | 19149 | 81 | 1664 | 267 | 136658 | 22844 | 0.228 |
| Coastal | COASTAL_NICE | 100 | AUTO | 90 | 19193 | 7 | 68 | 47547 | 20 | 16305 | 59 | 1764 | 214 | 114531 | 25310 | 0.253 |
| Hilly | GIRONA | 30 | GREEDY | 90 | 665 | 4 | 9 | 3251 | 3 | 1725 | 14 | 494 | 39 | 10645 | 8823 | 0.294 |
| Hilly | GIRONA | 30 | AUTO | 90 | 1346 | 3 | 10 | 3251 | 4 | 751 | 12 | 471 | 41 | 6726 | 8823 | 0.294 |
| Hilly | GIRONA | 30 | GREEDY | 180 | 259 | 3 | 12 | 3791 | 3 | 406 | 14 | 332 | 41 | 5747 | 9077 | 0.303 |
| Hilly | GIRONA | 30 | AUTO | 180 | 677 | 3 | 12 | 3791 | 2 | 250 | 13 | 303 | 36 | 5094 | 9077 | 0.303 |
| Hilly | GIRONA | 100 | GREEDY | 90 | - | - | - | - | - | - | - | - | - | - | - | Skip: no_feasible_evaluations |
| Hilly | GIRONA | 100 | AUTO | 90 | 1415 | 3 | 36 | 22304 | 24 | 27787 | 28 | 997 | 163 | 134449 | 27369 | 0.274 |
| Hilly | GIRONA | 100 | GREEDY | 180 | 760 | 3 | 31 | 20750 | 16 | 12135 | 29 | 995 | 127 | 70285 | 27102 | 0.271 |
| Hilly | GIRONA | 100 | AUTO | 180 | 1747 | 3 | 31 | 20750 | 9 | 4888 | 29 | 1030 | 99 | 41332 | 27102 | 0.271 |

## Component Latency Breakdown

| Stage Component | Mean (ms) | p50 (ms) | p90 (ms) | Max (ms) |
|---|---|---|---|---|
| **Initialization (n legs)** | 26.7 | 24 | 59 | 75 |
| **Evaluation (per proposal)** | 9.8 | 7 | 20 | 45 |
| **Finalization (detail+gate+price)** | 28.5 | 27 | 59 | 81 |
| **Total Refinement (k=4)** | 97.7 | 85 | 174 | 308 |

## Radius Bound Analysis (§4.4, §9)

- **Theoretical circular loop farthest point:** `L / π ≈ 0.318 L`
- **Observed p50 farthest point:** `0.253 L`
- **Observed p95 farthest point:** `0.303 L`
- **Observed Max farthest point:** `0.311 L`
- **Empirical bound selection:** `0.60 L` safely covers all observed real-world loop topologies while preventing runaway dilation.

## Decision D5 Evaluation (≤ 3.0s p90 added latency)

✅ **Passes D5 budget bar:** p90 added latency for k=4 evaluations is **174 ms** (≤ 3,000 ms).
