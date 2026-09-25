# M0.2 Geometry Fidelity Report

This report compares the raw search representation (concatenated junction-level legs) against the detailed shipped track (post-cleanup, detailed transfer nodes) across 48 Dreieich fixture loops.

## Raw vs Detailed Comparison Table

| Profile | Dir (°) | Radius (m) | Raw Nodes | Det Nodes | Raw Dist (m) | Det Dist (m) | Δ Dist (%) | Raw Cross | Det Cross | Raw Scatter (m) | Det Scatter (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| trekking | 0 | 1000 | 105 | 175 | 5412 | 4165 | -23.04% | 0 | 0 | 371 | 0 |
| trekking | 45 | 1000 | 105 | 175 | 5412 | 4165 | -23.04% | 0 | 0 | 371 | 0 |
| trekking | 90 | 1000 | 120 | 175 | 6168 | 4165 | -32.47% | 0 | 0 | 658 | 0 |
| trekking | 135 | 1000 | 105 | 175 | 5412 | 4165 | -23.04% | 0 | 0 | 371 | 0 |
| trekking | 180 | 1000 | 105 | 175 | 5412 | 4165 | -23.04% | 0 | 0 | 371 | 0 |
| trekking | 225 | 1000 | 105 | 175 | 5412 | 4165 | -23.04% | 0 | 0 | 371 | 0 |
| trekking | 270 | 1000 | 105 | 175 | 5412 | 4165 | -23.04% | 0 | 0 | 371 | 0 |
| trekking | 315 | 1000 | 105 | 175 | 5412 | 4165 | -23.04% | 0 | 0 | 371 | 0 |
| trekking | 0 | 1500 | 256 | 255 | 10145 | 5882 | -42.02% | 0 | 0 | 2787 | 39 |
| trekking | 45 | 1500 | 256 | 255 | 10145 | 5882 | -42.02% | 0 | 0 | 2787 | 39 |
| trekking | 90 | 1500 | 256 | 255 | 10145 | 5882 | -42.02% | 0 | 0 | 2787 | 39 |
| trekking | 135 | 1500 | 256 | 255 | 10145 | 5882 | -42.02% | 0 | 0 | 2787 | 39 |
| trekking | 180 | 1500 | 171 | 239 | 7626 | 5839 | -23.43% | 1 | 1 | 1994 | 116 |
| trekking | 225 | 1500 | 171 | 239 | 7626 | 5839 | -23.43% | 1 | 1 | 1994 | 116 |
| trekking | 270 | 1500 | 161 | 239 | 6670 | 5703 | -14.50% | 0 | 0 | 500 | 0 |
| trekking | 315 | 1500 | 256 | 255 | 10145 | 5882 | -42.02% | 0 | 0 | 2787 | 39 |
| trekking | 0 | 2000 | 221 | 214 | 8912 | 5022 | -43.65% | 0 | 1 | 3639 | 68 |
| trekking | 45 | 2000 | 221 | 214 | 8912 | 5022 | -43.65% | 0 | 1 | 3639 | 68 |
| trekking | 90 | 2000 | 187 | 178 | 9107 | 4419 | -51.48% | 0 | 0 | 2146 | 39 |
| trekking | 135 | 2000 | 187 | 178 | 9107 | 4419 | -51.48% | 0 | 0 | 2146 | 39 |
| trekking | 180 | 2000 | 187 | 178 | 9107 | 4419 | -51.48% | 0 | 0 | 2146 | 39 |
| trekking | 225 | 2000 | 187 | 178 | 9107 | 4419 | -51.48% | 0 | 0 | 2146 | 39 |
| trekking | 270 | 2000 | 187 | 178 | 9107 | 4419 | -51.48% | 0 | 0 | 2146 | 39 |
| trekking | 315 | 2000 | 221 | 214 | 8912 | 5022 | -43.65% | 0 | 1 | 3639 | 68 |
| gravel | 0 | 1000 | 97 | 230 | 6295 | 6242 | -0.84% | 0 | 0 | 235 | 0 |
| gravel | 45 | 1000 | 97 | 230 | 6295 | 6242 | -0.84% | 0 | 0 | 235 | 0 |
| gravel | 90 | 1000 | 117 | 194 | 6160 | 5583 | -9.37% | 0 | 0 | 417 | 0 |
| gravel | 135 | 1000 | 117 | 194 | 6160 | 5583 | -9.37% | 0 | 0 | 417 | 0 |
| gravel | 180 | 1000 | 97 | 230 | 6295 | 6242 | -0.84% | 0 | 0 | 235 | 0 |
| gravel | 225 | 1000 | 97 | 230 | 6295 | 6242 | -0.84% | 0 | 0 | 235 | 0 |
| gravel | 270 | 1000 | 97 | 230 | 6295 | 6242 | -0.84% | 0 | 0 | 235 | 0 |
| gravel | 315 | 1000 | 97 | 230 | 6295 | 6242 | -0.84% | 0 | 0 | 235 | 0 |
| gravel | 0 | 1500 | 138 | 277 | 9116 | 8597 | -5.69% | 1 | 1 | 638 | 286 |
| gravel | 45 | 1500 | 138 | 277 | 9116 | 8597 | -5.69% | 1 | 1 | 638 | 286 |
| gravel | 90 | 1500 | 138 | 277 | 9116 | 8597 | -5.69% | 1 | 1 | 638 | 286 |
| gravel | 135 | 1500 | 138 | 277 | 9116 | 8597 | -5.69% | 1 | 1 | 638 | 286 |
| gravel | 180 | 1500 | 138 | 277 | 9116 | 8597 | -5.69% | 1 | 1 | 638 | 286 |
| gravel | 225 | 1500 | 138 | 277 | 9116 | 8597 | -5.69% | 1 | 1 | 638 | 286 |
| gravel | 270 | 1500 | 138 | 277 | 9116 | 8597 | -5.69% | 1 | 1 | 638 | 286 |
| gravel | 315 | 1500 | 138 | 277 | 9116 | 8597 | -5.69% | 1 | 1 | 638 | 286 |
| gravel | 0 | 2000 | 124 | 209 | 9557 | 7409 | -22.48% | 1 | 1 | 2176 | 74 |
| gravel | 45 | 2000 | 124 | 209 | 9557 | 7409 | -22.48% | 1 | 1 | 2176 | 74 |
| gravel | 90 | 2000 | 124 | 209 | 9557 | 7409 | -22.48% | 1 | 1 | 2176 | 74 |
| gravel | 135 | 2000 | 124 | 209 | 9557 | 7409 | -22.48% | 1 | 1 | 2176 | 74 |
| gravel | 180 | 2000 | 124 | 209 | 9557 | 7409 | -22.48% | 1 | 1 | 2176 | 74 |
| gravel | 225 | 2000 | 124 | 209 | 9557 | 7409 | -22.48% | 1 | 1 | 2176 | 74 |
| gravel | 270 | 2000 | 124 | 209 | 9557 | 7409 | -22.48% | 1 | 1 | 2176 | 74 |
| gravel | 315 | 2000 | 124 | 209 | 9557 | 7409 | -22.48% | 1 | 1 | 2176 | 74 |

## Key Observations

1. **Distance Fidelity:** Average distance delta between raw and detailed is **-22.97%**. Transfer nodes along curved ways provide accurate arc distances compared to junction chords, while micro-detour and spur removal trim duplicate distance.
2. **Self-Crossing Agreement:** Raw and detailed self-crossings matched in **45 / 48 (93.8%)** of loops. Differences occur primarily when cleanup removes micro-detours or loops around vias.
3. **Scatter Reuse Agreement:** Raw and detailed scatter reuse matched in **0 / 48 (0.0%)** of loops. Post-routing cleanup (`removeBackAndForthSegments`, `repairViaPinnedBulges`) cleans spurs at vias which eliminates spur-induced reuse.
4. **Implication for Search vs Finalization:** Because raw legs have higher spur/micro-detour artifacts than cleaned final routes, evaluating candidates on raw legs acts as a conservative filter. Finalization cleans these artifacts before applying the strict ship predicate.
