# Did the loop algorithm improve?

The work improved specific correctness defects and made the evaluation trustworthy enough to expose a weak product result. It has not demonstrated better rides for most cyclists.

[Open the complete route report](../brouter-core/build/refine-evaluation-12687333987490758169/complete-review/index.html).

## Why A and B looked identical

426 of the 460 paired requests returned exactly the original geometry. This is expected when no candidate passes all publication checks, the search times out, the producing tier is unsupported, or pricing fails. The two files are retained so these outcomes remain in the experiment rather than disappearing from the denominator.

Safari was also still displaying the earlier six-case preview during this follow-up. All six pairs in that preview were unchanged. The new report opens the complete corrected experiment and puts changed pairs first, alphabetically. It labels identical pairs explicitly.

## What I actually improved

| Claim | What supports it | What it does not prove |
|---|---|---|
| Safer cleanup | The repaired code no longer treats nearby roads as interchangeable junctions; regression tests cover the fabricated joins and turn restrictions | Every generated route is valid. Additional construction/pricing failures remain |
| More faithful pricing | Original and finished geometry use continuous profile state, native elevation interpolation and native turn restrictions; all six original production regressions pass | Cyclists prefer a lower-cost route |
| A usable comparison | All 460 paired requests, failures and truncations are recorded, with immutable geometry and coverage-aware measurements | A controlled before-fix versus after-fix route-quality result. Both variants here use the correctness repairs |
| Some real route changes | 34 routes pass the existing cost and publication checks, with actual geometry changes | A generally better generator. 92.6% of requests are unchanged and the product bar fails |

The 34 routes improved under the chosen profile-cost criterion, and their target-distance fit improved. Other properties moved in both directions. The first changed route in alphabetical order, Annecy 100 km fastbike north, improves distance fit but adds about 5.6 km of observed high-traffic exposure. I would not call that an obvious cyclist win merely because its cost per metre fell by 2.04%.

My expectation was that fixing the pricing blockers would let the search run and reveal whether it was useful. That expectation was met. I hoped it would find enough appealing alternatives within its three-second allowance to justify enabling it. The evidence did not meet that expectation. I still recommend keeping refinement off by default.

Before calling the route generator meaningfully better, I would want accepted changes on many more requests, a manageable timeout rate, visible rider-relevant benefits with adequate data coverage, and cyclists choosing the new routes in blind comparisons. Those are expectations for further evidence, not new optimization objectives or retroactive acceptance rules.

## What the new report contains

- Every current request: 460 pairs and 920 original/result variants, with labelled GPX links, shared-map overlays, elevation profiles, all recorded metrics and candidate score traces.
- Exact identity labels for the 426 unchanged pairs. The changed-pair filter exposes all 34, with no ranking by gain.
- Orange original-only sections, blue result-only sections and grey shared geometry. Pan, zoom and fit-to-change controls make small differences easier to locate.
- A separate archive of 748 historical recordings across 562 route labels, from the combined loop report and its saved variant tracks. It contains AUTO, GREEDY, ISO_GREEDY, probe and isochrone recordings where available. Variant checkboxes overlay recordings on shared coordinates.
- Historical file provenance and hashes. Those runs use different settings and dates, so they are not evidence for refinement improvements.

The geometry highlight uses a 10 m corridor with samples spaced at most 25 m apart. It does not identify physical graph edges or distinguish traversal direction. The exact-identity label compares the complete captured point sequences separately.

Rejected search candidates have score traces but no saved geometry. They cannot be mapped retrospectively. No new routing experiment was run for this presentation, and the original evidence was left unchanged.

## Rebuild and validation

```sh
python3 misc/scripts/build-refinement-review.py \
  brouter-core/build/refine-evaluation-12687333987490758169 \
  --archive brouter-core/build/reports/loops
```

The written interpretation is specific to the corrected experiment. The generator rejects different outcome totals so an unrelated run cannot silently inherit these conclusions. The source templates are in `misc/scripts/refinement-review/`.

The report loads one case at a time and works offline. Assets remain under `build/`, so preserve them before a clean build. See `report-manifest.json` for evidence hashes and `validation.json` for the export checks. The original paired GPX files remain unchanged.
