# Findings: clean-teardrop self-intersections ship because the shape penalty only counts transverse crossings

Date: 2026-06-08. Supersedes the anti-reuse premise (see `ANTI-REUSE-RETURN-FIX-SCOPE.md` ⚠️UPDATE).
User picked this path: "candidate selection / self-intersection — why via3=Ettingen + why the teardrop shipped."

## Root cause (confirmed, prevent-at-source)

The planner ranks accepted loops with `RouteChoiceScore`, which since commit **d2bbea4c** penalises
self-intersections — but it reads `RoundTripQualityGate.countSelfIntersections`, which counts only
**transverse X-crossings** (`segmentsCross` = CCW orientation test, requires the two segments to
properly straddle). A **teardrop** — the route returns to within ~60 m of an earlier point and runs
*alongside* it (a pinch / "small loop back to the same point"), going a genuinely different way out
vs back — never transversely crosses. So `countSelfIntersections = 0`, the penalty is 0, and the
teardrop ships unopposed.

This is exactly the "small loop that comes back to the same point" the user named. It is a real
shape defect the cyclist sees, invisible to the current detector.

## Evidence (probe: `LoopWegendorfTraceTest.probeElasticArchitecture`)

Basel (region BASEL, dir 180, radius 4800 → reqDist ≈ 30159 m):

| Variant | dist | distR | nearRevisit | transverse xings | gate | RCS score |
|---|---|---|---|---|---|---|
| ISO_GREEDY (shipped) | 30596 m | 1.014 | **1 (7334 m teardrop)** | **0** | ACCEPT | **0.9080** |
| GREEDY (clean) | 26269 m | 0.871 | 0 | 0 | ACCEPT | 0.7987 |

- The teardrop tip = committed waypoint **via3 = Ettingen** (47.48213, 7.54747).
- Out vs back: **3 % shared node-IDs, 6/79 within 60 m** → a true teardrop (different roads), not a retrace.
- Both variants are **gate-accepted**; AUTO ships ISO purely on RCS (0.908 > 0.799).
  **RCS gap = 0.109** → a near-revisit ranking penalty > 0.109 flips AUTO to the clean GREEDY.

Berlin (URBAN_BERLIN, dir 90, radius 15915 → 100 km), the Wegendorf spur:

| Variant | dist | distR | nearRevisit | transverse xings | RCS score |
|---|---|---|---|---|---|
| ISO_GREEDY (spur) | 96022 m | 0.960 | 1 (3243 m) | **2** | 0.6928 |
| GREEDY (clean) | 96283 m | 0.963 | 0 | 0 | 0.8761 |

- Berlin's spur is a **partial retrace** (36 % shared node-IDs) that DOES produce 2 transverse crossings,
  so **d2bbea4c already catches it** — GREEDY (0.876) already outranks ISO (0.693) under AUTO.
  Berlin is the matrix's per-variant view; AUTO already picks clean. **Not** a near-revisit-penalty case.

⇒ The new penalty is specifically needed for **clean teardrops (xings = 0)** like Basel that d2bbea4c
can't see. The two cases are distinct sub-types, not one unified cause.

## The fix (mirrors d2bbea4c)

Add the **near-revisit count** (`LoopQualityMetrics.nearRevisitSpans`, eps 60 / minArc 600 / maxArc 10 km
— the 10 km cap matters; `computeSpurInfo`'s 6 km misses Basel's 7.3 km) to `RouteChoiceScore` as a
**ranking-only shape term** (in `score()`, excluded from `qualityScore()` — same split as the crossing
penalty). Ranking-only ⇒ never gates ⇒ no no-route risk; only flips a pick when a cleaner alternative
exists, so the ~29 % near-revisit fire rate that killed the *gate* approach does NOT bite here.

**Weight: calibrate to the corpus, NOT to Basel.** d2bbea4c uses 0.08/crossing. Basel's gap is 0.109,
so ~0.11+ flips it — but sizing to one case is the inflation trap. Measure the corpus and pick the
value that cuts shipped clean-teardrops without over-promoting shorter/worse-distance loops.

## The Basel tradeoff (must be surfaced — not a silent win)

The clean GREEDY is **13 % shorter** (26.3 vs 30.6 km) — the teardrop is "load-bearing" for Basel's
distance (the Ettingen excursion is how ISO hits ~30 km). And **neither** variant reaches the gravel
the user wanted: min-dist Biel-Benken 1076 m (GREEDY) / 1453 m (ISO), Neuwiller ~4 km both. The fix
removes the Ettingen teardrop (the literal complaint) but yields a shorter, still-not-scenic loop.
Delivering Biel-Benken/Neuwiller gravel is **desirability #15**, which the user deferred.

## Validation plan (d2bbea4c pattern)

1. Implement the ranking-only near-revisit term in `RouteChoiceScore.score()`.
2. Corpus A/B (offline AUTO sim over the routed matrix): shipped-clean-teardrop % before/after; MUST
   explicitly flag "penalty promoted a shorter / worse-distance loop" (the Basel failure mode), not
   just "teardrops down". Re-baseline goldens.
3. Tune the weight to the corpus min-teardrop-without-over-promotion point.

## Implementation + validation (2026-06-08, user chose "implement + corpus A/B gate")

**Implemented** `RouteChoiceScore` shape term #10 (`SHAPE_PENALTY_PER_TEARDROP`, default 0.12,
`-Dloop.teardroppenalty`): severity = Σ min(1, arc / 5000) over near-revisit spans (closure
excluded, >85% of perimeter); **ranking-only**, added to `score()` and **added back into
`qualityScore()`** so the soft `MIN_RCS_PASS` floor is byte-identical (a forced teardrop shipping
as best-available is not failed). Helper `teardropSeverity`. Fast unit tests
(AutoCompetition/Gate/Metrics/GreedyPerf) all pass.

**End-to-end AUTO proof** (the real `runAutoCandidateCompetition` flow, not an offline sim):
the penalty drops Basel's ISO 0.908→0.788, *below* `CLEAR_ACCEPT_THRESHOLD=0.85`, which both
triggers GREEDY routing and loses to GREEDY (0.799):

| Basel AUTO | shipped dist | teardrop |
|---|---|---|
| penalty 0 (baseline) | 30596 m | yes (7334 m) |
| penalty 0.12 | 26269 m | **none** |

(Note: 0.10 would route GREEDY but ISO 0.808 still *beats* GREEDY 0.799 → no flip; the 0.109
RCS gap genuinely requires >0.11.)

**Corpus A/B** (`corpusTeardropAB`, 30 cells = 4 regions × 8 dirs, gravel/30 km, weight swept;
offline AUTO sim replicating the 0.85-threshold gate):

| weight | teardrops shipped | flips | flips→worse-dist | avg distR | avg shipped sev |
|---|---|---|---|---|---|
| 0.00 | 26% (8/30) | 0 | 0 | 0.969 | 0.260 |
| 0.08 | 13% (4/30) | 5 | 3 | 0.956 | 0.112 |
| **0.12** | **10% (3/30)** | 6 | 4 | 0.952 | 0.079 |
| 0.15 / 0.20 | 10% (3/30) | 6 | 4 | 0.952 | 0.079 |

Teardrops **26%→10%** (mirrors d2bbea4c's 12.8%→3.7%); the 3 residuals are genuine (both variants
teardrop, or the alternative gate-rejected). W=0.12 is the knee (0.08→0.12 buys exactly the Basel
Ettingen removal; ≥0.12 adds nothing). avgDistR drops only 0.969→0.952.

**All 4 worse-distance flips verified GENUINE teardrops** (advisor's blocking check — the detector
over-fires, so the only place harm could occur is a benign near-pass demoted for a shorter loop):
- BASEL d180 & d270 = the Ettingen teardrop (7334 m, 3% retrace).
- FREIBURG d45 = 4310 m partial-lollipop (28% retrace); flips to GREEDY which has *higher* base RCS
  (0.92 vs 0.87) AND a smaller teardrop (sev 0.37) — a net-better loop.
- ANNECY d90 = 4718 m clean teardrop (8% retrace, 11/88 within 60 m) — not noise.
By construction the penalty only ever demotes a teardrop variant, so no clean loop is ever wrongly
promoted; the 4 "worse-distance" flips are the inherent "teardrop added distance, removing it
shortens" tradeoff, all on real defects.

**Broader A/B (fastbike / 30 km + gravel / 50 km, same 4 regions):** the failure-mode check holds
across profiles and distances — every worse-distance flip is again a genuine teardrop removal
(fastbike: BASEL d180 sev0.77→clean, ANNECY d90 sev0.50→clean; gravel-50 km: BASEL d0 sev1.25→clean),
no benign loop promoted.

**Golden re-baseline = NO-OP (verified).** The fix only changes AUTO's *cross-variant* pick;
`GreedyRoundTripPlanner` never references `RouteChoiceScore`, and both golden suites route **forced**
algorithms (`LoopGoldenSignatureTest` = GREEDY/ISO_GREEDY scenarios; `LoopGoldStandardTest` =
`loop.algorithms=probe,greedy`). `LoopGoldenSignatureTest` (byte-identical node-sequence hashes)
**PASSES unchanged** in verify mode → forced-algorithm output is untouched, so there are no moved
goldens to re-baseline. `qualityScore()` is byte-identical, so the matrix `MIN_RCS_PASS` floor is also
unaffected. Nothing in the suite asserts AUTO's pick, so the entire integrationTest suite stays green
without recapture.

**Matrix confirmed green by execution (not reasoning).** The matrix (`LoopQualityTestBase`) grades each
algorithm **per-variant** via `runVariant` (forced GREEDY/ISO_GREEDY/WAYPOINT/ISOCHRONE, lines 143–146),
not the AUTO winner — so AUTO-pick changes can't trip its distance/reuse/spur bars. `LoopQualityBaselTest`
(Basel = exactly where AUTO picks flip: dir 180/225/270) **PASSES all cells** (fastbike + gravel ×
30/50/100 km; mtb skipped pre-existing) with the penalty on. Plus fast unit tests
(AutoCompetition/Gate/Metrics/GreedyPerf) and `LoopGoldenSignatureTest` green. Full validation complete;
**no golden recapture required.**

**Remaining limitation (not a defect of this fix):** getting a clean loop at the *requested* distance
(vs the shorter clean variant) where the teardrop was load-bearing for distance is a separate problem
(#15 / candidate diversity), not this penalty.

## Artifacts kept

- `LoopQualityMetrics.nearRevisitSpans` / `hasNearRevisit` (main tree).
- `LoopWegendorfTraceTest.probeElasticArchitecture` (both-variants discriminating run + `routeP2P`,
  `writeLoopHtml`; HTML: /tmp/{region}-{ISO_GREEDY,GREEDY}.html, /tmp/basel-teardrop.html).
- No `RoutingContext`/`OsmPath`/planner changes (measure-first; the anti-reuse lever was never built).
