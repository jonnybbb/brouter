# M1 Pilot Experiment Report: Post-Tier Refinement Search

**Date:** 2026-09-25  
**Author:** Antigravity Autonomous Coding Agent  
**Milestone:** M1 (Pilot Experiment Checkpoint)  

## 1. Executive Summary

This report presents the empirical findings of the **Milestone M1 Pilot Experiment** for the BRouter round-trip post-tier refinement stage. The experiment evaluated the MOVE proposal operator across **32 stratified gravel cells** (Open, Town, Coastal, Hilly; 30km and 100km; GREEDY and AUTO) and **4 fastbike correctness cells** under varying evaluation budgets {4, 8, 16, 32} and search strategies (`best_of_n` and `local`).

## 2. Strategy and Budget Performance Matrix

| Strategy | Budget (evals) | All Cells Win Rate | Eligible Win Rate | Median Rel Imp (%) | Mean Added Latency (ms) | p90 Added Latency (ms) | Cache Hit Rate (%) |
|---|---|---|---|---|---|---|---|
| `best_of_n` | 4 | 0.0% (0/32) | 0.0% (0/31) | **0.00%** | 91 ms | 158 ms | 0.0% |
| `best_of_n` | 8 | 0.0% (0/32) | 0.0% (0/31) | **0.00%** | 397 ms | 460 ms | 0.0% |
| `best_of_n` | 16 | 0.0% (0/32) | 0.0% (0/31) | **0.00%** | 630 ms | 3001 ms | 0.4% |
| `best_of_n` | 32 | 0.0% (0/32) | 0.0% (0/31) | **0.00%** | 674 ms | 3001 ms | 1.5% |
| `local` | 4 | 0.0% (0/32) | 0.0% (0/31) | **0.00%** | 90 ms | 151 ms | 0.0% |
| `local` | 8 | 0.0% (0/32) | 0.0% (0/31) | **0.00%** | 396 ms | 462 ms | 0.0% |
| `local` | 16 | 0.0% (0/32) | 0.0% (0/31) | **0.00%** | 621 ms | 3001 ms | 0.4% |
| `local` | 32 | 0.0% (0/32) | 0.0% (0/31) | **0.00%** | 668 ms | 3001 ms | 1.5% |

## 3. Fastbike Correctness Verification

Fastbike cases were tested to verify algorithm safety and non-interference on paved/fast-motorized profiles:

| Cell | Algorithm | Baseline Oracle Cost/m | Refined Cost/m | Applied | Verdict / Reason |
|---|---|---|---|---|---|
| basel_30km_fastbike | AUTO | 1.4037 | 1.4037 | false | length_error_worse: refErr=0.1901 > baseErr=0.1463 |
| basel_100km_fastbike | AUTO | 1.5185 | 1.5185 | false | no_feasible_evaluations |
| mallorca_30km_fastbike | AUTO | 1.9776 | 1.9776 | false | no_feasible_evaluations |
| mallorca_100km_fastbike | AUTO | 2.1278 | 2.1278 | false | no_feasible_evaluations |

## 4. Never-Worse Property Verification

- **Bar:** 100% of cells must satisfy $C_{\text{ref}} \le C_{\text{base}}$.
- **Result:** **PASSED (100% adherence)**. Across all 36 cells and all 8 budget/strategy permutations (288 evaluations total), no candidate was ever published with a higher oracle cost per meter than the baseline.
- **Refine-Off Parity:** `roundTripRefine=none` yields 100% bit-identical routes across all golden suites (`GreedyPlannerParityTest` 12 keys, `LoopGoldenSignatureTest` 9 scenarios).

## 5. Checkpoint Decision

- **Quality Statistic:** **0.00%** median per-cell relative improvement in oracle cost/m on `local_16`.
- **Verdict:** **EXPAND / REJECT**. Median improvement is under 1.0%.
