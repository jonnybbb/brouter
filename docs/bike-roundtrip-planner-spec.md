# Bike Roundtrip Route Planner — Technical Specification

**Version:** 1.0  
**Based on:** Jaszcz et al. (IVUS 2024) · Stroobant et al. (Ghent University, 2017)  
**Scope:** Algorithm design, data model, inputs/outputs, terrain adaptations, and implementation guidance for a roundtrip bicycle route planner that works across diverse terrain types (flat, hilly, rural, island, urban).

---

## 1. Problem Statement

Given a start location, a desired round-trip distance, and a set of cyclist preferences, generate a **closed route** (a cycle in the graph) that:

- Starts and ends at the same node
- Has a total length within ±5% of the desired distance (or within a specified `[lmin, lmax]` window)
- Minimises a **weighted combination** of edge unpleasantness and route non-roundness
- Prefers surfaces appropriate to the chosen bike type
- Avoids re-traversing the same roads or regions as much as possible

This problem is **NP-hard** (reducible from Planar Hamiltonian Cycle). A practical solution requires a fast heuristic, optionally validated against a branch-and-bound lower bound.

---

## 2. Data Model

### 2.1 Graph Representation

The road network is represented as a **directed embedded multigraph**:

```
G = (V, E, c̄, l)
```

| Symbol | Type | Description |
|--------|------|-------------|
| `V` | set | Vertices (OSM nodes / road intersections) |
| `E ⊆ V × V` | set | Directed arcs (road segments) |
| `c̄: V → ℝ²` | function | 2D coordinate of each vertex (lat/lon projected to UTM) |
| `l: E → ℝ⁺` | function | Physical length of each edge in metres |

**Constraint:** For every arc `e = (u, v)`, `l(e) ≥ dv(u, v)` where `dv` is great-circle distance.

### 2.2 Displacement Metric

Use the **haversine formula** (great-circle / Orthodrome distance) for all displacement calculations:

```
d(a, b) = 2r · arcsin(√(sin²(Δlat/2) + cos(lat_a)·cos(lat_b)·sin²(Δlon/2)))
```

Where `r ≈ 6371 km` and all angles are in radians.

The displacement between two edge centres is:

```
de(⟨u₁,v₁⟩, ⟨u₂,v₂⟩) = d((c̄(u₁)+c̄(v₁))/2, (c̄(u₂)+c̄(v₂))/2)
```

### 2.3 Edge Attributes (from OSM)

Each edge must carry:

| Attribute | OSM Tag | Notes |
|-----------|---------|-------|
| `length` | `way.length` | Metres, computed from node coordinates |
| `surface` | `surface=*` | See §3.2 for classification |
| `highway` | `highway=*` | Road type (used for safety weighting) |
| `maxspeed` | `maxspeed=*` | km/h, used for fastness scoring |
| `cycleway` | `cycleway=*` | Presence and separation of bike lane |
| `access` | `access=*` | Filter impassable edges |
| `oneway` | `oneway=*` | Determines arc direction |
| `incline` | `incline=*` | Gradient %; used for elevation weighting |
| `name` | `name=*` | Optional, for narrative generation |
| `scenic_tags` | `natural`, `landuse`, `leisure`, `amenity` | See §4.3 for scenery scoring |

### 2.4 Pre-processing

1. **Download OSM data** for the bounding box of `[start ± lmax]` using the Overpass API or a local PBF extract.
2. **Build directed graph**: For two-way roads, add both arc directions. For one-way roads, add only the permitted direction.
3. **Filter inaccessible edges**: Remove edges where `access = no` or the highway type is motorway/trunk (unless a dedicated cycleway exists).
4. **Compute edge weights** (see §4) and store as edge attributes.
5. **Compute reaches** (see §6.2) for the forward routing optimisation.

---

## 3. Bike Profile System

### 3.1 Profiles

Three base profiles; users may define custom profiles:

| Profile | Target Surface | Use Case |
|---------|---------------|----------|
| `road` | Smooth paved | Road bike, racing, fast touring |
| `mountain` | Gravel, dirt, off-road | MTB, bikepacking on trails |
| `touring` | Mixed, preferring paved | Trekking bike, loaded touring |

### 3.2 Surface Classification

Group OSM `surface=*` values into three categories per profile:

| Category | OSM Surface Values | Description |
|----------|--------------------|-------------|
| `road_paths` | `asphalt`, `concrete`, `concrete:plates`, `paving_stones`, `sett`, `cobblestone` | Hard, level surfaces |
| `off_road_paths` | `ground`, `dirt`, `earth`, `unpaved`, `gravel`, `fine_gravel`, `pebblestone`, `grass`, `grass_paver` | Uneven, natural surfaces |
| `neutral_paths` | `paved`, `compacted`, `wood`, `unhewn_cobblestone` | Sub-optimal for extremes |
| `unknown` | `null`, unrecognised | Treated as neutral |

### 3.3 Surface Multipliers

A multiplier `m ∈ (0, ∞)` modifies effective edge length for routing. Values `< 1` make surfaces cheaper (preferred); values `> 1` make them more expensive (avoided):

```yaml
profiles:
  road:
    road_paths: 0.25        # strongly prefer
    neutral_paths: 1.25
    off_road_paths: 2.00    # strongly avoid
    unknown: 1.00

  mountain:
    road_paths: 2.00        # strongly avoid
    neutral_paths: 1.00
    off_road_paths: 0.25    # strongly prefer
    unknown: 1.00

  touring:
    road_paths: 0.60
    neutral_paths: 0.90
    off_road_paths: 1.40
    unknown: 1.00
```

**Custom profiles** are defined in a YAML configuration file and loaded at startup. All multiplier values must be strictly positive.

---

## 4. Edge Cost Functions

### 4.1 Multi-Criteria Local Cost

Each edge `e` has a local cost `cl(e)` computed as a weighted combination of `k` criteria:

```
cl(e) = c̄a(e) · w̄    where  Σ wᵢ = 1,  wᵢ ≥ 0
```

The three built-in criteria are:

#### 4.2 Criterion 1 — Surface / Bike Type Fit

```
c_surface(e) = l(e) · m(surface(e), profile)
```

Where `m(·)` is the multiplier from §3.3. This is the primary routing driver for bike-type preference.

#### 4.3 Criterion 2 — Scenic Value

Reduce edge cost when the road passes through or near:
- `natural=*`: forest, water, coastline, heath → scale factor `0.60`
- `leisure=*`: park, nature_reserve → scale factor `0.70`
- `amenity=cafe`, `tourism=viewpoint` within 200 m → additive bonus `−0.10 · l(e)`
- Urban/industrial areas → scale factor `1.20`

Lookup is done via spatial index (R-tree) over OSM polygon features.

#### 4.4 Criterion 3 — Road Safety

Scale factor applied to edge length based on infrastructure:

| Condition | Factor |
|-----------|--------|
| Dedicated separated cycleway | 0.50 |
| On-road marked cycle lane | 0.75 |
| Low-speed residential street (≤ 30 km/h) | 0.85 |
| Arterial road, no lane, speed ≤ 50 km/h | 1.20 |
| Arterial road, no lane, speed > 50 km/h | 1.80 |
| Motorway / trunk (passable only on cycleway) | 2.50 |

#### 4.5 Criterion 4 — Fastness (Optional)

Penalise edges that traverse busy intersections and traffic lights:

```
c_fast(e) = l(e) + n_intersections(e) · 30   [seconds equivalent]
           + n_traffic_lights(e) · 60
```

#### 4.6 Weight Vector

Default weights `w̄` (user-adjustable via a triangle interface):

```
w = (w_surface, w_scenic, w_safety, w_fastness)
  = (0.40,      0.30,     0.25,     0.05)
```

The full local cost is:

```
cl(e) = w_surface · c_surface(e)
       + w_scenic  · c_scenic(e)
       + w_safety  · c_safety(e)
       + w_fastness· c_fast(e)
```

### 4.7 Weighting Function for Dijkstra

During routing the effective edge weight is:

```
weight(e) = cl(e)  · (1 + visited_penalty(e))
```

Where:
- `visited_penalty(e) = ∞` if the edge has been marked as visited and we are still in the forward-search phase (forces edge avoidance).
- `visited_penalty(e) = 0` on the return path (edges may be re-used as a fallback).

---

## 5. Roundness Metric

### 5.1 Motivation

A naïve optimisation for lowest-cost edges tends to generate routes that orbit a single low-cost cycle repeatedly. To prevent this, each tour is assigned a **penalty** that quantifies how much it revisits the same regions.

### 5.2 Expected Displacement

For an ideal circular tour of length `l`, the displacement between two points separated by travelled distance `x` follows a half-sine curve. This is approximated by a piecewise-linear function:

```
dexp(x) ≈ (l/π) · min(2x/l, 2(l−x)/l)   for 0 ≤ x ≤ l
```

The slope is independent of total tour length, making it usable before the tour length is known.

### 5.3 Edge Pair Penalty

For any pair of edges `(eᵢ, eⱼ)` with centres separated by great-circle displacement `de(eᵢ, eⱼ)` and minimal travel distance `dmin(eᵢ, eⱼ)`:

```
p(eᵢ, eⱼ) = max(0, (σ · dexp(dmin) − de) / (σ · dexp(dmin)))
```

Where `σ ∈ [0,1]` is a user-controlled **strictness factor**:
- `σ → 0`: only penalise reusing the exact same edge
- `σ → 1`: penalise visiting any nearby region

**Recommended default:** `σ = 0.4`

### 5.4 Average Tour Penalty

For a tour `π = (e₁, e₂, ..., eₙ)`:

```
pavg(π) = (Σᵢ Σⱼ p(eᵢ, eⱼ) · l(eᵢ) · l(eⱼ)) / l(π)²
```

This approximation becomes more accurate as tour length increases. For tours shorter than 20 km, use a higher-resolution graph (edges ≤ 10 m) to reduce discretisation error.

---

## 6. Route Generation Algorithm

### 6.1 Objective Function

The overall cost to minimise:

```
ct,avg(π) = cl,avg(π) + λ · pavg(π)
```

Under the constraint: `lmin ≤ l(π) ≤ lmax`

Where:
- `cl,avg(π) = cl(π) / l(π)` — average local cost per metre
- `λ > 0` — penalty weight (recommended default: `λ = 12`)
- `lmin = D · 0.95`, `lmax = D · 1.05` for a target distance `D`

### 6.2 Phase 0 — Graph Preprocessing

**Reach computation** (Gutman, 2004): For each node `v`, precompute `r(v)` — the maximum reach across all least-cost paths through `v`. Reaches are computed for representative weight combinations `w̄` and cached. During routing, the nearest precomputed `w̄` is used.

Nodes with `r(v) < lrem` are pruned from the forward search tree, where:

```
lrem(πs→v) = (lmin − l(πs→v) − dv(s,v)) / 2
```

This yields sparse middle sections and dense start/end sections of the search tree, reducing runtime by ~4× for long tours.

### 6.3 Phase 1 — Forward Routing

1. Run **Dijkstra** from start node `s` using `cl(e)` as edge weight.
2. Build forward shortest-path tree `f` containing all nodes `v` where:
   ```
   l(πf(v)) + dv(s, v) ≤ lmax
   ```
3. Collect candidate turning points `C` = all nodes `v` where:
   ```
   l(πf(v)) + dv(s, v) ≥ lmin
   ```
   Also include nodes `v` with a neighbour outside the tree (graph boundary nodes).
4. Select turning point `t` from `C`. For multiple route generations, adjust selection probabilities to diversify directions; decrease probability of nodes near previously chosen turning points.

### 6.4 Phase 2 — Backward Routing

For each node `v` on the forward path `πf(t)`:

1. Compute adapted edge cost `c'(e)` that penalises proximity to `πf(t)`:
   ```
   c'(e) = cl(e) + (2λ / lmax) · Σ_{e' ∈ πf(t)} p(e, e') · l(e) · l(e')
   ```
2. Run **Dijkstra** backwards from each `v` to `s` using `c'(e)`.
3. Evaluate tour `π = πf(v) + πr(v)` and record score `ct,avg(π)`.
4. **Pruning:** Skip node `w` if:
   ```
   l(πf(c)) + dv(c, w) + l(πr(w)) ≥ lmax
   ```
   Where `c` is the current frontier node of the forward path.
5. Return the best-scoring tour that satisfies `lmin ≤ l(π) ≤ lmax`.

**Recommended:** Run the full algorithm 4 times and return the best result. Each run selects a different turning point to explore different directions.

### 6.5 Forward Path Approximation

To reduce backward routing cost from O(lmax³) to O(lmax² log lmax), approximate `πf(t)` with O(log lmax) aggregate edges. Replace consecutive edges `eᵢ...eⱼ` with a single synthetic edge `eᵢ→ⱼ` when:

```
(2 · dmax(eᵢ→ⱼ) + σ · dexp(l_{eᵢ→eⱼ})) / (σ · dexp(min(l_{s→eⱼ}, l_{eᵢ→t}))) < ε
```

Recommended: `ε = 0.01`

### 6.6 Sub-Route Greedy Algorithm (Simpler Alternative)

For environments where the Ghent heuristic is impractical (low memory, constrained compute), use the Silesian greedy algorithm:

**Parameters:**
- `n = 5` sub-routes (optimal for roundness; `n < 5` yields out-and-back, `n > 7` yields overlapping routes)
- `Er = 0.05` distance error tolerance
- `K = 10` maximum retries per sub-route

**Algorithm (per sub-route `i`):**

1. Set target sub-route distance `part = D / n`.
2. Run Dijkstra with cutoff `part` from current node `a`, collecting all reachable nodes.
3. Score each candidate node `b`:
   ```
   score(a, b, p) = (d(a,b) − Rₐ)² + (d(p,b) − Rₚ)²
   ```
   Where `Rₐ = Rₚ = part` (estimated radial distances), `p` is the previous sub-route end.
4. Select `b` with minimum score (or randomly from top-`x` for route diversity).
5. Check if `total_distance + part_distance + back_distance` is within `D · (1 ± Er)`:
   - Within bounds → append sub-route, continue to next.
   - Too long → halve `part`, retry (up to `K` times).
   - Just right → close the route back to start via Dijkstra.
6. Mark all traversed edges as `visited = True` to discourage re-use in subsequent sub-routes.

---

## 7. Terrain Adaptations

### 7.1 Flat Terrain (Netherlands, Belgian coast, plains)

- **Challenge:** No natural landmarks → routes tend to be straight out-and-back.
- **Adaptation:** Increase `λ` (penalty weight) to `λ = 20` to force roundness more aggressively. Increase `n = 6` sub-routes.
- **Surface priority:** Road and cycle path infrastructure is typically excellent; prefer `cycleway=*` edges. Disable or minimise elevation weighting.
- **Known issue:** Sparse canal networks may create disconnected graph components. Implement bridge/ferry edge support with `route=ferry` OSM tags.

### 7.2 Hilly / Mountainous Terrain (Alps, Pyrenees, Swiss Pre-Alps)

- **Challenge:** Elevation significantly affects effort and enjoyment. Route length in km does not reflect rider effort.
- **Adaptation:**
  - Add elevation as an additional criterion. Fetch elevation from SRTM or Copernicus DEM (30 m resolution).
  - Compute **climbing-adjusted distance** (VAM normalisation):
    ```
    l_adjusted(e) = l(e) · (1 + k_climb · max(0, Δh(e)/l(e)))
                           · (1 + k_descent · max(0, −Δh(e)/l(e)))
    ```
    Recommended defaults: `k_climb = 8.0`, `k_descent = 0.5` (climbs cost more than descents).
  - Allow the user to specify a **maximum elevation gain** constraint (e.g. ≤ 1500 m for a 60 km route).
  - Use adjusted distance as the length function `l(e)` throughout the algorithm.
- **Surface priority:** Mountain profile prefers off-road; touring profile prefers switchback roads.
- **Known issue:** Ridgeline paths often form dead-ends (accessible from one side only). Increase `K` retries.

### 7.3 Rural / Sparse Networks (countryside, forest regions)

- **Challenge:** Low road density → algorithm may loop infinitely or return short, poor-quality routes.
- **Adaptation:**
  - Lower `lmin` tolerance: allow `lmin = D · 0.80` instead of `D · 0.95` for sparse graphs.
  - Include `track`, `path`, and `bridleway` OSM highway types (filtered out by default in urban settings).
  - Assign moderate multipliers to unpaved tracks: `off_road = 0.80` for touring profile in rural mode.
  - Implement a **connectivity check** before routing: verify `s` belongs to the largest connected component. If not, snap to nearest connected node.
- **Known issue:** NaN surface labels are more frequent. Apply a fallback classification based on `highway` type:
  - `highway=track` → `off_road`
  - `highway=path` → `off_road`
  - `highway=residential` / `unclassified` → `neutral`
  - `highway=primary/secondary/tertiary` → `road`

### 7.4 Island / Coastal Terrain

- **Challenge:** Road network forms a ring with few interior connections. The algorithm may always find the same route (clockwise or anti-clockwise circumnavigation).
- **Adaptation:**
  - Detect island topology: if the graph has a single large cycle and few interior branches, enable **interior path bonus** — reduce `cl` for any edge that is not part of the outer ring.
  - Split the island graph into sectors (e.g. 4 quadrants) and force sub-routes to visit distinct sectors.
  - If only one valid roundtrip exists (e.g. very small island), offer the user a partial loop with ferry or out-and-back segments clearly flagged.
  - Support `route=ferry` edge types with a configurable penalty (default: `c_ferry = 3.0 · l(e)`, user can disable ferry use entirely).
- **Known issue:** Coastal paths often have gaps (private property, cliffs). Build fallback routing onto inland alternatives.

### 7.5 Urban Terrain

- **Challenge:** High road density → algorithm finds many valid routes but round-trip "loop quality" degrades because streets are short and heavily interconnected.
- **Adaptation:**
  - Increase penalty weight `λ = 15`, strictness `σ = 0.5` to enforce avoiding nearby streets.
  - Add heavy bonus for dedicated cycle infrastructure (`cycleway=track` or `cycleway=lane`).
  - Penalise busy roads (`highway=primary/secondary`) more aggressively in safety criterion.
  - For very short urban loops (< 10 km), use higher-resolution graph representation (see §5.4).

---

## 8. API & Input/Output Specification

### 8.1 Route Request

```typescript
interface RouteRequest {
  start: { lat: number; lon: number };   // WGS84
  targetDistance: number;                 // metres
  distanceTolerance?: number;             // default 0.05 (±5%)
  bikeProfile: 'road' | 'mountain' | 'touring' | CustomProfile;
  preferences: {
    scenicWeight:  number;  // 0–1, default 0.30
    safetyWeight:  number;  // 0–1, default 0.25
    surfaceWeight: number;  // 0–1, default 0.40
    fastWeight:    number;  // 0–1, default 0.05
  };
  roundness: {
    strictness: number;      // σ ∈ [0,1], default 0.4
    penaltyWeight: number;   // λ > 0, default 12
  };
  terrain?: 'flat' | 'hilly' | 'rural' | 'island' | 'urban' | 'auto';
  maxElevationGain?: number;             // metres, optional
  avoidFerries?: boolean;                // default false
  numCandidates?: number;                // routing attempts, default 4
}
```

### 8.2 Route Response

```typescript
interface RouteResponse {
  geometry: GeoJSON.LineString;          // complete route geometry
  totalDistance: number;                  // metres
  totalElevationGain: number;             // metres
  totalElevationLoss: number;             // metres
  estimatedDuration: number;              // seconds (flat-speed adjusted)
  score: {
    averageLocalCost: number;
    averagePenalty: number;
    totalScore: number;
  };
  surfaceBreakdown: {
    road: number;      // % of route
    offRoad: number;
    neutral: number;
    unknown: number;
  };
  waypoints: Array<{
    lat: number;
    lon: number;
    distanceFromStart: number;
    elevation?: number;
  }>;
  distanceError: number;   // |actualDistance − targetDistance| / targetDistance
  warnings: string[];      // e.g. "Sparse network: tolerance relaxed to 15%"
}
```

### 8.3 GPX Export

The route geometry must be exportable as a GPX 1.1 file:
- `<trk>` with `<trkseg>` and `<trkpt lat= lon=>` for all waypoints
- Elevation (`<ele>`) included when DEM data is available
- Route name includes: profile, distance, date

---

## 9. Performance Requirements

| Tour Length | Target Latency (4 attempts) |
|-------------|----------------------------|
| < 20 km     | < 500 ms                   |
| 20–80 km    | < 2 s                      |
| 80–150 km   | < 5 s                      |

Based on Stroobant et al. benchmarks on Intel Core i7, graph loaded in RAM.

**Graph loading:** The OSM subgraph for the routing bounding box should be pre-extracted and cached. Avoid re-downloading on each request.

**Algorithmic complexity:** `O(lmax² · log(lmax))` per attempt, where `lmax` is in km. With reach optimisation, practical speedup is ~4×.

---

## 10. Quality Metrics

| Metric | Target |
|--------|--------|
| Distance MAPE (short routes 2–6 km) | < 5% (achieved: 3.24%) |
| Distance MAPE (long routes 20–60 km) | < 5% (achieved: 2.74%) |
| Tour-finding success rate (4 attempts) | > 95% for routes with `lmax − lmin ≥ 5 km` |
| Heuristic score vs. optimal gap | < 10% above lower bound (typical) |

---

## 11. Known Limitations & Mitigations

| Limitation | Mitigation |
|------------|------------|
| Mislabelled or missing OSM `surface=*` | Fallback classification by `highway=*` type (§7.3) |
| Disconnected graph components (islands, cul-de-sacs) | Snap start to largest component; relax tolerance |
| Very sparse networks (rural, mountain ridgelines) | Increase `K` retries; widen `[lmin, lmax]` window |
| Short tours (< 5 km) have high roundness approximation error | Use high-resolution graph with edge length ≤ 10 m |
| Penalty formula assumes uniform edge length distribution | Error decreases naturally for tours > 20 km |
| Terrain-type auto-detection not 100% reliable | Allow user to manually override terrain mode |
| Algorithm may produce identical routes for repeated requests | Use top-`x` random node selection during sub-route finding |

---

## 12. Configuration Reference

All tunable parameters, their meaning, recommended defaults, and valid ranges:

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `n` (sub-routes) | 5 | 3–8 | Roundness of generated loop |
| `Er` (distance tolerance) | 0.05 | 0.02–0.15 | Allowed distance error from target |
| `K` (max retries) | 10 | 5–20 | Robustness in sparse graphs |
| `σ` (strictness) | 0.40 | 0–1 | Aggressiveness of roundness penalty |
| `λ` (penalty weight) | 12 | 0–50 | Trade-off between cost and roundness |
| `β` (turn-point min offset) | 0.60 | 0–1 | Fraction of `lmin` before valid turning point |
| `ε` (path approximation) | 0.01 | 0.001–0.05 | Accuracy vs. speed of forward path approx |
| `k_climb` | 8.0 | 4–15 | Effort multiplier per metre of climbing |
| `k_descent` | 0.5 | 0–2 | Effort reduction per metre of descent |
| `x` (top candidates) | 5 | 1–20 | Randomness in sub-route node selection |

---

## 13. Implementation Notes for BRouter Integration

Javik uses **BRouter** as its routing backbone. The following mappings apply:

- BRouter profiles already encode surface multipliers similar to §3.3. Replace or supplement with the profile YAML in §3.3 for custom behaviour.
- BRouter handles OSM graph ingestion and Dijkstra; the roundtrip logic (phases 1–2 of §6) wraps around BRouter's point-to-point routing calls.
- BRouter's `nogo` areas can be used to implement the visited-edge penalty (§4.7): after each sub-route, register traversed edges as nogo polygons.
- For reach-based optimisation, BRouter does not expose internal reach values; implement the reach computation as a pre-processing step on the extracted OSM graph before handing off to BRouter.
- BRouter's elevation profiles (from SRTM) can be used directly for the elevation-adjusted distance in §7.2.

---

## 14. References

1. Jaszcz A., Hankus S., Bober M., Bugla B. — *Efficient Dijkstra-based greedy algorithm for cycle-route planning*. IVUS 2024, CEUR-WS Vol. 3885, paper 32.
2. Stroobant P., Audenaert P., Colle D., Pickavet M. — *Generating Constrained Length Personalized Bicycle Tours*. Ghent University – imec, IDLab, 2017.
3. OpenStreetMap contributors — https://www.openstreetmap.org
4. Gutman R. — *Reach-based Routing: A New Approach to Shortest Path Algorithms Optimized for Road Networks*. ALENEX 2004.
