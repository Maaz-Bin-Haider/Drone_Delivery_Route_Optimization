# Technical Design Document

## Drone Delivery Route Optimization System

**Course:** Design & Analysis of Algorithms (DAA)
**Document version:** 1.1
**Date:** 22 September 2026
**Companion document:** [Software Requirements Specification](SRS.md)

---

### Document Control

| Field | Value |
|---|---|
| Group number | **02** |
| Institution | `<UNIVERSITY / DEPARTMENT>` |
| Course instructor | `<PROFESSOR NAME>` |
| Submission date | `<DEADLINE>` |

### Group Members

| # | Name | Seat number |
|---|---|---|
| 1 | `<NAME>` | `<SEAT NO.>` |
| 2 | `<NAME>` | `<SEAT NO.>` |
| 3 | `<NAME>` | `<SEAT NO.>` |
| 4 | `<NAME>` | `<SEAT NO.>` |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Design Goals and Principles](#2-design-goals-and-principles)
3. [System Architecture](#3-system-architecture)
4. [Technology Stack](#4-technology-stack)
5. [Domain Data Model](#5-domain-data-model)
6. [The Cost Model](#6-the-cost-model)
7. [Routing Algorithms](#7-routing-algorithms)
8. [Energy Feasibility and Charging Reroute](#8-energy-feasibility-and-charging-reroute)
9. [Prioritization and Fleet Assignment](#9-prioritization-and-fleet-assignment)
10. [Environment Model](#10-environment-model)
11. [Planning Orchestrator and Caching](#11-planning-orchestrator-and-caching)
12. [REST API Specification](#12-rest-api-specification)
13. [Frontend Design](#13-frontend-design)
14. [Project Structure](#14-project-structure)
15. [Complexity Analysis Summary](#15-complexity-analysis-summary)
16. [Experimental Plan](#16-experimental-plan)
17. [Testing Strategy](#17-testing-strategy)
18. [Risks and Mitigations](#18-risks-and-mitigations)
19. [Work Distribution](#19-work-distribution)
20. [Requirements Traceability](#20-requirements-traceability)
21. [Future Work](#21-future-work)

---

## 1. Introduction

### 1.1 Purpose

This Technical Design Document (TDD) specifies **how** the Drone Delivery Route Optimization
System (DDROS) satisfies the requirements stated in the companion SRS. It defines the
architecture, the data model, the mathematical cost model, the algorithms with their
pseudocode and complexity, the module decomposition, the API contract and the verification
strategy.

Where the SRS says *what* must happen, this document says *by what mechanism*, and — for a
Design & Analysis of Algorithms deliverable — *at what cost*.

### 1.2 Scope

This document covers the complete system: algorithm tier, application tier and presentation
tier. It is the implementation reference for the development team and the design record for
assessment.

### 1.3 Relationship to the SRS

Every design element in this document is traceable to one or more requirement identifiers of
the form FR-n.m or NFR-n. Section 20 provides the reverse mapping from requirement to module.

### 1.4 Intended Audience

The development team, the course instructor, and any future maintainer. Readers are assumed
familiar with graph algorithms at the level of *Introduction to Algorithms* (Cormen et al.),
and with basic web application structure.

---

## 2. Design Goals and Principles

| # | Principle | Consequence for the design |
|---|---|---|
| **P1** | **Algorithms are the product.** | The algorithm tier is written from first principles, is free of framework dependencies, and is independently testable. No third-party shortest-path routine is used. *(NFR-6, NFR-7)* |
| **P2** | **Every decision is explainable.** | Each algorithm returns not merely a result but the evidence behind it: the route, its cost breakdown, the alternatives rejected, and the search statistics. *(FR-10.7)* |
| **P3** | **Measurement is built in, not bolted on.** | Instrumentation lives inside the search routines, so every run in normal operation yields the same statistics the benchmark harness collects. *(FR-11.1)* |
| **P4** | **Correctness is continuously cross-checked.** | Dijkstra and A* must agree on total cost for identical inputs. This equivalence is asserted in the test suite and re-verified on every benchmark run, turning heuristic admissibility into a testable property. *(FR-3.4, FR-11.6)* |
| **P5** | **Determinism.** | All tie-breaking is explicit and total. Identical inputs always yield byte-identical output. *(NFR-11)* |
| **P6** | **Fail loudly, never silently.** | Unreachable destinations, infeasible energy budgets and malformed input produce explicit typed results, never exceptions escaping to the user or silently empty routes. *(NFR-10, UI-11)* |
| **P7** | **Separation of cost from search.** | The search algorithms know nothing about wind, no-fly zones or weights; they consume an abstract edge-cost function. Environmental complexity therefore never leaks into the algorithm code. |

Principle **P7** is the single most important structural decision in this design. It is what
allows wind modelling and no-fly zones to be added as extensions without modifying Dijkstra
or A* at all.

---

## 3. System Architecture

### 3.1 Layered View

```
╔══════════════════════════════════════════════════════════════════════╗
║  PRESENTATION TIER  (browser — HTML/CSS/JavaScript)                  ║
║                                                                      ║
║   map.js        controls.js     results.js    charts.js   animation.js ║
║   Leaflet map   weight sliders  plan table    benchmark   drone       ║
║   + routes      wind, NFZ       + totals      plots       playback    ║
╚═══════════════════════════════════╤══════════════════════════════════╝
                                    │  HTTP / JSON  (localhost:5000)
╔═══════════════════════════════════╧══════════════════════════════════╗
║  APPLICATION TIER  (Flask)                                           ║
║                                                                      ║
║   api.py — endpoint routing, payload validation, serialisation       ║
║   orchestrator.py — the planning cycle                               ║
║   cache.py — all-pairs route cache with configuration-keyed          ║
║              invalidation                                            ║
╚═══════════════════════════════════╤══════════════════════════════════╝
                                    │  in-process function calls
╔═══════════════════════════════════╧══════════════════════════════════╗
║  ALGORITHM TIER  (pure Python — no framework imports)                ║
║                                                                      ║
║   ┌───────────┐ ┌────────────┐ ┌─────────────┐ ┌──────────────────┐  ║
║   │  graph/   │ │ algorithms/│ │ scheduling/ │ │   environment/   │  ║
║   │  Graph    │ │  dijkstra  │ │ priority_   │ │   wind           │  ║
║   │  adjacency│ │  astar     │ │  queue      │ │   no_fly         │  ║
║   │  validate │ │  heuristics│ │ assignment  │ │                  │  ║
║   │           │ │  constrained│ │             │ │                  │  ║
║   └───────────┘ └────────────┘ └─────────────┘ └──────────────────┘  ║
║   ┌───────────┐ ┌────────────┐ ┌─────────────┐                       ║
║   │structures/│ │   cost/    │ │  analysis/  │                       ║
║   │ min_heap  │ │ cost_model │ │ benchmark   │                       ║
║   └───────────┘ └────────────┘ └─────────────┘                       ║
╚══════════════════════════════════════════════════════════════════════╝
                                    │
╔═══════════════════════════════════╧══════════════════════════════════╗
║  DATA  —  data/*.json : city_map · fleet · deliveries · no_fly_zones ║
╚══════════════════════════════════════════════════════════════════════╝
```

### 3.2 Dependency Rule

Dependencies point **downward only**. The algorithm tier imports nothing from the
application or presentation tiers. This is enforced by a test that inspects the import graph
of `ddros/algorithms`, `ddros/graph`, `ddros/cost`, `ddros/scheduling` and `ddros/structures`
and fails if any of them references `flask` or `ddros.web`. *(NFR-7)*

### 3.3 The Planning Cycle

A single planning cycle — triggered at start-up and on every configuration change — proceeds
as follows:

```
  ┌──────────────────────┐
  │ 1. Load / read       │  city map, fleet, deliveries, NFZ definitions
  │    scenario          │
  └──────────┬───────────┘
             ▼
  ┌──────────────────────┐
  │ 2. Build environment │  wind vector + active NFZ set
  │                      │  → edge mask, edge energy/time multipliers
  └──────────┬───────────┘
             ▼
  ┌──────────────────────┐
  │ 3. Build cost model  │  α, β, γ + normalisation constants
  │                      │  → c(e) for every unmasked edge
  └──────────┬───────────┘
             ▼
  ┌──────────────────────┐
  │ 4. Precompute        │  V × Dijkstra → all-pairs cost, distance,
  │    all-pairs routes  │  energy, time, path   (cached)
  └──────────┬───────────┘
             ▼
  ┌──────────────────────┐
  │ 5. Build priority    │  min-heap keyed (priority, sequence)
  │    queue             │
  └──────────┬───────────┘
             ▼
  ┌──────────────────────┐   ◄──────────────────────────────┐
  │ 6. Pop next delivery │                                  │
  └──────────┬───────────┘                                  │
             ▼                                              │
  ┌──────────────────────┐                                  │
  │ 7. For each drone:   │  direct route feasible?          │
  │    test eligibility  │  else charging reroute?          │
  │    + projected       │  else ineligible                 │
  │    completion time   │                                  │
  └──────────┬───────────┘                                  │
             ▼                                              │
  ┌──────────────────────┐                                  │
  │ 8. Assign to the     │  update position, SoC, ready time │
  │    earliest finisher │                                  │
  └──────────┬───────────┘                                  │
             ▼                                              │
        queue empty? ──── no ───────────────────────────────┘
             │ yes
             ▼
  ┌──────────────────────┐
  │ 9. Emit plan         │  per-delivery detail, per-drone timeline,
  │                      │  batch totals, search statistics
  └──────────────────────┘
```

Steps 2–4 are the expensive part and are cached; steps 5–9 are cheap table lookups. Section
11 explains the cache key and its invalidation.

---

## 4. Technology Stack

| Layer | Choice | Rationale |
|---|---|---|
| Language | **Python 3.11+** | Readable algorithm code that a grader can follow line by line; rich standard library; `dataclasses` and `typing` give a clean domain model without boilerplate. |
| Web framework | **Flask** | Minimal surface area. The project needs about eight JSON endpoints and one static page; Flask adds almost no conceptual overhead, keeping attention on the algorithms. FastAPI was considered but its async model and Pydantic layer are unnecessary weight for a single-user local simulation. |
| Map rendering | **Leaflet.js** | Mature, dependency-free, renders geographic coordinates natively, and supports the polylines, circle markers, polygons and animated markers this design needs. |
| Charting | **Chart.js** | Small, declarative, sufficient for the benchmark line and bar charts. Rendered client-side so no server-side plotting dependency is required. |
| Geometry | **Hand-written** (`ddros/geo.py`) | Haversine distance, initial bearing, point-to-segment distance and point-in-polygon are each a few lines and are needed inside the admissibility argument, so they are implemented and tested directly rather than imported. |
| Testing | **pytest** | Concise assertions, parameterised cases, coverage integration. |
| Numerics | **Python stdlib only** (`math`, `statistics`) | NumPy is deliberately avoided in the algorithm tier so that every operation a grader inspects is explicit. |

> **Note on `heapq`.** The Python standard library provides a min-heap. Because constraint
> C-1 requires the priority queue to be implemented from first principles, `ddros/structures/min_heap.py`
> contains a hand-written binary heap with `sift_up` / `sift_down`, and the test suite
> cross-validates its output against `heapq` on randomised inputs. The hand-written heap is
> what the system uses; `heapq` appears only inside tests, as an oracle.

---

## 5. Domain Data Model

### 5.1 Core Entities

```python
class NodeType(Enum):
    WAREHOUSE        = "warehouse"
    CUSTOMER         = "customer"
    CHARGING_STATION = "charging_station"
    WAYPOINT         = "waypoint"

class Priority(IntEnum):          # lower value == higher precedence
    URGENT = 0
    HIGH   = 1
    NORMAL = 2

class DeliveryStatus(Enum):
    PENDING = "pending"; ASSIGNED = "assigned"
    DELIVERED = "delivered"; UNSERVICEABLE = "unserviceable"

@dataclass(frozen=True)
class Node:
    id:   str
    name: str
    lat:  float
    lon:  float
    type: NodeType

@dataclass(frozen=True)
class Edge:
    u: str                  # endpoint node id
    v: str                  # endpoint node id
    distance_km:   float    # > 0
    base_energy_pct: float  # >= 0, percentage of full charge, still-air
    bearing_uv_deg: float   # initial bearing u -> v, degrees from true north

@dataclass
class Drone:
    id:          str
    current_node: str
    battery_pct:  float     # 0..100, state of charge
    ready_at_min: float = 0.0
    assigned:     list[str] = field(default_factory=list)

@dataclass
class DeliveryRequest:
    id:          str
    destination: str
    priority:    Priority
    sequence:    int                       # tie-breaker, FR-2.5
    status:      DeliveryStatus = DeliveryStatus.PENDING

@dataclass(frozen=True)
class NoFlyZone:
    id:      str
    shape:   Literal["circle", "polygon"]
    active:  bool
    centre:  tuple[float, float] | None = None   # circle
    radius_km: float | None            = None    # circle
    vertices: list[tuple[float, float]] | None = None  # polygon
```

### 5.2 Result Types

```python
@dataclass(frozen=True)
class SearchStats:
    nodes_expanded:  int
    heap_operations: int
    runtime_ms:      float

@dataclass(frozen=True)
class Route:
    path:        list[str]     # ordered node ids, source .. target
    distance_km: float
    energy_pct:  float
    time_min:    float
    cost:        float         # composite, dimensionless
    algorithm:   Literal["dijkstra", "astar"]
    stats:       SearchStats

@dataclass(frozen=True)
class ChargingReroute:
    station_id:        str
    leg_one:           Route
    leg_two:           Route
    arrival_battery_pct: float
    recharge_min:      float

@dataclass(frozen=True)
class Assignment:
    delivery_id: str
    drone_id:    str
    route:       Route
    reroute:     ChargingReroute | None
    depart_min:  float
    arrive_min:  float
    battery_before_pct: float
    battery_after_pct:  float
    reason:      str           # human-readable justification, FR-10.7
```

`Route` being frozen and self-describing is what makes principle **P2** practical: any result
can be rendered without consulting the algorithm that produced it.

### 5.3 Graph Structure

The graph is stored as an adjacency list — a dictionary from node id to a list of incident
edge records — giving O(V + E) space *(FR-1.5)*.

```python
class Graph:
    nodes: dict[str, Node]
    adj:   dict[str, list[Edge]]        # adjacency list, FR-1.5

    def neighbours(self, node_id: str) -> Iterable[Edge]: ...
    def reversed(self) -> "Graph": ...  # used by the reverse Dijkstra of §8.3
```

Because the graph is undirected, each edge appears in the adjacency list of both endpoints,
with `bearing_uv_deg` reversed (bearing + 180° mod 360) in the second entry. This matters:
the wind model of §6.1 is direction-dependent, so the *same* corridor genuinely has different
costs in the two directions.

### 5.4 Scenario File Schemas

`data/city_map.json`:

```json
{
  "name": "Northport",
  "nodes": [
    {"id": "W",  "name": "Central Warehouse", "lat": 24.8607, "lon": 67.0011, "type": "warehouse"},
    {"id": "C1", "name": "Marine Drive",      "lat": 24.8712, "lon": 67.0284, "type": "customer"},
    {"id": "S1", "name": "Charging Pad North","lat": 24.8801, "lon": 67.0102, "type": "charging_station"}
  ],
  "edges": [
    {"u": "W", "v": "C1", "distance_km": 3.2, "base_energy_pct": 6.4}
  ]
}
```

`distance_km` may be omitted, in which case it is computed by haversine from the endpoint
coordinates *(FR-1.9)*. `base_energy_pct` may likewise be omitted and defaults to
`distance_km × ENERGY_RATE_PCT_PER_KM`. `bearing_uv_deg` is always derived, never authored.

`data/fleet.json`, `data/deliveries.json` and `data/no_fly_zones.json` follow the field
structure of the corresponding dataclasses in §5.1.

### 5.5 Validation

`ddros/graph/validation.py` enforces, on load *(FR-1.7)*:

| Check | Failure mode |
|---|---|
| Every edge endpoint exists in `nodes` | `MapValidationError` naming the edge |
| No self-loops (`u != v`) | `MapValidationError` naming the node |
| `distance_km > 0` and `base_energy_pct >= 0` | `MapValidationError` naming the edge *(FR-1.4)* |
| Node ids unique | `MapValidationError` naming the duplicate |
| Graph connected (single component, verified by BFS in O(V + E)) | `MapValidationError` listing unreachable nodes |
| Exactly one WAREHOUSE node | `MapValidationError` |
| At least one CHARGING_STATION | `MapValidationError` |
| Every delivery destination exists | `ScenarioValidationError` naming the delivery *(FR-2.3)* |

---

## 6. The Cost Model

This section defines the mathematics underpinning every routing decision. It implements
FR-4.1 to FR-4.6 and FR-9.1 to FR-9.4, and it is the foundation of the admissibility argument
in §7.3.

### 6.1 Wind Model

Let the wind be a uniform vector of speed `w` (m/s) blowing **toward** bearing `φ_w`
(degrees from true north). For an edge `e` with initial bearing `θ_e`, the **along-track
component** is

```
    w∥(e)  =  w · cos(θ_e − φ_w)                                    … (1)
```

`w∥ > 0` is a tailwind, `w∥ < 0` a headwind. The drone's **ground speed** along that edge is

```
    v_g(e)  =  max(v_air + w∥(e),  v_min)                            … (2)
```

where `v_air` is the constant nominal airspeed *(assumption A-2)* and `v_min > 0` is a floor
that prevents division by zero and keeps time finite in extreme headwind.

**Flight time** follows directly:

```
    t(e)  =  d_e / v_g(e)                                            … (3)
```

**Energy.** Modelling the drone as drawing roughly constant power in cruise, energy consumed
is proportional to time aloft. Since the still-air baseline `g_base(e)` corresponds to
`v_g = v_air`, the wind-adjusted energy is

```
    μ(e)  =  clamp( v_air / v_g(e),  μ_min,  μ_max )                 … (4)
    g(e)  =  g_base(e) · μ(e)                                        … (5)
```

A tailwind raises `v_g`, so `μ < 1` and the edge costs less energy; a headwind does the
reverse. The clamp — defaults `μ_min = 0.70`, `μ_max = 1.60` — bounds the model within the
range where the constant-power approximation is defensible, and guarantees `g(e) > 0`, which
§6.4 requires *(FR-9.4)*.

One vector therefore drives both time and energy consistently, rather than two unrelated
fudge factors.

### 6.2 Normalisation

Distance is in kilometres, energy in percent and time in minutes. Adding them directly would
be dimensionally meaningless and would make the weights uninterpretable, so each term is
divided by a constant reference *(FR-4.2)*:

```
    D_ref  =  max over e ∈ E of  d_e
    G_ref  =  max over e ∈ E of  g_base(e) · μ_max
    T_ref  =  max over e ∈ E of  d_e / v_min
```

These are **worst-case** references computed once from the graph. Critically they do **not**
depend on the current wind or weights, so they are constant throughout any search — a
property the consistency proof of §7.3 relies on. Each normalised term lies in `[0, 1]`.

### 6.3 Composite Edge Cost

```
    c(e)  =  α · d_e/D_ref  +  β · g(e)/G_ref  +  γ · t(e)/T_ref     … (6)

    subject to   α + β + γ = 1,   α, β, γ ∈ [0, 1]                   … (FR-4.3)
```

The route cost is the sum of its edge costs. Setting `(α, β, γ) = (1, 0, 0)` recovers pure
shortest-distance routing; `(0, 1, 0)` gives pure minimum-energy routing. The dashboard
sliders move continuously between these, which is precisely the trade-off the project brief
raises in §2.4 — *"the shortest route may not always be the best route"* — made directly
manipulable.

### 6.4 Non-Negativity

**Claim.** `c(e) ≥ 0` for every edge and every admissible weight setting.

**Proof.** `d_e > 0` by validation, and `D_ref > 0`. By (4), `μ(e) ≥ μ_min > 0`, and
`g_base(e) ≥ 0`, so `g(e) ≥ 0` and `G_ref > 0`. By (2), `v_g(e) ≥ v_min > 0`, so by (3)
`t(e) > 0`, and `T_ref > 0`. Each of the three normalised terms is therefore non-negative,
and `α, β, γ ≥ 0`, so their weighted sum is non-negative. ∎

This satisfies FR-4.4 and establishes the precondition for Dijkstra's correctness *(C-2)* —
without it, the entire routing layer would be unsound.

### 6.5 Interface

```python
class CostModel:
    """Evaluates edge cost under weights + environment. Constant-time per edge."""
    def __init__(self, graph, weights: Weights, wind: Wind, nfz: NoFlyMask): ...
    def edge_cost(self, e: Edge)   -> float:  ...   # equation (6)
    def edge_energy(self, e: Edge) -> float:  ...   # equation (5)
    def edge_time(self, e: Edge)   -> float:  ...   # equation (3)
    def is_open(self, e: Edge)     -> bool:   ...   # False if masked by an NFZ
    @property
    def lambda_min(self) -> float: ...              # Λ, see §7.3
```

Per principle **P7**, `CostModel` is the *only* object the search algorithms consult. They
have no knowledge of wind, zones or weights.

---

## 7. Routing Algorithms

### 7.1 Binary Min-Heap

The priority queue backing both searches is a hand-written binary min-heap over
`(key, insertion_counter, payload)` tuples. The insertion counter makes ordering total and
therefore deterministic *(P5)*.

| Operation | Complexity |
|---|---|
| `push` | O(log n) |
| `pop_min` | O(log n) |
| `peek` | O(1) |
| build from n items | O(n) via Floyd's heapify |

Stale entries are handled by **lazy deletion**: when a shorter path to an already-settled
node is found, a new entry is pushed and the obsolete one is discarded on pop. This avoids
implementing `decrease-key` and is the standard practical formulation.

### 7.2 Dijkstra's Algorithm  *(FR-3.1)*

```
ALGORITHM Dijkstra(G, source, target, costModel)
  INPUT   G          adjacency-list graph
          source     start node id
          target     goal node id
          costModel  edge cost oracle (§6.5)
  OUTPUT  Route, or UNREACHABLE

 1  for each v in G.nodes:  dist[v] ← ∞ ;  prev[v] ← NIL
 2  dist[source] ← 0
 3  settled ← ∅
 4  Q ← new MinHeap ;  Q.push(0, source)
 5  while Q not empty:
 6      (d, u) ← Q.pop_min()
 7      if u ∈ settled:  continue                    ▷ stale entry, lazy deletion
 8      settled ← settled ∪ {u}
 9      stats.nodes_expanded ← stats.nodes_expanded + 1
10      if u = target:  break                        ▷ early exit, FR-3.1
11      for each edge e = (u, v) in G.adj[u]:
12          if not costModel.is_open(e):  continue   ▷ no-fly mask, FR-8.2
13          if v ∈ settled:  continue
14          nd ← d + costModel.edge_cost(e)
15          if nd < dist[v]:
16              dist[v] ← nd ;  prev[v] ← (u, e)
17              Q.push(nd, v)
18  if dist[target] = ∞:  return UNREACHABLE         ▷ FR-3.7
19  return BuildRoute(prev, source, target, costModel)
```

`BuildRoute` walks `prev` backward from the target, reverses it, and accumulates distance,
energy and time over the edges — giving the full breakdown of FR-3.5 in O(|path|).

**Complexity.** Each node is settled once and each edge relaxed once, with a heap operation
of O(log V) per relaxation. Lazy deletion admits at most O(E) heap entries.

```
    Time   O((V + E) log V)          Space   O(V + E)
```

**Correctness.** Standard: by §6.4 all edge costs are non-negative, so when a node is popped
with minimum tentative distance, no shorter path to it can remain undiscovered.

### 7.3 A* Search  *(FR-3.2)*

A* is Dijkstra with the priority key changed from `g(v)` to `f(v) = g(v) + h(v)`. Lines
1–19 above are reused verbatim except:

```
 4′  Q.push(h(source), source)
17′  Q.push(nd + h(v), v)
```

#### 7.3.1 The Heuristic

Let `ĥ(n)` be the haversine great-circle distance in kilometres from node `n` to the target.
Define the per-kilometre minimum rates

```
    ġ_min  =  min over e ∈ E of ( g_base(e) / d_e ) · μ_min     (least possible energy per km)
    ṫ_min  =  1 / (v_air + w_max)                               (least possible time per km)
```

and set

```
    h(n)  =  ĥ(n) · [ α/D_ref  +  β·ġ_min/G_ref  +  γ·ṫ_min/T_ref ]
          =  ĥ(n) · Λ                                             … (7)
```

where `Λ` is a non-negative constant for a given configuration — computed once when the cost
model is built, making each heuristic evaluation a single multiplication.

#### 7.3.2 Admissibility  *(FR-3.3)*

**Claim.** `h(n) ≤ c*(n, target)`, the true minimum composite cost from `n` to the target.

**Proof.** Let `P = ⟨n = v₀, v₁, …, v_k = target⟩` be any path. By the triangle inequality for
great-circle distance, `Σᵢ d(vᵢ, vᵢ₊₁) ≥ ĥ(n)`. For each edge on `P`:

- its distance term contributes `α · d_e/D_ref ≥ Λ_α · d_e` where `Λ_α = α/D_ref`;
- its energy term is `β · g_base(e)·μ(e)/G_ref ≥ β · d_e · ġ_min/G_ref`, because
  `g_base(e) ≥ d_e · min(g_base/d)` and `μ(e) ≥ μ_min` by the clamp in (4);
- its time term is `γ · d_e/(v_g(e)·T_ref) ≥ γ · d_e · ṫ_min/T_ref`, because
  `v_g(e) ≤ v_air + w_max` by (2).

Summing, `c(P) ≥ Λ · Σᵢ d(vᵢ, vᵢ₊₁) ≥ Λ · ĥ(n) = h(n)`. Since this holds for every path, it
holds for the minimum, so `h(n) ≤ c*(n, target)`. ∎

#### 7.3.3 Consistency

**Claim.** `h(u) ≤ c(u, v) + h(v)` for every edge `(u, v)`.

**Proof.** By the triangle inequality, `ĥ(u) ≤ d(u,v) + ĥ(v)`, so
`h(u) = Λ·ĥ(u) ≤ Λ·d(u,v) + Λ·ĥ(v)`. From the per-edge bounds in §7.3.2,
`c(u,v) ≥ Λ·d(u,v)`. Substituting, `h(u) ≤ c(u,v) + h(v)`. ∎

Consistency implies each node is expanded at most once, so the `settled` check of line 7
remains valid and A* retains Dijkstra's asymptotic bound while expanding strictly fewer
nodes in practice.

**Complexity.** `O((V + E) log V)` worst case, identical to Dijkstra. The benefit is
constant-factor: fewer nodes expanded. FR-11.2 exists precisely to measure that factor rather
than assert it.

> **Design note — why the weights matter here.** When `β` or `γ` dominates, `Λ` is governed by
> the *minimum* energy or time rate across the whole graph, which is a weaker bound than the
> distance term gives. A* therefore degrades gracefully toward Dijkstra as the routing
> objective moves away from pure distance. This is a genuine, measurable prediction of the
> design, and §16 tests it.

### 7.4 Instrumentation  *(FR-11.1)*

Both searches populate a `SearchStats` record: `nodes_expanded` incremented at line 9,
`heap_operations` counted inside the heap, and `runtime_ms` measured with
`time.perf_counter()`. Because the counters live in the search itself, benchmark numbers and
live-run numbers are produced by identical code paths *(P3)*.

---

## 8. Energy Feasibility and Charging Reroute

Implements FR-4.6 and FR-5.1 to FR-5.7.

### 8.1 Feasibility Test

A route `R` is feasible for drone `k` iff

```
    Σ_{e ∈ R} g(e)   ≤   SoC_k  −  reserve                          … (8)
```

with `reserve` defaulting to 15 % *(FR-4.6)*.

### 8.2 Problem Statement and Honest Limitation  *(FR-5.7)*

Finding a minimum-cost path subject to an additive resource budget is the **Resource
Constrained Shortest Path Problem (RCSPP)**, which is NP-hard in general (reducible from
KNAPSACK). Two consequences follow, and the design states both plainly rather than concealing
them:

1. The minimum-**cost** path is not necessarily the minimum-**energy** path. A path rejected
   by (8) does not prove that no feasible path exists.
2. An exact solution requires multi-label search over the `(cost, energy)` Pareto frontier,
   which is exponential in the worst case although pseudo-polynomial in practice.

**Adopted approach.** The default is a two-phase heuristic (§8.3), which is fast, easy to
analyse, and adequate at the scale of the demonstration map. An exact **Pareto-label
variant** (§8.4) is provided as a selectable alternative, and §16 compares the two. Presenting
both — with the approximation's failure mode identified — is the defensible engineering
position.

### 8.3 Two-Phase Charging Reroute

The naive approach runs two searches per candidate station: `O(|C|)` pairs of Dijkstra runs.
The design instead exploits the structure of the problem to require only **two searches in
total**, irrespective of how many charging stations exist.

> **Key observation.** A single Dijkstra run from the drone's position yields the cost and
> energy to *every* node, including all charging stations. A single Dijkstra run on the
> **reversed** graph from the destination yields the cost and energy from *every* node to the
> destination. The best station is then an `argmin` over a precomputed table.

```
ALGORITHM ChargingReroute(G, s, z, drone, costModel)
  INPUT   s  drone position,  z  destination
  OUTPUT  ChargingReroute, or INFEASIBLE

 1  usable  ← drone.battery_pct − RESERVE
 2  full    ← 100 − RESERVE
 3
 4  Fwd ← DijkstraAllTargets(G,            s, costModel)   ▷ one run,  O((V+E) log V)
 5  Rev ← DijkstraAllTargets(G.reversed(), z, costModel)   ▷ one run,  O((V+E) log V)
 6
 7  best ← NIL ;  bestCost ← ∞
 8  for each station c where G.nodes[c].type = CHARGING_STATION:   ▷ O(|C|)
 9      if Fwd[c] = UNREACHABLE or Rev[c] = UNREACHABLE:  continue
10      if Fwd[c].energy > usable:  continue                ▷ leg 1 infeasible, FR-5.2
11      if Rev[c].energy > full:    continue                ▷ leg 2 infeasible, FR-5.2
12      arrival   ← drone.battery_pct − Fwd[c].energy
13      recharge  ← (100 − arrival) / CHARGE_RATE_PCT_PER_MIN        ▷ FR-5.4
14      total     ← Fwd[c].cost + Rev[c].cost + γ · recharge/T_ref   ▷ FR-5.3
15      if total < bestCost:  bestCost ← total ;  best ← c
16
17  if best = NIL:  return INFEASIBLE                       ▷ FR-5.6
18  return ChargingReroute(best, Fwd[best], Rev[best], arrival, recharge)
```

**Complexity.**

```
    Time   O((V + E) log V  +  |C|)        rather than   O(|C| · (V + E) log V)
    Space  O(V + E)
```

For a map with 20 nodes and 3 stations this is a threefold reduction; the asymptotic
improvement is what makes the design scale. This optimisation — recognising that the
all-targets output of one Dijkstra run subsumes many single-pair queries — is a central
algorithmic contribution of the project and should be highlighted in the presentation.

**Where the approximation can fail.** Lines 10 and 11 test the energy of the *minimum-cost*
path to and from each station. If that path is energy-infeasible but some costlier path is
feasible, the station is wrongly rejected. §8.4 exists for this case.

### 8.4 Exact Pareto-Label Variant  *(optional, selectable)*

A label-setting search where each label is `(cost, energy, node, parent)` and a label is
discarded only if another label at the same node dominates it in **both** cost and energy.
Labels with `energy > budget` are pruned at generation.

| | |
|---|---|
| Correctness | Returns the true minimum-cost path among all energy-feasible paths. |
| Time | O(L log L) where L is the number of Pareto-optimal labels; exponential in the worst case, small in practice on sparse geographic graphs. |
| Use | Selectable in the UI; §16 reports how often it differs from §8.3 and at what cost. |

### 8.5 Eligibility Decision

```
    for drone k and delivery p:
        R_direct ← route(k.pos → p.dest)
        if R_direct feasible by (8):        → eligible, cost = R_direct
        else:
            R_charge ← ChargingReroute(...)
            if R_charge ≠ INFEASIBLE:       → eligible, cost = R_charge
            else:                           → ineligible, reason recorded  (FR-7.6)
```

The recorded reason propagates into `Assignment.reason` and reaches the results table,
satisfying FR-10.7.

---

## 9. Prioritization and Fleet Assignment

### 9.1 Delivery Priority Queue  *(FR-6)*

Pending deliveries sit in the same hand-written min-heap, keyed by the tuple

```
    key(p)  =  ( p.priority.value ,  p.sequence )                    … (9)
```

Lexicographic comparison gives precedence by class, then strict FIFO within a class
*(FR-6.2)*. Because `sequence` is unique, the ordering is total and the schedule deterministic
*(P5)*. Push and pop are O(log n) *(FR-6.4)*.

**Optional aging** *(FR-6.6)*. If enabled, a NORMAL request waiting longer than `AGE_LIMIT`
is re-pushed with its class promoted one level. Documented as configurable and off by
default, so the baseline behaviour remains a pure priority queue.

### 9.2 All-Pairs Precomputation

Assignment must evaluate `travel_time(drone_position → destination)` for every
(delivery, drone) pair. Computing a fresh search per pair costs `O(P·D·(V+E) log V)`.

Instead, the orchestrator runs Dijkstra **once from every node**:

```
    Time   O(V · (V + E) log V)        Space   O(V²)
```

For the demonstration map (V ≈ 20–30) this is a few milliseconds and yields a complete
`cost / distance / energy / time / path` table. Assignment then becomes `O(1)` lookups. Since
a planning cycle evaluates far more than `V` pairs, the precomputation pays for itself
immediately, and it is cached across cycles per §11.

### 9.3 Greedy Makespan Assignment  *(FR-7.2, FR-7.3)*

The objective is to minimise the **makespan** — the completion time of the last delivery —
because that is what the project brief means by completing deliveries faster.

```
ALGORITHM AssignFleet(deliveries, drones, APSP, costModel)
  OUTPUT  list of Assignment, list of unserviceable

 1  Q ← MinHeap of deliveries keyed by (9)                   ▷ O(P)  heapify
 2  assignments ← [] ;  unserviceable ← []
 3  while Q not empty:                                       ▷ P iterations
 4      p ← Q.pop_min()                                      ▷ O(log P)
 5      best ← NIL ;  bestFinish ← ∞
 6      for each drone k in drones:                          ▷ D iterations
 7          (ok, route, reroute) ← Eligible(k, p, APSP)      ▷ O(1) table lookup
 8          if not ok:  continue                             ▷ FR-7.4
 9          finish ← k.ready_at + route.time_min
10                     + (reroute.recharge_min if reroute else 0)
11                     + SERVICE_TIME_MIN
12          if finish < bestFinish or (finish = bestFinish and k.id < best.id):
13              bestFinish ← finish ;  best ← k ;  bestRoute ← route
14      if best = NIL:
15          p.status ← UNSERVICEABLE ;  unserviceable.append(p)     ▷ FR-7.6
16          continue
17      best.ready_at   ← bestFinish                          ▷ FR-7.5
18      best.current_node ← p.destination
19      best.battery_pct ← UpdateBattery(best, bestRoute, reroute)
20      best.assigned.append(p.id)
21      p.status ← ASSIGNED
22      assignments.append(Assignment(p, best, bestRoute, reroute, …))
23  return assignments, unserviceable
```

Line 12's secondary comparison on drone id makes the tie-break total and deterministic.

**Complexity.**

```
    Assignment loop   O(P · D  +  P log P)
    Including APSP    O(V · (V + E) log V  +  P · D  +  P log P)
```

### 9.4 Analysis of the Greedy Strategy

This is **list scheduling**: process jobs in a fixed priority order, assigning each to the
machine that finishes it soonest. For `m` identical parallel machines with independent job
durations, Graham (1969) proves list scheduling is a `(2 − 1/m)`-approximation for makespan.

**The bound does not transfer directly here, and the documentation must say so.** Two
differences break the premise:

1. **Sequence-dependent durations.** A delivery's duration depends on where the drone
   currently is, so job times are not independent of assignment order.
2. **Feasibility coupling.** Battery state makes the set of *eligible* machines change over
   time, which the classical model does not admit.

The greedy strategy is therefore presented as a well-motivated heuristic with a known
analogue, not as an algorithm carrying a proven approximation ratio. §16 measures its actual
quality by comparing against an exhaustive optimal assignment on small instances — which is
the honest empirical substitute for a bound that does not apply.

**Known failure mode.** Because priority order is fixed by FR-6.3, a batch of URGENT
deliveries clustered in one district may load a single drone even where a later reassignment
would reduce the makespan. Greedy does not backtrack. This is documented rather than hidden,
and §21 notes the local-search improvement that would address it.

**Observed: Graham's timing anomaly reproduces here.** List scheduling is not monotone in
the number of machines — adding one can make the makespan *worse*, because it changes every
subsequent assignment decision. Graham (1969) named this for identical machines; on the
demonstration scenario it appears directly. Sweeping the roster with its configured charges:

| Drones | Makespan | Charging detours | Undelivered |
|---|---|---|---|
| 1 | 131 min | 2 | 1 |
| 2 | 107 min | 1 | 0 |
| 3 | 52 min | 0 | 0 |
| 4 | 20 min | 0 | 0 |
| 5 | **17 min** | 0 | 0 |
| 6 | **18 min** &nbsp;← worse than five | 0 | 0 |
| 7 | 12 min | 0 | 0 |
| 8 | 12 min | 0 | 0 |

**The cause is the scheduler, not the fleet's battery states.** An earlier reading of a
smaller roster attributed the anomaly to battery heterogeneity. A control run refutes that:
giving every drone an identical full charge — which removes heterogeneity entirely, and
removes charging detours with it — leaves the anomaly intact at exactly the same place.

| Drones (identical full charge) | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|
| Makespan | 32.7 | 20.4 | **16.7** | **17.9** ← | 11.9 | 11.9 |
| Charging detours | 0 | 0 | 0 | 0 | 0 | 0 |

What remains once battery state is held constant is the property that actually produces the
anomaly: **travel times are sequence-dependent.** A delivery's duration depends on where its
drone happens to be when it is assigned, so adding one aircraft re-partitions the batch and
can leave some drone with a materially worse tour. This is the classical anomaly rather than
anything peculiar to drones, and battery heterogeneity — when present — only adds to it.

Both the anomaly and the identical-battery control are asserted by tests, because a suite
demanding monotonicity would be asserting something false about greedy scheduling.

### 9.5 Choosing the fleet size  *(FR-7.9)*

The roster is a pool, not a launch order. Since more aircraft are not reliably better, the
number to dispatch is itself a decision:

```
ALGORITHM SizeFleet(table, roster, deliveries, tolerance)
 1  ordered ← roster sorted by battery descending, then id      ▷ best-charged first
 2  for k ← 1 .. |ordered|:
 3      plan[k] ← AssignFleet(table, ordered[1..k], deliveries) ▷ O(P·k + P log P)
 4  fewest ← min over k of |plan[k].unserviceable|
 5  viable ← { k : |plan[k].unserviceable| = fewest }           ▷ service level first
 6  best   ← argmin over viable of makespan
 7  return min { k ∈ viable : makespan[k] ≤ makespan[best]·(1+tolerance) }
```

Line 5 matters more than it looks. A fleet too small to finish the batch posts a *shorter*
makespan simply because it delivered less — one drone "finishes" a nineteen-parcel round in
40 minutes by abandoning ten of them. Comparing on speed before service level would select
exactly the worst option, so service level is filtered first and never traded away.

The tolerance (default 3 %) expresses the operator's trade: an aircraft that shaves a few
seconds off the schedule is not worth launching. Setting it to zero recovers pure
makespan-minimisation.

**Complexity.** `O(N)` assignments over the cached route table, so
`O(N·(P·N + P log P))`. For a roster of eight and a batch of twenty this is a few
milliseconds, which is why an exhaustive sweep over sizes is affordable rather than needing
a heuristic of its own.

---

## 10. Environment Model

### 10.1 No-Fly Zones  *(FR-8)*

A zone masks edges and nodes; it never mutates the graph. `NoFlyMask` exposes
`is_blocked(edge) -> bool`, consulted at line 12 of the Dijkstra pseudocode. Building the mask
is `O(E · Z)` for `Z` zones, performed once per configuration change and cached.

**Circle test.** An edge is blocked if the minimum distance from the zone centre to the
segment is less than the radius. Coordinates are projected to a local planar frame by the
equirectangular approximation about the map's mean latitude — accurate to well under a metre
at city scale, and far cheaper than exact spherical geometry:

```
    x = R · (lon − lon₀) · cos(lat₀)        y = R · (lat − lat₀)
```

Point-to-segment distance is then the standard clamped projection, in O(1).

**Polygon test.** An edge is blocked if either endpoint is inside the polygon (ray-casting
parity test, O(n) in the vertex count) or if the segment crosses any polygon edge (orientation
test via cross products, O(n)).

**Destination inside a zone.** The node itself is masked, the delivery is marked
UNSERVICEABLE, and the blocking zone is named in the reason *(FR-8.3)*.

**Penalty reporting** *(FR-8.6)*. The orchestrator computes each route twice — once with the
mask and once without — and reports the cost difference as the penalty attributable to the
zone. This is the clearest possible demonstration that the zones are genuinely affecting
routing rather than merely decorating the map.

### 10.2 Wind  *(FR-9)*

`Wind(speed_ms, bearing_deg)` is immutable. Its entire effect enters through equations (1)–(5)
inside `CostModel`; no other module is aware of it. Changing the wind invalidates the cache
(§11) and triggers a replan *(FR-9.5)*.

FR-9.6 requires a demonstrable case where wind alone changes the optimal route. The
demonstration map is constructed to contain such a pair: two roughly equal-cost corridors of
differing bearing, so that rotating the wind flips which is cheaper. This is a **scenario
design requirement on `data/city_map.json`**, verified by an automated test
(`test_wind_changes_route`) rather than left to chance.

---

## 11. Planning Orchestrator and Caching

### 11.1 Configuration Key

Steps 2–4 of the planning cycle are pure functions of the configuration, so their results are
cached under

```
    key = (α, β, γ, wind.speed, wind.bearing, frozenset(active_nfz_ids), map_version)
```

A cache hit skips the environment build, the cost-model build and the all-pairs
precomputation entirely, letting the UI sliders feel responsive *(NFR-3)*. Weights are
rounded to three decimals before keying, so slider jitter does not defeat the cache.

### 11.2 Invalidation

Any change to a key component invalidates the entry. Changes to the **delivery batch** or
**fleet state** do *not* invalidate it, since the all-pairs table depends only on the graph and
the cost model — a useful asymmetry, as adding a delivery is then nearly free.

The cache is an LRU of bounded size (default 16 entries), preventing unbounded growth as the
operator sweeps the sliders.

---

## 12. REST API Specification

All payloads are `application/json`; the server binds to `127.0.0.1:5000` by default
*(NFR-13, CI-1)*.

| Method | Endpoint | Purpose | Requirement |
|---|---|---|---|
| `GET` | `/api/scenario` | Current map, fleet, deliveries and zone definitions | FR-1.6, FR-2.4 |
| `POST` | `/api/plan` | Run a full planning cycle and return the plan, including the fleet-sizing decision | FR-7, FR-7.9, FR-10 |
| `POST` | `/api/route` | Single point-to-point route | FR-3 |
| `POST` | `/api/compare` | Dijkstra vs A* on one pair, with statistics | FR-3.4, FR-11.2 |
| `POST` | `/api/route/alternatives` | Minimum-distance and minimum-energy routes side by side | FR-4.7 |
| `GET` | `/api/deliveries` | The current batch | FR-2.4 |
| `POST` | `/api/deliveries` | Add a delivery request | FR-2.4, FR-2.7 |
| `DELETE` | `/api/deliveries/<id>` | Remove one order | FR-2.7 |
| `POST` | `/api/deliveries/clear` | Empty the batch | FR-2.7 |
| `GET` | `/api/presets` | List the prepared demonstration plans | FR-2.8 |
| `POST` | `/api/presets/<id>` | Load a plan, replacing the batch | FR-2.8, FR-2.9 |
| `POST` | `/api/benchmark` | Run the benchmark sweep | FR-11.3 |
| `GET` | `/api/export?format=json\|csv` | Export the current plan | FR-10.6, SI-3 |

### 12.1 `POST /api/plan`

Request:

```json
{
  "weights": {"alpha": 0.5, "beta": 0.3, "gamma": 0.2},
  "wind":    {"speed_ms": 6.0, "bearing_deg": 270},
  "active_no_fly_zones": ["nfz_airport"],
  "algorithm": "astar",
  "reserve_pct": 15,
  "constrained_mode": "two_phase"
}
```

Response (abbreviated):

```json
{
  "makespan_min": 47.3,
  "totals": {"distance_km": 61.4, "energy_pct": 182.7,
             "delivered": 9, "unserviceable": 1},
  "assignments": [
    {
      "delivery_id": "PKG-005", "drone_id": "D2", "destination": "C8",
      "priority": "HIGH", "algorithm": "astar",
      "route": {"path": ["W","A","C","F","C8"],
                "distance_km": 6.2, "energy_pct": 24.1,
                "time_min": 12.4, "cost": 0.318},
      "charging_stop": null,
      "depart_min": 0.0, "arrive_min": 12.4,
      "battery_before_pct": 60.0, "battery_after_pct": 35.9,
      "reason": "D2 completes at 14.4 min vs D1 at 18.1 min and D3 at 16.7 min",
      "stats": {"nodes_expanded": 11, "heap_operations": 34, "runtime_ms": 0.42}
    }
  ],
  "unserviceable": [
    {"delivery_id": "PKG-010", "reason":
     "Destination C11 lies inside active no-fly zone 'nfz_airport'"}
  ],
  "drones": [
    {"id": "D2", "deliveries": 4, "distance_km": 21.8,
     "energy_used_pct": 24.1, "idle_min": 3.2, "battery_pct": 35.9}
  ]
}
```

The `reason` field on every assignment and every rejection is what discharges FR-10.7: the
plan explains itself without the reader needing to consult the code.

### 12.2 Error Handling

| Condition | Status | Body |
|---|---|---|
| Malformed payload | 400 | `{"error": "...", "field": "..."}` |
| Weights not summing to 1 | 400 | `{"error": "alpha + beta + gamma must equal 1"}` |
| Unknown node id | 400 | `{"error": "unknown node 'X'"}` |
| Unreachable target | 200 | Route result with `"unreachable": true` *(FR-3.7)* |
| Internal failure | 500 | `{"error": "..."}` with the traceback logged server-side |

An unreachable target is a **valid outcome**, not an error, and so returns 200 with an explicit
flag — this is principle **P6** expressed in the API contract.

---

## 13. Frontend Design

### 13.1 Layout

```
┌────────────────────────────────────────────────────────────────────────┐
│  DDROS — Drone Delivery Route Optimization            [Plan] [Export]  │
├──────────────────────────────────────┬─────────────────────────────────┤
│                                      │  CONTROLS                       │
│                                      │  α distance ▁▃▅▇  0.50          │
│          LEAFLET MAP                 │  β energy   ▁▃▅▇  0.30          │
│                                      │  γ time     ▁▃▅▇  0.20          │
│   ▣ warehouse   ● customer           │  ─────────────────────────────  │
│   ⚡ charging    ◆ drone              │  Wind  6.0 m/s   bearing 270°   │
│   ▨ no-fly zone                      │  ↳ compass dial                 │
│                                      │  ─────────────────────────────  │
│   ─── D1   ─── D2   ─── D3           │  No-fly zones                   │
│                                      │   ☑ Airport approach            │
│   [▶ play] [⏸] [↻ reset]             │   ☐ Stadium                     │
│                                      │  ─────────────────────────────  │
│                                      │  Algorithm  ( ) Dijkstra        │
│                                      │             (•) A*              │
│                                      │             ( ) Compare both    │
├──────────────────────────────────────┴─────────────────────────────────┤
│  DELIVERY PLAN                                                         │
│  Pkg   Pri    Drone  Route              Dist  Energy Time  Algo  Why    │
│  B     URGENT D1     W→A→B→C7           4.1km 16.4%  8.2m  A*    …     │
│  D     HIGH   D2     W→A→C→F→C8         6.2km 24.1%  12.4m A*    …     │
├────────────────────────────────────────────────────────────────────────┤
│  ANALYSIS     nodes expanded · runtime vs V · Dijkstra ≡ A* check       │
└────────────────────────────────────────────────────────────────────────┘
```

### 13.2 Modules

| File | Responsibility |
|---|---|
| `map.js` | Leaflet initialisation, node markers, edge polylines, zone polygons, route rendering, colour legend |
| `controls.js` | Slider and toggle state, debounced replan requests, weight renormalisation to sum 1 |
| `results.js` | Plan table, per-drone panel, batch totals, unserviceable list |
| `charts.js` | Chart.js benchmark plots |
| `animation.js` | `requestAnimationFrame` interpolation of drone markers along route polylines, with play/pause/reset/scrub. Each drone is drawn as a quadcopter rotated to the bearing of the segment it is flying, with its rotors animated only while airborne, so direction of travel and flight state are both readable from the sprite (UI-5a). |

### 13.3 Interaction Notes

- Slider input is **debounced at 150 ms** so that dragging issues one request rather than
  dozens, which together with the §11 cache keeps replans inside the NFR-3 budget.
- Weights renormalise automatically: moving one slider redistributes the remainder across the
  other two, so the `α + β + γ = 1` constraint of FR-4.3 cannot be violated from the UI.
- Route colours are drawn from a palette chosen to remain distinguishable under
  deuteranopia and protanopia, and each route additionally carries a distinct dash pattern so
  that colour is never the sole channel *(NFR-21)*.
- Kestrel Bay is fictional, so there is no base map to tile. Its geography is drawn from
  `data/city_backdrop.json` — bay, river, parks and district labels — which makes the
  depiction unambiguously invented and leaves the application with **no network dependency
  at run time** *(C-4, FR-1.10, FR-1.11)*.
- A wheel gesture over the map scrolls the page rather than zooming Leaflet. With the default
  behaviour the reader cannot scroll past the map to the results at all; zoom stays available
  on the control, by double-click and by Ctrl+wheel *(UI-15)*.

---

## 14. Project Structure

```
Drone_Delivery/
├── README.md
├── requirements.txt
├── run.py                        # entry point: python run.py
├── docs/
│   ├── SRS.md
│   └── TDD.md
├── data/
│   ├── city_map.json             # Kestrel Bay: 34 nodes, 92 corridors (FR-1.8)
│   ├── city_backdrop.json        # drawn geography: water, river, parks (FR-1.11)
│   ├── fleet.json                # five drones
│   ├── deliveries.json
│   ├── presets.json              # prepared demonstration plans (FR-2.8)
│   └── no_fly_zones.json
├── ddros/
│   ├── __init__.py
│   ├── geo.py                    # haversine, bearing, projection, point-in-polygon
│   ├── domain/
│   │   ├── models.py             # §5.1 dataclasses
│   │   └── loader.py             # JSON -> domain objects
│   ├── graph/
│   │   ├── graph.py              # adjacency list, reversed()
│   │   └── validation.py         # §5.5 checks
│   ├── structures/
│   │   └── min_heap.py           # hand-written binary heap  (C-1)
│   ├── cost/
│   │   └── cost_model.py         # §6 equations (1)-(7)
│   ├── environment/
│   │   ├── wind.py
│   │   └── no_fly.py             # NoFlyMask, geometry tests
│   ├── algorithms/
│   │   ├── dijkstra.py           # §7.2
│   │   ├── astar.py              # §7.3
│   │   ├── heuristics.py         # Λ and h(n)
│   │   └── constrained.py        # §8.3 two-phase, §8.4 Pareto labels
│   ├── scheduling/
│   │   ├── priority_queue.py     # §9.1
│   │   ├── assignment.py         # §9.3 greedy makespan
│   │   └── fleet_sizing.py       # §9.5 how many drones to launch
│   ├── simulation/
│   │   ├── orchestrator.py       # §3.3 planning cycle
│   │   └── cache.py              # §11
│   ├── analysis/
│   │   ├── instrumentation.py    # SearchStats
│   │   ├── benchmark.py          # §16 sweeps
│   │   └── generators.py         # synthetic graphs, benchmark fixtures only
│   └── web/
│       ├── app.py                # Flask app factory
│       ├── api.py                # §12 endpoints
│       ├── templates/index.html
│       └── static/
│           ├── css/style.css
│           └── js/{map,controls,results,charts,animation}.js
└── tests/
    ├── test_geo.py               ├── test_no_fly.py
    ├── test_min_heap.py          ├── test_constrained.py
    ├── test_graph_validation.py  ├── test_priority_queue.py
    ├── test_cost_model.py        ├── test_assignment.py
    ├── test_dijkstra.py          ├── test_orchestrator.py
    ├── test_astar.py             ├── test_equivalence.py
    └── test_layering.py
```

> **Note on `analysis/generators.py`.** The application always uses the fixed demonstration
> map. The synthetic graph generator exists solely so the benchmark harness can vary `V` and
> `E` as FR-11.3 requires; it is never surfaced in the user interface. This resolves open item
> **O-1** in the SRS.

---

## 15. Complexity Analysis Summary

Notation: `V` vertices, `E` edges, `P` deliveries, `D` drones, `C` charging stations,
`Z` no-fly zones.

| Component | Operation | Time | Space |
|---|---|---|---|
| Graph construction | Build adjacency list | O(V + E) | O(V + E) |
| Graph validation | Connectivity via BFS | O(V + E) | O(V) |
| Min-heap | push / pop | O(log n) | O(n) |
| Min-heap | heapify | O(n) | O(n) |
| No-fly mask | Build over all edges and zones | O(E · Z) | O(E) |
| Cost model | Single edge evaluation | O(1) | O(1) |
| Heuristic | h(n) after Λ is precomputed | O(1) | O(1) |
| **Dijkstra** | Single source-target route | **O((V + E) log V)** | O(V + E) |
| **A\*** | Single source-target route | **O((V + E) log V)** worst case; fewer expansions in practice | O(V + E) |
| Charging reroute (two-phase) | Best station selection | **O((V + E) log V + C)** | O(V + E) |
| Charging reroute (naive) | *rejected alternative* | O(C · (V + E) log V) | O(V + E) |
| Charging reroute (Pareto) | Exact RCSPP | O(L log L), L labels; exponential worst case | O(L) |
| All-pairs precomputation | V Dijkstra runs | O(V · (V + E) log V) | O(V²) |
| Delivery priority queue | Build, then P pops | O(P log P) | O(P) |
| Greedy assignment | Given the all-pairs table | O(P · D) | O(P + D) |
| Fleet sizing | Sweep every roster size N | O(N · (P · N + P log P)) | O(P + N) |
| **Full planning cycle** | Cold cache | **O(V · (V + E) log V + P · D + P log P)** | O(V² + P + D) |
| **Full planning cycle** | Warm cache | **O(P · D + P log P)** | O(V² + P + D) |

For the demonstration map (`V ≈ 25`, `E ≈ 60`, `P = 10`, `D = 3`) the cold cycle is dominated
by the all-pairs term at roughly `25 × 85 × log 25 ≈ 10⁴` elementary operations — comfortably
inside the 1-second budget of NFR-2, with orders of magnitude to spare.

---

## 16. Experimental Plan

Satisfies FR-11 and supplies the "Analysis" half of the coursework.

### Experiment 1 — Dijkstra vs A*: nodes expanded

| | |
|---|---|
| **Hypothesis** | A* expands materially fewer nodes than Dijkstra for the same route, with the advantage greatest at `α = 1` and narrowing as `β` or `γ` grows (per the design note in §7.3). |
| **Method** | Synthetic graphs at `V ∈ {10, 25, 50, 100, 250, 500, 1000}`. For each, 50 random source-target pairs. Record nodes expanded by both algorithms. 10 repetitions; report mean ± SD *(FR-11.4)*. |
| **Output** | Line chart: nodes expanded vs V, two series. Bar chart: A* expansion ratio across weight settings `(1,0,0)`, `(0.5,0.3,0.2)`, `(0,1,0)`. |
| **Validation** | Assert total cost equality on every pair *(FR-11.6)*. Any mismatch falsifies admissibility and fails the run. |
| **Observed** | Confirmed, and the advantage *grows* with scale: A* expands 52.1 % of Dijkstra's nodes at V = 10, falling to 21.7 % at V = 1000. Averaged across sizes the ratio is 33.9 % under pure distance weighting, 41.0 % balanced and 51.4 % under pure energy weighting — matching the prediction of the §7.3 design note that A* degrades toward Dijkstra as the objective moves away from distance. Costs agreed on every pair at every size. |

### Experiment 2 — Empirical growth vs theoretical bound

| | |
|---|---|
| **Hypothesis** | Measured runtime grows as `O((V + E) log V)`. |
| **Method** | Same sweep; record wall-clock time. Plot runtime against `(V + E) log V` and fit a straight line. |
| **Output** | Scatter with fitted line and R². A high R² is direct empirical support for the derived bound. |
| **Observed** | R² = 0.9927 for Dijkstra and 0.9973 for A* across V ∈ {10 … 1000}. Measured runtime tracks `(V + E) log V` closely, supporting the bound derived in §7.2. |

### Experiment 3 — Value of energy-aware routing

| | |
|---|---|
| **Hypothesis** | *Per journey*, shifting weight from distance to energy reduces energy consumed at a modest cost in distance — the brief's Route A / Route B claim, quantified. **At batch level the effect is not monotonic**, for the reason given below. |
| **Method** | Two measurements, deliberately separated. (a) Per journey: for a fixed origin and destination, record distance and energy at `β ∈ {0.0, 0.2, …, 1.0}`. (b) Per batch: plan the whole batch at the same settings and record total distance, total energy, makespan and the number of charging detours triggered. |
| **Output** | Dual-axis chart of distance and energy vs β for (a), identifying the knee of the trade-off. For (b), a chart overlaying total batch energy with the charging-detour count. |
| **Observed** | (a) behaves as the brief predicts at every setting. (b) does **not**. On the Peak Load batch under an 18 m/s north-easterly, raising β from 0.0 to 0.1 added a third charging detour and pushed total batch energy *up* from 408 % to 433 %; beyond β = 0.5 the third detour disappears again and energy settles at 414 %. Energy-optimal routes are slower, which delays drones, which pushes a delivery past its feasible range and forces a detour whose extra leg costs more than the per-route saving recovers. **The effect requires the fleet to be near its feasibility boundary**: on a batch the drones absorb comfortably no detour is triggered and batch energy falls monotonically, exactly as the per-journey result predicts. The `charging_detours` column is what distinguishes the two regimes, which is why the experiment reports it alongside the energy total. |
| **Why it matters** | This is the most instructive result in the project. It shows that locally optimising each route does not optimise the schedule that contains them, because routing and scheduling are coupled through battery state and time. The correct conclusion is not that energy-aware routing fails, but that the objective must be set at the level the operator actually cares about. A mid-range β captures most of the per-route saving without triggering the detour. |

### Experiment 4 — Value of the fleet

| | |
|---|---|
| **Hypothesis** | Makespan falls substantially but sub-linearly as drones are added. |
| **Method** | Plan the same batch with `D ∈ {1, 2, 3, 4, 5}`. Record makespan and per-drone utilisation. |
| **Output** | Makespan vs D, with the ideal `1/D` curve overlaid to show the gap, plus per-row charging-detour and flight-time columns. Directly quantifies benefit §4.4 of the brief *(FR-11.7)*. |
| **Observed (anomaly)** | Makespan is **not monotone** in fleet size: six drones are slower than five (17.9 min against 16.7). This is Graham's timing anomaly, and §9.4 gives the measurement plus a control run showing it survives identical batteries and zero charging detours — so its cause is sequence-dependent travel inside the greedy assignment, not the fleet's battery states. It is the direct motivation for the fleet sizing of §9.5. |
| **Observed (superlinearity)** | The hypothesis was **wrong in an instructive way**: speedup is *super*-linear — 5.90× from three drones, against an ideal of 3×. This is not parallelism beating its own bound. Two further costs disappear as the fleet grows. One drone cannot carry the batch on a single charge and pays 53.5 minutes across three recharge cycles, which vanish entirely from three drones onward; and a single drone must chain all ten destinations into one sequential tour, spending 124.2 minutes airborne against 49.3 for three drones. The benchmark therefore reports detours and flight time per row, so the parallel speedup is separable from the recharge and tour-length savings rather than conflated with them. |

### Experiment 5 — Greedy assignment quality

| | |
|---|---|
| **Hypothesis** | Greedy makespan lies within a small factor of optimal on instances where optimal is computable. |
| **Method** | For `P ≤ 8` and `D ≤ 3`, enumerate all `D^P` assignments exhaustively to find the true optimum. Compare against greedy. 200 random instances. |
| **Output** | Histogram of the ratio greedy/optimal; report mean and worst case. This is the empirical substitute for the approximation bound that §9.4 explains does not transfer. |
| **Observed** | Over 120 instances at P = 6, D = 3: mean ratio **1.229** (SD 0.162), worst case **1.665**, and an optimal schedule found in only 8.3 % of instances. Greedy never beat the reference optimum, which is the check that the Held-Karp implementation is correct. The strategy is therefore fast but leaves roughly a quarter of the achievable makespan on the table — direct motivation for the local-search improvement listed in §21. Instances where greedy needed a charging detour are excluded, since the brute-force optimum models scheduling only; none arose at this size. |

### Experiment 6 — Two-phase vs exact constrained routing

| | |
|---|---|
| **Hypothesis** | The two-phase heuristic of §8.3 matches the exact Pareto search of §8.4 on the great majority of instances, at far lower cost. |
| **Method** | Generate instances with tight battery budgets, where the approximation is most likely to fail. Compare returned cost, feasibility verdict and runtime. |
| **Output** | Table: agreement rate, mean cost gap, speed-up factor. Reporting the disagreement rate honestly is the point of the experiment. |
| **Observed** | Two findings, the second unwelcome. **(1)** Agreement is 82.5 % on deliberately adversarial instances, and all 21 disagreements run the same way: the cheap test wrongly reports infeasibility. It never reported a route feasible when none existed, so the approximation is *conservative* — it can refuse a delivery it could have served, but never promises one it cannot. **(2)** Once the naive figure is timed fairly — the Dijkstra run *plus* the check, rather than the check alone — the exact Pareto search costs only about 1.2× more at this graph size. An earlier measurement that timed only the comparison made the heuristic look roughly 900× faster, which was an artefact of the instrumentation, not a property of the algorithm. The approximation is justified by scale, not by this scenario; at twenty-odd nodes the exact method is the better default. |

### Experiment 7 — Environmental sensitivity

| | |
|---|---|
| **Hypothesis** | Wind bearing alone can change the optimal route; no-fly zones impose a measurable, bounded penalty. |
| **Method** | Sweep wind bearing 0°–350° in 10° steps over fixed pairs and record the chosen path. Toggle each zone and record the cost delta. |
| **Output** | Polar plot of chosen route vs wind bearing *(FR-9.6)*; table of per-zone penalties *(FR-8.6)*. |
| **Observed** | Wind alone changes the optimal route on the demonstration map: C11→C12 and W→C8 each yield 4 distinct optimal routes across bearings, C12→C8 yields 3. Across the whole map 123 of 231 node pairs are wind-sensitive, and 31 pairs show the minimum-distance and minimum-energy routes diverging — C3→C4 most sharply, at 8.0 % longer for 10.6 % less energy, which is the brief's own Route A / Route B illustration reproduced from the model rather than asserted. Zone penalties: the airport corridor cuts off one customer and imposes a 3.4 % mean cost penalty on reroutes; the stadium polygon cuts off one and imposes 13.6 %. |

---

## 17. Testing Strategy

### 17.1 Levels

| Level | Scope | Tooling |
|---|---|---|
| Unit | Individual functions and data structures | pytest |
| Property | Invariants over randomised inputs | pytest with seeded randomisation |
| Integration | Full planning cycle over the demonstration scenario | pytest |
| Contract | API request/response shapes | Flask test client |
| Architectural | Layering rule of §3.2 | import-graph inspection |

### 17.2 Key Test Cases

| Test | Asserts | Requirement |
|---|---|---|
| `test_min_heap_matches_heapq` | Hand-written heap matches `heapq` over 10 000 random operations | C-1 |
| `test_dijkstra_known_graph` | Correct path on the brief's worked example: `W→B→E = 4 km`, beating `W→A→E = 8` and `W→C→D→E = 9` | FR-3.1 |
| `test_equivalence_dijkstra_astar` | Identical total cost over 500 random pairs at varied weights | FR-3.4, FR-11.6 |
| `test_heuristic_admissible` | `h(n) ≤ c*(n, target)` at every node, verified against a full Dijkstra from the target | FR-3.3 |
| `test_heuristic_consistent` | `h(u) ≤ c(u,v) + h(v)` for every edge | FR-3.3 |
| `test_cost_non_negative` | `c(e) ≥ 0` across the full weight simplex and wind range | FR-4.4 |
| `test_wind_symmetry` | Reversing wind bearing by 180° swaps head- and tailwind effects | FR-9.2 |
| `test_wind_changes_route` | A specified pair routes differently under two wind bearings | FR-9.6 |
| `test_energy_feasibility_respected` | No emitted route exceeds SoC less reserve | FR-4.6 |
| `test_charging_reroute_selects_min_cost` | Two-phase result matches brute force over all stations | FR-5.3 |
| `test_pareto_exact_dominates_two_phase` | Exact variant never returns a worse cost than the heuristic | §8.4 |
| `test_priority_ordering` | URGENT before HIGH before NORMAL; FIFO within class | FR-6.2 |
| `test_nfz_blocks_edges` | Edges crossing a zone are excluded; destination inside a zone is UNSERVICEABLE | FR-8.2, FR-8.3 |
| `test_assignment_updates_state` | Position, SoC and ready time update correctly after each assignment | FR-7.5 |
| `test_determinism` | Two identical runs produce byte-identical output | NFR-11 |
| `test_unreachable_handled` | Disconnected target yields an explicit unreachable result, not an exception | FR-3.7, NFR-10 |
| `test_layering` | No algorithm-tier module imports Flask or `ddros.web` | NFR-7 |

### 17.3 Coverage

Target ≥ 80 % statement coverage of the algorithm tier *(NFR-16)*, measured with
`pytest --cov=ddros`. The web tier is exercised by contract tests but is not held to the same
threshold, since its logic is thin by design.

---

## 18. Risks and Mitigations

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | A* heuristic subtly inadmissible at some weight setting, silently degrading optimality | Medium | High | Equality assertion against Dijkstra on every benchmark run *(FR-11.6)*, plus direct admissibility and consistency tests. A violation fails the build rather than passing unnoticed. |
| R2 | Frontend work crowds out algorithm work | High | High | Algorithm tier is completed and fully tested behind a CLI before any frontend work begins. The web layer is strictly a presentation skin over a system that already works. |
| R3 | Two-phase reroute rejects a genuinely feasible delivery | Medium | Medium | Documented in §8.2; exact Pareto variant implemented as a fallback; disagreement rate measured in Experiment 6. |
| R4 | Live replanning too slow, harming the demonstration | Low | Medium | Configuration-keyed cache (§11), 150 ms debounce, all-pairs precomputation. Budget verified against NFR-2 and NFR-3. |
| R5 | Leaflet tiles unavailable during the presentation | Medium | Low | Graceful degradation to a plain background *(C-4)*; the graph remains fully legible without tiles. |
| R6 | Floating-point tie-breaking makes results non-deterministic | Medium | Medium | All comparisons use an explicit epsilon; every tie-break falls through to a unique integer or string key *(P5)*. |
| R7 | Scope creep into unselected extensions | Medium | Medium | §1.2 of the SRS enumerates exclusions explicitly. Dynamic requests and payload capacity are out of scope and are recorded in §21 as future work. |
| R8 | Demonstration map fails to exhibit the intended phenomena | Medium | Medium | Map properties — a wind-sensitive pair, an energy-vs-distance pair, a zone-blockable route — are asserted by automated tests, not assumed. |
| R10 | Adding a drone can lengthen the schedule (Graham's anomaly, observed in §9.4) | Confirmed | Medium | Documented and asserted by tests, with a control run showing it survives identical batteries and zero detours — the cause is sequence-dependent travel inside the greedy assignment, not the fleet's battery states. Mitigated in practice by the fleet sizing of §9.5, which evaluates every size and will not select one that a smaller fleet beats. The local-search pass in §21 would remove the underlying cause. |
| R9 | Pure energy-weighted routing increases *total* batch energy by triggering charging detours (observed, see Experiment 3) | Confirmed | Medium | Documented as a finding rather than suppressed. The dashboard defaults to a balanced weighting; the analysis panel reports the charging-detour count alongside total energy so the coupling is visible rather than surprising. |

---

## 19. Work Distribution

The module decomposition below is the intended unit of work allocation. Owner cells are left
blank for the team to complete.

| Module | Files | Requirements | Owner |
|---|---|---|---|
| Geometry and domain model | `geo.py`, `domain/` | FR-1.2, FR-1.9 | |
| Graph and validation | `graph/` | FR-1.1, FR-1.5–FR-1.7 | |
| Data structures | `structures/min_heap.py` | FR-6.1, FR-6.4, C-1 | |
| Cost model and wind | `cost/`, `environment/wind.py` | FR-4.1–FR-4.5, FR-9.1–FR-9.4 | |
| Routing algorithms | `algorithms/dijkstra.py`, `astar.py`, `heuristics.py` | FR-3.1–FR-3.8 | |
| Constrained routing | `algorithms/constrained.py` | FR-5.1–FR-5.7 | |
| No-fly zones | `environment/no_fly.py` | FR-8.1–FR-8.6 | |
| Scheduling and assignment | `scheduling/` | FR-6, FR-7 | |
| Orchestrator and caching | `simulation/` | §3.3, §11 | |
| Benchmarking and analysis | `analysis/` | FR-11.1–FR-11.7 | |
| REST API | `web/api.py`, `web/app.py` | SI-1, §12 | |
| Frontend — map and animation | `static/js/map.js`, `animation.js` | UI-2–UI-5 | |
| Frontend — controls and results | `static/js/controls.js`, `results.js`, `charts.js` | UI-6–UI-10, FR-10 | |
| Test suite | `tests/` | NFR-16, §17 | |
| Documentation | `docs/` | — | |

---

## 20. Requirements Traceability

| Requirement group | Realised by | Section |
|---|---|---|
| FR-1 City map | `graph/graph.py`, `graph/validation.py`, `domain/loader.py` | §5.3, §5.5 |
| FR-2 Delivery requests | `domain/models.py`, `scheduling/priority_queue.py` | §5.1, §9.1 |
| FR-3 Route computation | `algorithms/dijkstra.py`, `algorithms/astar.py`, `heuristics.py` | §7.2, §7.3 |
| FR-4 Composite cost and energy | `cost/cost_model.py` | §6 |
| FR-5 Charging reroute | `algorithms/constrained.py` | §8 |
| FR-6 Prioritization | `scheduling/priority_queue.py`, `structures/min_heap.py` | §7.1, §9.1 |
| FR-7 Fleet assignment | `scheduling/assignment.py` | §9.3 |
| FR-7.9 Fleet sizing | `scheduling/fleet_sizing.py` | §9.5 |
| FR-8 No-fly zones | `environment/no_fly.py`, `geo.py` | §10.1 |
| FR-9 Wind | `environment/wind.py`, `cost/cost_model.py` | §6.1, §10.2 |
| FR-10 Results presentation | `web/api.py`, `static/js/results.js`, `map.js` | §12.1, §13 |
| FR-11 Analysis | `analysis/` | §7.4, §16 |
| NFR-6, NFR-7 | Layering rule and `test_layering.py` | §3.2 |
| NFR-1–NFR-5 Performance | Caching, all-pairs precomputation | §9.2, §11 |
| NFR-11 Determinism | Total tie-breaking throughout | §7.1, §9.1, §9.3 |

---

## 21. Future Work

Deliberately excluded from this release; recorded so the boundary is explicit.

| # | Extension | Note |
|---|---|---|
| 1 | **Dynamic delivery requests** | Requests arriving mid-simulation, with incremental replanning. The priority queue and orchestrator already support insertion; only the event loop is missing. |
| 2 | **Payload capacity** | Weight and volume constraints turning single-destination flights into multi-stop trips — effectively a capacitated vehicle routing problem. |
| 3 | **Local search over assignments** | A 2-opt or simulated-annealing pass over the greedy schedule, addressing the non-backtracking failure mode identified in §9.4. |
| 4 | **Heterogeneous fleet** | Per-drone airspeed, capacity and battery, relaxing assumption A-5. |
| 5 | **Charger queuing** | Modelling contention at charging stations, relaxing assumption A-6. |
| 6 | **Spatially varying wind** | A wind field rather than a uniform vector, relaxing assumption A-4. |
| 7 | **Bidirectional and ALT search** | Further routing speed-ups, giving a third and fourth algorithm for the comparative analysis. |

---

*End of Technical Design Document.*
