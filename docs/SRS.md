# Software Requirements Specification

## Drone Delivery Route Optimization System

**Document version:** 1.4
**Date:** 22 September 2026
**Prepared in accordance with:** IEEE Std 830-1998, *IEEE Recommended Practice for Software Requirements Specifications*

---

### Revision History

| Version | Date | Description |
|---|---|---|
| 1.0 | 22 Sep 2026 | Initial specification derived from the approved project brief |
| 1.1 | 22 Sep 2026 | Fictional city of Kestrel Bay (34 locations); fleet raised to five; operator-composed batches and prepared demonstration plans added |
| 1.2 | 22 Sep 2026 | Roster of eight with automatic fleet sizing; parcel-release animation on delivery; map sizing requirements |
| 1.3 | 22 Sep 2026 | En-route consolidation: a drone crossing a pending destination delivers it in passing |
| 1.4 | 23 Sep 2026 | Administrative cover details removed; the document stands on its technical content |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Overall Description](#2-overall-description)
3. [Specific Requirements](#3-specific-requirements)
4. [Appendices](#4-appendices)

---

## 1. Introduction

### 1.1 Purpose

This Software Requirements Specification (SRS) defines the functional and non-functional
requirements of the **Drone Delivery Route Optimization System (DDROS)**, a simulation
platform that plans, prioritizes and assigns unmanned aerial package deliveries across a
modelled city.

The document is intended for anyone implementing, reviewing or maintaining the system. It is the
authoritative statement of *what* the system must do. The companion
[Technical Design Document](TDD.md) specifies *how* those requirements are realised.

### 1.2 Scope

**Product name:** Drone Delivery Route Optimization System (DDROS)

DDROS is a **software simulation**. It does not interface with physical drone hardware,
flight controllers, or live telemetry. It models a fleet of five delivery drones operating
over a weighted graph representation of **Kestrel Bay**, a fictional coastal city of
thirty-four named locations, and automates four decisions that a human dispatcher would
otherwise make manually:

1. **Which delivery is handled next** — by priority rather than arrival order.
2. **Which drone handles it** — by which aircraft can complete it earliest.
3. **Which route that drone flies** — by a configurable trade-off between distance, flight
   time and energy consumption, not by distance alone.
4. **Whether the route is energy-feasible** — and, if not, which charging station to route
   through.

The system additionally models two environmental factors that alter these decisions:
**no-fly zones** (restricted airspace that removes edges from the graph) and **wind**
(which raises or lowers the energy cost of an edge depending on heading).

**Benefits.** Automating these decisions reduces total delivery time, reduces battery
consumption per delivery, prevents urgent packages from queuing behind routine ones,
distributes work across the fleet instead of overloading a single aircraft, and lowers
operational cost by eliminating unnecessary travel.

**Analytical objective.** The system has a second purpose beyond planning deliveries: it must
show graphs, adjacency lists, priority queues, Dijkstra's algorithm, A* search and greedy
strategies operating together inside one realistic problem, and it must support **empirical
measurement** of those algorithms alongside their derived complexity. Every complexity claim
the design makes is therefore measured rather than asserted.

**Out of scope.** The following are explicitly excluded from this release:

- Physical drone control, flight firmware, or any hardware interface.
- Real-time telemetry ingestion or live GPS tracking.
- Dynamic delivery requests arriving mid-simulation (the system plans a static batch).
- Drone payload weight and volume capacity constraints.
- Multi-user accounts, authentication, authorization, or persistent server-side storage.
- Real-world street network data or third-party routing services.
- Regulatory airspace compliance certification.

### 1.3 Definitions, Acronyms and Abbreviations

| Term | Definition |
|---|---|
| **A\*** | A best-first graph search algorithm that uses an admissible heuristic to direct the search toward the goal. |
| **Adjacency list** | A graph representation storing, for each vertex, the list of its incident edges. Space complexity O(V + E). |
| **Admissible heuristic** | A heuristic that never overestimates the true remaining cost to the goal; required for A* optimality. |
| **Bearing** | Compass direction of travel along an edge, measured in degrees clockwise from true north. |
| **Consistent heuristic** | A heuristic satisfying h(n) ≤ c(n, n') + h(n'); guarantees each node is expanded at most once by A*. |
| **DDROS** | Drone Delivery Route Optimization System — the product specified herein. |
| **Dijkstra's algorithm** | Single-source shortest-path algorithm for graphs with non-negative edge weights. |
| **Edge** | A traversable air corridor between two locations, weighted by distance, time and energy. |
| **Greedy algorithm** | A strategy that commits to the locally optimal choice at each step without backtracking. |
| **Haversine distance** | Great-circle distance between two latitude/longitude points on a sphere. |
| **Headwind / tailwind** | The component of the wind vector opposing / assisting the direction of flight. |
| **Makespan** | The completion time of the *last* delivery in a schedule; the quantity the fleet assignment minimises. |
| **Min-heap** | A binary heap where the minimum element is at the root; the concrete data structure backing the priority queue. |
| **NFZ** | No-Fly Zone — a geographic region through which flight is prohibited. |
| **Node / vertex** | A discrete location in the city model: warehouse, customer, charging station or waypoint. |
| **Nodes expanded** | Count of vertices removed from the priority queue during a search; the primary efficiency metric for comparing Dijkstra and A*. |
| **Priority queue** | An abstract data type returning the highest-priority element first; used for both delivery ordering and search frontier management. |
| **Reserve margin** | A percentage of battery held back as a safety buffer and never consumed by planned routes. |
| **SoC** | State of Charge — a drone's remaining battery, expressed as a percentage. |

### 1.4 References

1. **Project Brief.** *Drone Delivery Route Optimization System — Design & Analysis of
   Algorithms Project Brief*, `Drone_Delivery_Route_Optimization_Beautiful.pdf`, 8 pp.
   (The originating requirements source; this SRS traces to its §2.1–§2.7.)
2. IEEE Std 830-1998, *IEEE Recommended Practice for Software Requirements Specifications*.
3. Cormen, T. H., Leiserson, C. E., Rivest, R. L., Stein, C. *Introduction to Algorithms*,
   4th ed., MIT Press, 2022. (Chs. 20–22: elementary graph algorithms, single-source
   shortest paths.)
4. Hart, P. E., Nilsson, N. J., Raphael, B. "A Formal Basis for the Heuristic Determination
   of Minimum Cost Paths", *IEEE Transactions on Systems Science and Cybernetics*, 4(2),
   1968, pp. 100–107. (Original statement of A* and its admissibility condition.)
5. Graham, R. L. "Bounds on Multiprocessing Timing Anomalies", *SIAM Journal on Applied
   Mathematics*, 17(2), 1969, pp. 416–429. (List scheduling and the makespan bound.)
6. Handler, G. Y., Zang, I. "A Dual Algorithm for the Constrained Shortest Path Problem",
   *Networks*, 10(4), 1980, pp. 293–309. (The resource-constrained shortest path problem
   underlying energy-feasible routing.)

### 1.5 Overview

Section 2 describes the product's context, principal functions, users, constraints and
assumptions in narrative form. Section 3 states the detailed, individually numbered and
independently verifiable requirements. Section 4 provides the traceability matrix linking
every requirement back to the originating project brief.

Requirements are identified as **FR-n.m** (functional) and **NFR-n** (non-functional).
The keywords **shall** (mandatory), **should** (recommended) and **may** (optional) are used
throughout with their conventional normative meaning.

---

## 2. Overall Description

### 2.1 Product Perspective

DDROS is a **new, self-contained product**. It is not a component of, nor a replacement for,
any existing system, and it has no external system interfaces.

The product is organised as a three-tier application executing entirely on a single host:

```
┌─────────────────────────────────────────────────────────┐
│  PRESENTATION TIER — browser                            │
│  Leaflet map · control panel · results table · charts   │
└───────────────────────────┬─────────────────────────────┘
                            │  HTTP / JSON (localhost)
┌───────────────────────────┴─────────────────────────────┐
│  APPLICATION TIER — Python web service                  │
│  REST endpoints · request validation · orchestration    │
└───────────────────────────┬─────────────────────────────┘
                            │  in-process calls
┌───────────────────────────┴─────────────────────────────┐
│  ALGORITHM TIER — pure Python, no framework coupling    │
│  Graph · Dijkstra · A* · priority queue · greedy        │
│  assignment · cost model · environment model            │
└─────────────────────────────────────────────────────────┘
```

The algorithm tier is deliberately free of any web-framework dependency so that it can be
exercised directly by the unit-test suite and the benchmarking harness without starting a
server.

**Data stores.** The city map, drone fleet and delivery batch are supplied as JSON files
read at start-up. No database is used, and no state persists between runs beyond exported
result files.

### 2.2 Product Functions

The system performs eight principal functions:

| Ref | Function | Summary |
|---|---|---|
| F1 | **City modelling** | Load and validate a weighted graph of city locations and the air corridors connecting them. |
| F2 | **Delivery intake** | Accept a batch of delivery requests, each with a destination and a priority class, composed by the operator or loaded from a prepared demonstration plan. |
| F3 | **Route optimization** | Compute an optimal route between two locations using Dijkstra's algorithm or A*, under a configurable composite cost. |
| F4 | **Energy-aware routing** | Evaluate routes on energy consumption as well as distance, and reject routes exceeding a drone's usable charge. |
| F5 | **Charging-station rerouting** | When no direct route is energy-feasible, plan a two-leg route through the charging station that minimises total cost. |
| F6 | **Delivery prioritization** | Order the delivery batch so urgent consignments are dispatched ahead of routine ones. |
| F7 | **Fleet assignment** | Distribute deliveries across available drones so the whole batch completes as early as possible. |
| F8 | **Visualization & analysis** | Present the resulting plan on an interactive map with full decision detail, and report measured algorithm performance. |

### 2.3 User Characteristics

| User class | Description | Technical expertise | Frequency of use |
|---|---|---|---|
| **Reviewer** | Reads the system to judge correctness, algorithmic depth and the quality of the analysis. | Expert in algorithms; not assumed familiar with this codebase. | Occasionally, on first contact. |
| **Operator** | Person driving the simulation during a live demonstration — loading scenarios, adjusting weights and wind, triggering replanning. | General computing literacy; no programming required. | Frequently during demonstration. |
| **Developer** | The author, or anyone later extending or maintaining the system. | Proficient in Python and familiar with graph algorithms. | Continuously during development. |

Because the Reviewer is a first-time reader, **every decision the system makes shall be
displayed with the reasoning behind it** — the drone chosen, the route taken, the distance,
the energy consumed, the priority class and the algorithm used. The system must never
present an unexplained result.

### 2.4 Constraints

| Ref | Constraint |
|---|---|
| C-1 | **Algorithms shall be implemented from first principles.** Dijkstra, A*, the min-heap priority queue and the greedy assignment strategy shall be written for this project. Calling a library shortest-path routine (e.g. `networkx.shortest_path`, `scipy.sparse.csgraph`) to satisfy FR-3 is prohibited, as the algorithms are the substance of the work. |
| C-2 | Edge weights shall be non-negative, which is a precondition of Dijkstra's correctness. |
| C-3 | The A* heuristic shall be admissible and consistent, and this property shall be justified in the design documentation. |
| C-4 | The system shall run offline on a single machine. Because the city is fictional and its backdrop is drawn from local data, no base-map or other network service is contacted at run time. |
| C-5 | The implementation language shall be Python 3.11 or later. |
| C-6 | The system shall run on Windows, macOS and Linux without modification. |
| C-7 | The complete system shall be demonstrable within a 15-minute presentation slot. |
| C-8 | No third-party paid service, API key or cloud account shall be required. |

### 2.5 Assumptions and Dependencies

**Assumptions**

| Ref | Assumption |
|---|---|
| A-1 | The city map is static for the duration of a simulation run; roads and corridors do not appear or disappear except through no-fly-zone activation. |
| A-2 | Drones fly at a constant nominal airspeed; acceleration, climb and descent profiles are not modelled. |
| A-3 | Energy consumption is proportional to distance flown, scaled by a wind factor. Payload mass, temperature and battery age are not modelled. |
| A-4 | Wind is uniform across the entire map and constant during a planning cycle. |
| A-5 | All drones in the fleet are homogeneous in speed and battery capacity, differing only in current state of charge and position. The supplied roster of eight drones starts at differing charges; the scheduling anomaly recorded in TDD §9.4 is present with or without that heterogeneity. |
| A-6 | Charging stations are always available; queuing for a charger is not modelled. |
| A-7 | Delivery service time at the destination is a fixed constant, identical for all packages. |
| A-8 | The complete delivery batch is known before planning begins. |

**Dependencies**

| Ref | Dependency |
|---|---|
| D-1 | Python 3.11+ runtime. |
| D-2 | A Python web framework (Flask or FastAPI) for the application tier. |
| D-3 | Leaflet.js for interactive map rendering in the browser. |
| D-4 | A charting library for benchmark visualisation. |
| D-5 | A modern browser with JavaScript enabled. |
| D-6 | `pytest` for the automated test suite. |

---

## 3. Specific Requirements

### 3.1 External Interface Requirements

#### 3.1.1 User Interfaces

| Ref | Requirement |
|---|---|
| UI-1 | The system **shall** present a single-page web application comprising a map panel, a control panel, a results panel and an analysis panel. |
| UI-2 | The map panel **shall** render the city graph over a Leaflet base map, drawing nodes as type-distinguished markers (warehouse, customer, charging station, waypoint) and edges as connecting polylines. |
| UI-3 | The map panel **shall** render active no-fly zones as shaded polygons visually distinct from all other map features. |
| UI-4 | The map panel **shall** highlight each computed route as a coloured polyline, using one distinct colour per drone, with a legend mapping colours to drone identifiers. |
| UI-5 | The map panel **shall** animate a drone marker travelling along its assigned route, with play, pause, reset and scrub controls on a shared clock. |
| UI-5a | The drone marker **shall** be drawn as a recognisable rotorcraft, **shall** be oriented to its current heading, and **shall** visibly indicate flight — for example by animating its rotors — only while that drone is actually in transit. Motion **shall** respect a reduced-motion preference. |
| UI-6 | The control panel **shall** provide continuous sliders for the cost weights α (distance), β (energy) and γ (time). |
| UI-7 | The control panel **shall** provide controls for wind speed and wind bearing, and a toggle for each defined no-fly zone. |
| UI-8 | The control panel **shall** provide a selector for the routing algorithm: Dijkstra, A*, or side-by-side comparison. |
| UI-9 | Altering any control in UI-6 to UI-8 **shall** trigger recomputation, and the map and results **shall** update to reflect the new plan. |
| UI-10 | The results panel **shall** display, for every delivery, the fields mandated by FR-10.3. |
| UI-11 | The interface **shall** report errors (unreachable destination, no feasible drone, invalid input) as a clear message identifying the affected delivery, and **shall not** fail silently. |
| UI-12 | The interface **shall** be legible at a projector resolution of 1280×720 or greater. |
| UI-13 | The control panel **shall** present the prepared demonstration plans with their names, descriptions and order counts, and loading one **shall** replan immediately. |
| UI-14 | The control panel **shall** provide an order manager listing the current batch, with a control to remove any single order, a control to clear the batch, and a form to add an order by destination and priority. Destinations **shall** be offered by name and district, not by identifier. |
| UI-15 | A wheel gesture over the map **shall** scroll the page rather than zoom the map, so the reader is never trapped above the results. Zoom **shall** remain available by explicit control. |
| UI-16 | The control panel **shall** offer a fleet-size control with an automatic setting and a forced setting for any size within the roster, and the results **shall** show which sizes were evaluated, what each achieved and why one was chosen *(FR-7.9, FR-7.11, FR-7.12)*. |
| UI-17 | On delivery, the map **shall** show the parcel being released at the destination. The effect **shall** play only during playback, **shall not** replay in bulk when the clock is scrubbed, and **shall** be suppressed under a reduced-motion preference. |
| UI-18 | The map **shall** occupy as much of the viewport height as the surrounding controls allow, and **shall** offer a mode in which it fills the viewport entirely. |

#### 3.1.2 Hardware Interfaces

| Ref | Requirement |
|---|---|
| HI-1 | The system requires no hardware interface beyond a standard keyboard, pointing device and display. It **shall not** communicate with drone hardware, radio links or GPS receivers. |

#### 3.1.3 Software Interfaces

| Ref | Requirement |
|---|---|
| SI-1 | The application tier **shall** expose a JSON-over-HTTP interface to the presentation tier. Request and response schemas are specified in the Technical Design Document. |
| SI-2 | The system **shall** read scenario data — city map, drone fleet, delivery batch, no-fly zones — from UTF-8 encoded JSON files conforming to the published schema. |
| SI-3 | The system **shall** export simulation results as CSV from the interface, and as JSON through the API and the command line. |
| SI-4 | The system **shall not** depend on any network service at run time other than optional map tiles. |

#### 3.1.4 Communications Interfaces

| Ref | Requirement |
|---|---|
| CI-1 | Client and server **shall** communicate over HTTP on a configurable localhost port, default 5000. |
| CI-2 | All request and response payloads **shall** be `application/json`. |

---

### 3.2 Functional Requirements

Requirements are grouped by capability. The **Brief** column traces each group to the
section of the originating project brief that mandates it.

---

#### FR-1 — City Map Representation  *(Brief §2.1)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-1.1 | The system **shall** represent the city as a weighted, undirected graph G = (V, E). | Must |
| FR-1.2 | Each vertex **shall** carry a unique identifier, a human-readable name, a latitude, a longitude and a type drawn from {WAREHOUSE, CUSTOMER, CHARGING_STATION, WAYPOINT}. | Must |
| FR-1.3 | Each edge **shall** carry a base distance in kilometres, a base flight time in minutes and a base energy consumption expressed as a percentage of full charge. | Must |
| FR-1.4 | All edge weights **shall** be strictly non-negative. | Must |
| FR-1.5 | The graph **shall** be stored as an adjacency list, giving O(V + E) space. | Must |
| FR-1.6 | The system **shall** load the graph from a JSON map file and **shall** reject a malformed file with a diagnostic naming the offending element. | Must |
| FR-1.7 | On load, the system **shall** validate that the graph is connected, that every edge references existing vertices, that no edge is a self-loop, and that all weights are non-negative. Validation failure **shall** abort the load with an explanatory error. | Must |
| FR-1.8 | The supplied demonstration map **shall** contain at least 30 vertices, including exactly one warehouse, at least four charging stations and at least twenty customer locations. | Must |
| FR-1.10 | The city **shall** be fictional. Every location **shall** carry an invented name and a named district, and the map **shall not** depict or be rendered over any real geography. | Must |
| FR-1.11 | The system **shall** supply a drawn backdrop for the fictional city — water, watercourse, parks and district labels — so that no external base-map service is required at run time. | Must |
| FR-1.9 | Edge distance **should** be derivable from vertex coordinates by the haversine formula, so that the map remains geometrically self-consistent and the A* heuristic remains admissible. | Should |

---

#### FR-2 — Delivery Request Management  *(Brief §2.2)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-2.1 | The system **shall** accept a batch of delivery requests, each carrying a unique package identifier, a destination vertex and a priority class. | Must |
| FR-2.2 | Priority **shall** be one of URGENT, HIGH or NORMAL, ranked in that order of precedence. | Must |
| FR-2.3 | The system **shall** validate that each destination identifies an existing vertex, and **shall** reject a request naming an unknown destination with a diagnostic. | Must |
| FR-2.4 | The system **shall** load a delivery batch from JSON and **shall** additionally permit a request to be added interactively. | Must |
| FR-2.5 | Each request **shall** record a submission sequence number, used to break ties between requests of equal priority. | Must |
| FR-2.6 | The system **shall** maintain a status for each request: PENDING, ASSIGNED, DELIVERED or UNSERVICEABLE. | Must |
| FR-2.7 | The operator **shall** be able to compose a batch by hand: adding an order by choosing a destination and a priority, removing an individual order, and clearing the batch entirely. | Must |
| FR-2.8 | The system **shall** supply at least three prepared demonstration plans, each with a name and a description of the behaviour it exhibits, and the operator **shall** be able to load any of them in a single action. | Must |
| FR-2.9 | Loading a plan **shall** replace the batch entirely and renumber it, so the loaded plan dispatches in its own order rather than inheriting the sequence of whatever it replaced. | Must |
| FR-2.10 | An empty batch **shall** be a valid state, producing an empty plan rather than an error. | Must |

---

#### FR-3 — Route Computation  *(Brief §2.3)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-3.1 | The system **shall** implement Dijkstra's algorithm, backed by a binary min-heap, to compute a minimum-cost route between two vertices. | Must |
| FR-3.2 | The system **shall** implement A* search using a haversine-based distance heuristic. | Must |
| FR-3.3 | The A* heuristic **shall** be admissible and consistent with respect to the active composite cost function, for every admissible setting of the cost weights. | Must |
| FR-3.4 | Both algorithms **shall**, for the same graph, cost configuration and vertex pair, return routes of identical total cost. | Must |
| FR-3.5 | A route result **shall** report the ordered vertex sequence, total distance, total flight time, total energy consumption, total composite cost and the algorithm used. | Must |
| FR-3.6 | A route result **shall** additionally report the number of vertices expanded, the number of heap operations performed and the wall-clock execution time. | Must |
| FR-3.7 | Where no route exists between the two vertices, the system **shall** return an explicit unreachable result rather than raising an unhandled error. | Must |
| FR-3.8 | Routing **shall not** be performed by any third-party shortest-path library (see constraint C-1). | Must |

---

#### FR-4 — Composite Cost and Energy Awareness  *(Brief §2.4)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-4.1 | The system **shall** evaluate each edge under a composite cost combining distance, energy and flight time. | Must |
| FR-4.2 | The three contributing terms **shall** be normalised to a common dimensionless scale before combination, so that the weights are directly comparable. | Must |
| FR-4.3 | The weights α, β and γ **shall** be adjustable at run time through the user interface, and **shall** be constrained to α + β + γ = 1 with each weight in [0, 1]. | Must |
| FR-4.4 | The composite cost of every edge **shall** be non-negative for every admissible weight setting, preserving the precondition of Dijkstra's correctness. | Must |
| FR-4.5 | The system **shall** compute the total energy consumption of a route as the sum of the wind-adjusted energy costs of its edges. | Must |
| FR-4.6 | A route **shall** be deemed energy-feasible for a drone only where its total energy consumption does not exceed the drone's state of charge less a configurable reserve margin, which **shall** default to 15 %. | Must |
| FR-4.7 | The system **shall** be able to compute, and display side by side, the minimum-distance route and the minimum-energy route between the same pair of vertices, so that the distinction between them is evident. | Must |
| FR-4.8 | Where the two routes of FR-4.7 differ, the interface **should** annotate the difference explicitly, for example "6 % longer, 15 % less energy". | Should |

---

#### FR-5 — Charging-Station Rerouting

| Ref | Requirement | Priority |
|---|---|---|
| FR-5.1 | Where no direct route from a drone's position to a destination is energy-feasible, the system **shall** attempt a two-leg route passing through a charging station. | Must |
| FR-5.2 | The first leg **shall** be energy-feasible on the drone's current charge; the second leg **shall** be energy-feasible on a full charge. | Must |
| FR-5.3 | Where several charging stations yield a feasible two-leg route, the system **shall** select the one minimising total composite cost inclusive of recharge time. | Must |
| FR-5.4 | Recharge time **shall** be modelled as a function of the charge replenished, using a configurable rate in percent per minute. | Must |
| FR-5.5 | A rerouted result **shall** identify the charging station used, the charge level on arrival, the recharge duration and the per-leg breakdown. | Must |
| FR-5.6 | Where no feasible route exists even through a charging station, the delivery **shall** be marked UNSERVICEABLE for that drone and offered to the remaining fleet. | Must |
| FR-5.7 | The documentation **shall** state that the exact energy-constrained shortest path problem is NP-hard, and **shall** characterise the approximation the implementation uses. | Must |

---

#### FR-6 — Delivery Prioritization  *(Brief §2.5)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-6.1 | The system **shall** maintain pending deliveries in a priority queue implemented as a binary min-heap. | Must |
| FR-6.2 | The queue **shall** order requests by priority class first, and by submission sequence number second, giving deterministic first-in-first-out behaviour within a class. | Must |
| FR-6.3 | The system **shall** dispatch deliveries strictly in the order yielded by the queue. | Must |
| FR-6.4 | Insertion and extraction **shall** each operate in O(log n) time. | Must |
| FR-6.5 | The interface **shall** display the pending queue in its current dispatch order, so that prioritization is directly observable. | Must |
| FR-6.6 | The system **may** apply an aging policy that promotes a long-waiting NORMAL request, preventing indefinite starvation. Where implemented, the policy **shall** be documented and configurable. | May |

---

#### FR-7 — Multi-Drone Assignment  *(Brief §2.6)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-7.1 | The system **shall** maintain a fleet of drones, each with a unique identifier, a current vertex, a state of charge, a ready time and an ordered list of assigned deliveries. | Must |
| FR-7.2 | The system **shall** assign deliveries using a greedy strategy whose objective is to minimise the makespan — the completion time of the final delivery in the batch. | Must |
| FR-7.3 | For each delivery taken from the priority queue, the system **shall** select the eligible drone minimising projected completion time, that being its ready time plus its travel time to the destination plus the fixed service time. | Must |
| FR-7.4 | A drone **shall** be eligible for a delivery only where an energy-feasible route exists, directly or via a charging station. | Must |
| FR-7.5 | Following each assignment, the system **shall** update the selected drone's position, state of charge and ready time. | Must |
| FR-7.6 | Where no drone in the fleet is eligible, the delivery **shall** be marked UNSERVICEABLE and reported with the reason. | Must |
| FR-7.7 | The system **shall** report per-drone utilisation: deliveries assigned, distance flown, energy consumed and idle time. | Must |
| FR-7.8 | The system **shall** report the makespan of the resulting schedule, and **should** contrast it with the single-drone baseline to quantify the benefit of the fleet. | Must / Should |
| FR-7.9 | The fleet **shall** be treated as a roster rather than a launch order. The system **shall** select how many drones to dispatch, evaluating every size and choosing the smallest that comes within a configurable tolerance of the best achievable makespan. Drones not dispatched **shall** be reported as reserve. | Must |
| FR-7.10 | Fleet sizes **shall** be compared on service level before speed. A fleet too small to complete the batch posts a shorter makespan only because it delivered less, and **shall not** be selected on that basis. | Must |
| FR-7.11 | The operator **shall** be able to override the selected size and force any size within the roster, so the effect of the choice can be demonstrated. | Must |
| FR-7.12 | The system **shall** report which sizes were evaluated, what each achieved, and why the chosen one was selected. | Must |
| FR-7.13 | Where a drone's chosen route passes through the destination of another pending delivery, the system **shall** be able to deliver that parcel in passing rather than dispatching a second drone to a location already being overflown. | Must |
| FR-7.14 | En-route consolidation **shall** be governed by a selectable policy: **off**, **safe** (a parcel may ride along only if it is at least as urgent as the delivery whose route it is on) or **always**. The default **shall** be **safe**, so priority order is never inverted without the operator asking for it. | Must |
| FR-7.15 | An en-route delivery **shall** add no distance and no energy to the fleet totals, since it shares a flight already accounted for, and its route **shall** be reported as the prefix of that flight up to the drop point. | Must |
| FR-7.16 | No delivery **shall** be assigned more than once, by any combination of direct assignment and en-route consolidation. | Must |

---

#### FR-8 — No-Fly Zones  *(Extension)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-8.1 | The system **shall** support no-fly zones defined as circles, given a centre and a radius, or as polygons, given an ordered vertex list. | Must |
| FR-8.2 | An edge whose great-circle path intersects an active no-fly zone **shall** be excluded from all route computations. | Must |
| FR-8.3 | A vertex lying inside an active no-fly zone **shall** be excluded from routing; where it is a delivery destination, that delivery **shall** be marked UNSERVICEABLE with the zone identified as the cause. | Must |
| FR-8.4 | No-fly zones **shall** be individually activatable and deactivatable at run time, and any change **shall** trigger replanning. | Must |
| FR-8.5 | Active zones **shall** be rendered on the map, visually distinct from all other features. | Must |
| FR-8.6 | The system **should** report, for each affected delivery, the cost penalty incurred by routing around a zone relative to the unrestricted route. | Should |

---

#### FR-9 — Wind and Weather Modelling  *(Extension)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-9.1 | The system **shall** model wind as a uniform vector over the map, defined by a speed and a bearing. | Must |
| FR-9.2 | The system **shall** compute, for each edge, the along-track wind component from the edge's bearing and the wind vector. | Must |
| FR-9.3 | A tailwind **shall** reduce, and a headwind **shall** increase, the edge's energy cost and flight time relative to their base values. | Must |
| FR-9.4 | The wind-adjusted energy multiplier **shall** be clamped to a bounded range, ensuring costs remain strictly positive and Dijkstra's precondition is preserved. | Must |
| FR-9.5 | Wind speed and bearing **shall** be adjustable at run time, and any change **shall** trigger replanning. | Must |
| FR-9.6 | The system **should** demonstrate that a change in wind alone can change the optimal route between an unchanged pair of vertices. | Should |
| FR-9.7 | The interface **should** display the active wind vector on the map. | Should |

---

#### FR-10 — Results Presentation  *(Brief §2.7)*

| Ref | Requirement | Priority |
|---|---|---|
| FR-10.1 | On completing a planning cycle, the system **shall** present the full delivery plan. | Must |
| FR-10.2 | The plan **shall** be rendered both graphically on the map and in tabular form. | Must |
| FR-10.3 | For each delivery, the system **shall** display: the selected drone, the package identifier, the destination, the ordered route as vertex names, the total distance in kilometres, the estimated energy consumption as a percentage, the priority class, the algorithm used, the estimated delivery time, and any charging stop. | Must |
| FR-10.4 | The system **shall** display, per drone, its remaining state of charge and its delivery timeline. | Must |
| FR-10.5 | The system **shall** display batch-level totals: makespan, total distance, total energy, deliveries completed and deliveries unserviceable. | Must |
| FR-10.6 | The system **shall** export the complete plan. The interface **shall** offer CSV, which is the format a reader opens directly; JSON remains available through `/api/export?format=json` and `run.py plan --json` for anything consuming the plan programmatically. | Must |
| FR-10.7 | Every displayed decision **shall** be accompanied by the reason for it, in keeping with the explainability obligation of §2.3. | Must |

---

#### FR-11 — Algorithm Analysis and Benchmarking

| Ref | Requirement | Priority |
|---|---|---|
| FR-11.1 | Every search **shall** be instrumented to record vertices expanded, heap operations and wall-clock time. | Must |
| FR-11.2 | The system **shall** provide a benchmarking harness that executes Dijkstra and A* over identical inputs and reports their comparative performance. | Must |
| FR-11.3 | The harness **shall** evaluate performance across a range of graph sizes, so that empirical growth can be compared against theoretical complexity. | Must |
| FR-11.4 | The harness **shall** repeat each measurement a configurable number of times and report the mean and standard deviation, so that results are not distorted by single-run noise. | Must |
| FR-11.5 | Benchmark results **shall** be rendered as charts within the analysis panel and **shall** be exportable. | Must |
| FR-11.6 | The harness **shall** verify on every run that Dijkstra and A* agree on total route cost, providing a continuous correctness check of the heuristic's admissibility. | Must |
| FR-11.7 | The system **should** also quantify the benefit of the fleet assignment by comparing achieved makespan against a single-drone baseline and against a balanced-load assignment. | Should |

---

### 3.3 Performance Requirements

| Ref | Requirement |
|---|---|
| NFR-1 | A single route computation over the demonstration map (≥ 20 vertices) **shall** complete within 50 ms on commodity hardware. |
| NFR-2 | A full planning cycle for 10 deliveries across 3 drones **shall** complete within 1 second. |
| NFR-3 | Any user interface control **shall** produce a visible response within 200 ms, and a complete replan within 2 seconds. |
| NFR-4 | The algorithm tier **shall** handle graphs of up to 1000 vertices without exhausting memory on a machine with 8 GB of RAM. |
| NFR-5 | The benchmarking harness **shall** complete its full sweep within 5 minutes. |

### 3.4 Design Constraints

| Ref | Requirement |
|---|---|
| NFR-6 | All algorithms named in FR-3, FR-6 and FR-7 **shall** be implemented from first principles, per constraint C-1. |
| NFR-7 | The algorithm tier **shall** have no dependency on the web framework, and **shall** be executable and testable in isolation. |
| NFR-8 | The system **shall** run on Windows, macOS and Linux with an unmodified codebase. |
| NFR-9 | Source code **shall** conform to PEP 8 and **shall** carry type annotations on all public interfaces. |

### 3.5 Software System Attributes

#### 3.5.1 Reliability

| Ref | Requirement |
|---|---|
| NFR-10 | The system **shall** handle unreachable destinations, infeasible energy budgets, empty fleets and malformed input without terminating abnormally. |
| NFR-11 | Given identical inputs and configuration, the system **shall** produce identical output; all tie-breaking **shall** be deterministic. |

#### 3.5.2 Availability

| Ref | Requirement |
|---|---|
| NFR-12 | The system **shall** start and be ready to accept input within 5 seconds of launch. |

#### 3.5.3 Security

| Ref | Requirement |
|---|---|
| NFR-13 | The system is a local single-user simulation and requires no authentication. It **shall** bind to the loopback interface by default and **shall not** expose itself on external network interfaces without explicit configuration. |
| NFR-14 | All input read from file or received over HTTP **shall** be validated before use. |

#### 3.5.4 Maintainability

| Ref | Requirement |
|---|---|
| NFR-15 | Each algorithm **shall** reside in its own module with a documented interface. |
| NFR-16 | The automated test suite **shall** achieve at least 80 % statement coverage of the algorithm tier. |
| NFR-17 | Every algorithm module **shall** carry a docstring stating its time and space complexity. |

#### 3.5.5 Portability

| Ref | Requirement |
|---|---|
| NFR-18 | Dependencies **shall** be declared in a single manifest, and installation **shall** require one command. |
| NFR-19 | The system **shall not** rely on platform-specific paths, shell commands or line endings. |

#### 3.5.6 Usability

| Ref | Requirement |
|---|---|
| NFR-20 | A first-time user **shall** be able to load the demonstration scenario and produce a complete plan in no more than three interactions. |
| NFR-21 | Route colours and node markers **shall** remain distinguishable under projection, including for common forms of colour-vision deficiency. |

---

## 4. Appendices

### Appendix A — Traceability to the Project Brief

| Brief section | Subject | Requirements |
|---|---|---|
| §2.1 | Create a simulated city map | FR-1.1 – FR-1.9 |
| §2.2 | Receive delivery requests | FR-2.1 – FR-2.10 |
| §2.3 | Find the best route | FR-3.1 – FR-3.8 |
| §2.4 | Consider battery / energy usage | FR-4.1 – FR-4.8, FR-5.1 – FR-5.7 |
| §2.5 | Prioritize deliveries | FR-6.1 – FR-6.6 |
| §2.6 | Divide work between multiple drones | FR-7.1 – FR-7.12 |
| §2.7 | Show selected route and delivery information | FR-10.1 – FR-10.7, UI-1 – UI-15 |
| §3 | Algorithms and data structures used | FR-1.5, FR-3.1, FR-3.2, FR-6.1, FR-7.2, NFR-6 |
| §4.5 | Reduced operational cost | FR-7.13 – FR-7.15 (en-route consolidation) |
| §4 | Main benefits of the project | FR-7.8, FR-11.7 (quantified evidence for the claimed benefits) |
| — | Extension: fictional city and drawn backdrop | FR-1.10, FR-1.11 |
| — | Extension: operator-composed batches and demonstration plans | FR-2.7 – FR-2.10, UI-13, UI-14 |
| — | Extension: no-fly zones | FR-8.1 – FR-8.6 |
| — | Extension: wind and weather | FR-9.1 – FR-9.7 |
| — | Complexity analysis and measurement | FR-11.1 – FR-11.7, NFR-17 |

### Appendix B — Algorithm and Data Structure Allocation

The brief requires that distinct algorithms solve distinct parts of the problem. This table
records where each is used and which requirement mandates it.

| Data structure / algorithm | Applied to | Requirement |
|---|---|---|
| Graph (weighted, undirected) | City model | FR-1.1 |
| Adjacency list | Storing connectivity in O(V + E) | FR-1.5 |
| Binary min-heap | Search frontier; delivery priority queue | FR-3.1, FR-6.1 |
| Dijkstra's algorithm | Guaranteed minimum-cost route | FR-3.1 |
| A* search | Goal-directed route search | FR-3.2 |
| Haversine heuristic | Admissible lower bound for A* | FR-3.3 |
| Greedy selection | Choosing the next delivery by priority | FR-6.3 |
| Greedy assignment | Choosing the drone minimising completion time | FR-7.2, FR-7.3 |
| Two-phase constrained search | Energy-feasible routing via a charging station | FR-5.1 – FR-5.3 |

### Appendix C — Open Items

| Ref | Item | Status |
|---|---|---|
| O-1 | FR-11.3 requires benchmarking across a range of graph sizes, while the map is specified as a single hand-crafted city (FR-1.8). The resolution adopted is that the application always uses the fixed demonstration map, and the benchmarking harness additionally generates synthetic graphs of varying size purely as test fixtures. These synthetic graphs are never exposed in the user interface. | Assumed; to be confirmed |
| O-2 | Whether the aging policy of FR-6.6 is implemented. | Optional |

---

*End of Software Requirements Specification.*
