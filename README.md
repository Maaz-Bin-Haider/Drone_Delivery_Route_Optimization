# Drone Delivery Route Optimization System

**Group 02 — Design & Analysis of Algorithms**

A simulation that plans, prioritizes and assigns unmanned package deliveries across
**Kestrel Bay**, a fictional coastal city of 34 named locations served by a fleet of five
drones. It automates four decisions a human dispatcher would otherwise make by hand: which
delivery goes next, which drone takes it, which route that drone flies, and whether the route
is energy-feasible.

No single algorithm solves the problem. A weighted graph models the city, a priority queue
orders the work, Dijkstra and A* find routes, and greedy strategies pick the drone — each
handling a different part of the whole.

## Documentation

| Document | Contents |
|---|---|
| [Software Requirements Specification](docs/SRS.md) | IEEE 830-1998. What the system must do, as numbered and traceable requirements. |
| [Technical Design Document](docs/TDD.md) | Architecture, cost mathematics, algorithm pseudocode and complexity, experimental results. |

## Running it

```
python run.py serve                            launch the web dashboard
python run.py plan                             plan the batch and print the schedule
python run.py plan --beta 0.6                  weight the objective toward energy
python run.py plan --wind 12 --bearing 250     plan under a westerly wind
python run.py plan --nfz nfz_aerodrome         activate a no-fly zone
python run.py compare DEP L16                  Dijkstra against A* on one pair
python run.py alternatives L06 L21 --wind 16 --bearing 225
python run.py queue                            show the dispatch order
python run.py benchmark                        run all seven experiments
python run.py benchmark --experiment 5         run one experiment
python run.py benchmark --quick                smaller graphs, fewer repetitions
```

## Dashboard

`python run.py serve`, then open <http://127.0.0.1:5000>.

Kestrel Bay is fictional, so there is no base map: its bay, river, parks and districts are
drawn from local data, which leaves the application with **no network dependency at run
time**. Each drone gets one colour *and* one dash pattern, so routes stay distinguishable
under colour-vision deficiency and on a projector.

**Composing a batch.** Three prepared demonstration plans are one click away:

| Plan | What it shows |
|---|---|
| Morning Round | The baseline. Ten routine parcels; all five drones share the work and nothing needs recharging. |
| Medical Emergency | Four urgent consignments queued behind six routine parcels — the priority queue overtaking work that arrived first. |
| Peak Load | Eighteen parcels to the far corners. Batteries run down and drones start routing through charging pads. |

You can also clear the batch and build one by hand, choosing each destination by name and
district and setting its priority, or remove individual orders.

Weight, wind and airspace controls replan live. The plan table gives every delivery its
route, distance, energy, arrival time and the reason that drone was chosen over the others.
Playback flies the fleet on a shared clock: each drone is drawn as a quadcopter turned to its
heading, rotors spinning only while it is actually airborne.

## Experiments

The harness measures every complexity claim the design makes rather than asserting it.
Headline results, reproducible with `python run.py benchmark`:

| # | Question | Result |
|---|---|---|
| 1 | Does A* actually search less than Dijkstra? | Yes, and increasingly so with scale: 52% of Dijkstra's expansions at V=10, 21.7% at V=1000. |
| 2 | Does runtime match the derived `O((V+E) log V)`? | R² = 0.992 (Dijkstra), 0.991 (A*). |
| 3 | Does energy-aware routing save energy? | Per journey yes; per batch **no** when the fleet is near its battery limit — slower routes trigger charging detours. |
| 4 | What does each extra drone buy? | Superlinear speedup — and makespan is **not monotone**: three drones are slower than two. |
| 5 | How good is the greedy schedule? | 23% above optimal on average, 66% worst case. |
| 6 | Is the fast feasibility test safe? | Conservative, never unsafe — but exact search costs only 1.2× more at this scale. |
| 7 | Do wind and no-fly zones change decisions? | 381 of 561 node pairs are wind-sensitive. |

Experiments 3, 4 and 6 each contradicted their own starting hypothesis, and the fleet sweep
reproduces **Graham's timing anomaly** — adding a drone can lengthen the schedule. A control
run with identical charges restores monotonicity, identifying battery heterogeneity as the
cause. All of it is reported in [TDD §16](docs/TDD.md) and §9.4 as observed, not corrected
away.

## Tests

```
python -m pytest tests -q
python -m pytest tests -q --cov=ddros
```

176 tests, 93% statement coverage.

## Layout

```
ddros/
  web/              Flask API and the Leaflet dashboard
  geo.py            haversine, bearing, projection, polygon geometry
  graph/            adjacency-list city graph and its validation
  structures/       hand-written binary min-heap
  cost/             composite distance/energy/time cost model
  environment/      wind field and no-fly zones
  algorithms/       Dijkstra, A*, heuristics, energy-constrained routing
  scheduling/       delivery priority queue, greedy fleet assignment
  simulation/       planning orchestrator, all-pairs route cache
  analysis/         search instrumentation and benchmarking
data/               city map and backdrop, fleet, batch, demo plans, no-fly zones
tests/              unit, property, integration, contract and architectural tests
```

The algorithm tier imports nothing from the web tier, so it can be tested and benchmarked
without starting a server. A test enforces this rather than trusting it.

## Design notes

Dijkstra, A*, the min-heap and the greedy strategies are implemented from first principles
rather than imported, since they are the subject of the project. `heapq` appears only in the
test suite, as an oracle to cross-check the hand-written heap.

The A* heuristic is proved admissible and consistent in TDD sections 7.3.2 and 7.3.3, and both
properties are asserted by tests across every weight and wind combination. Every planning
cycle re-checks that A* and Dijkstra agree on cost, so an inadmissible heuristic would fail
loudly rather than silently return sub-optimal routes.
