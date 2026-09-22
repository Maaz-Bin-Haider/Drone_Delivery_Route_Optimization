# Drone Delivery Route Optimization System

**Group 02 — Design & Analysis of Algorithms**

A simulation that plans, prioritizes and assigns unmanned package deliveries across a
modelled city. It automates four decisions a human dispatcher would otherwise make by hand:
which delivery goes next, which drone takes it, which route that drone flies, and whether the
route is energy-feasible.

No single algorithm solves the problem. A weighted graph models the city, a priority queue
orders the work, Dijkstra and A* find routes, and greedy strategies pick the drone — each
handling a different part of the whole.

## Documentation

| Document | Contents |
|---|---|
| [Software Requirements Specification](docs/SRS.md) | IEEE 830-1998. What the system must do, as numbered and traceable requirements. |
| [Technical Design Document](docs/TDD.md) | Architecture, cost mathematics, algorithm pseudocode and complexity, experimental plan. |

## Running it

```
python run.py serve                           launch the web dashboard
python run.py plan                            plan the batch and print the schedule
python run.py plan --beta 0.6                 weight the objective toward energy
python run.py plan --wind 12 --bearing 250    plan under a westerly wind
python run.py plan --nfz nfz_airport          activate a no-fly zone
python run.py compare W C8                    Dijkstra against A* on one pair
python run.py alternatives C3 C4 --wind 16 --bearing 225
python run.py queue                           show the dispatch order
```

## Tests

```
python -m pytest tests -q
python -m pytest tests -q --cov=ddros
```

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
data/               city map, fleet, delivery batch, no-fly zones
tests/              unit, property, integration and architectural tests
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
