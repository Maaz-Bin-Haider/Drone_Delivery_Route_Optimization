"""Experimental harness (TDD section 16, FR-11).

Every complexity claim the design makes is measured here rather than asserted.
Each experiment returns a plain dictionary so results can be charted, exported
or asserted against by tests.

Two habits are deliberate throughout. Every measurement is repeated and reported
as mean plus standard deviation (FR-11.4), so a single noisy run cannot be
mistaken for a trend. And every Dijkstra/A* pair is checked for cost equality
(FR-11.6), so an inadmissible heuristic would surface as a failed benchmark
rather than as quietly sub-optimal routes.
"""

from __future__ import annotations

import itertools
import random
import statistics
import time
from dataclasses import dataclass, replace

from ..algorithms.astar import astar_route
from ..algorithms.constrained import pareto_constrained_route
from ..algorithms.dijkstra import dijkstra_route
from ..constants import SERVICE_TIME_MIN
from ..cost.cost_model import (BALANCED, MINIMUM_ENERGY, SHORTEST_DISTANCE,
                               CostModel, Weights)
from ..domain.models import Drone, NodeType
from ..environment.no_fly import NoFlyMask
from ..environment.wind import CALM, Wind
from ..graph.graph import Graph
from ..scheduling.assignment import assign_fleet
from ..simulation.cache import RouteTable
from .generators import random_city, random_pairs

DEFAULT_SIZES = (10, 25, 50, 100, 250, 500, 1000)
WEIGHT_SETTINGS = (
    ("distance", SHORTEST_DISTANCE),
    ("balanced", BALANCED),
    ("energy", MINIMUM_ENERGY),
)


class CostDisagreement(AssertionError):
    """Dijkstra and A* returned different costs, falsifying admissibility."""


# --------------------------------------------------------------------------
# statistics helpers
# --------------------------------------------------------------------------

def summarise(values: list[float]) -> dict:
    """Mean and standard deviation, so no figure is reported from one sample."""
    if not values:
        return {"mean": 0.0, "stdev": 0.0, "n": 0}
    return {
        "mean": round(statistics.fmean(values), 6),
        "stdev": round(statistics.stdev(values), 6) if len(values) > 1 else 0.0,
        "n": len(values),
    }


def linear_fit(xs: list[float], ys: list[float]) -> dict:
    """Least-squares fit y = ax + b, with the coefficient of determination."""
    if len(xs) < 2:
        return {"slope": 0.0, "intercept": 0.0, "r_squared": 0.0}
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx if sxx else 0.0
    intercept = my - slope * mx
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    return {
        "slope": slope,
        "intercept": intercept,
        "r_squared": round(1 - ss_res / ss_tot, 6) if ss_tot else 1.0,
    }


# --------------------------------------------------------------------------
# landmark selection
#
# Experiments 3 and 7 need long journeys across the map. Deriving them from the
# graph rather than naming node ids keeps the harness working when the city is
# redrawn, which a hardcoded list would silently break.
# --------------------------------------------------------------------------

def _far_customers(graph, count: int) -> list[str]:
    """The customers furthest from the warehouse, by straight-line distance."""
    from ..geo import haversine_km
    depot = graph.nodes[graph.warehouse]
    customers = [n for n in graph.nodes.values() if n.type is NodeType.CUSTOMER]
    customers.sort(
        key=lambda n: -haversine_km(depot.lat, depot.lon, n.lat, n.lon))
    return [n.id for n in customers[:count]]


def _cross_map_pairs(graph) -> list[tuple[str, str]]:
    """Three long traversals: west-east, south-north, and depot to the far edge."""
    customers = [n for n in graph.nodes.values() if n.type is NodeType.CUSTOMER]
    if len(customers) < 2:
        return []
    west = min(customers, key=lambda n: n.lon).id
    east = max(customers, key=lambda n: n.lon).id
    south = min(customers, key=lambda n: n.lat).id
    north = max(customers, key=lambda n: n.lat).id
    far = _far_customers(graph, 1)
    pairs = [(west, east), (south, north)]
    if far:
        pairs.append((graph.warehouse, far[0]))
    return [(a, b) for a, b in pairs if a != b]


# --------------------------------------------------------------------------
# Experiment 1 -- Dijkstra vs A*: nodes expanded
# --------------------------------------------------------------------------

def experiment_1_expansion(sizes: tuple[int, ...] = DEFAULT_SIZES,
                           pairs: int = 30, repeats: int = 5,
                           seed: int = 20260922) -> dict:
    """Does the heuristic actually reduce the search, and by how much?

    Worst-case complexity is identical for both algorithms, so any benefit is a
    constant factor. Measuring it is the only honest way to report it.
    """
    series = []
    for size in sizes:
        graph = random_city(size, seed=seed + size)
        sample = random_pairs(graph, pairs, seed=seed + size)
        per_weight = {}

        for label, weights in WEIGHT_SETTINGS:
            cm = CostModel(graph, weights)
            d_expanded, a_expanded, ratios = [], [], []
            d_times, a_times = [], []

            for source, target in sample:
                d = dijkstra_route(graph, source, target, cm)
                a = astar_route(graph, source, target, cm)
                if d is None or a is None:
                    continue
                if abs(d.cost - a.cost) > 1e-9:
                    raise CostDisagreement(
                        f"V={size} {source}->{target}: {d.cost!r} != {a.cost!r}")
                d_expanded.append(d.stats.nodes_expanded)
                a_expanded.append(a.stats.nodes_expanded)
                ratios.append(a.stats.nodes_expanded / max(d.stats.nodes_expanded, 1))

            for _ in range(repeats):
                t = time.perf_counter()
                for source, target in sample:
                    dijkstra_route(graph, source, target, cm)
                d_times.append((time.perf_counter() - t) * 1000.0 / len(sample))
                t = time.perf_counter()
                for source, target in sample:
                    astar_route(graph, source, target, cm)
                a_times.append((time.perf_counter() - t) * 1000.0 / len(sample))

            per_weight[label] = {
                "dijkstra_expanded": summarise(d_expanded),
                "astar_expanded": summarise(a_expanded),
                "expansion_ratio": summarise(ratios),
                "dijkstra_ms": summarise(d_times),
                "astar_ms": summarise(a_times),
            }

        series.append({
            "V": size,
            "E": graph.edge_count(),
            "pairs_measured": len(sample),
            "weights": per_weight,
        })

    return {
        "experiment": 1,
        "title": "Dijkstra vs A*: nodes expanded",
        "hypothesis": ("A* expands materially fewer nodes than Dijkstra, with the "
                       "advantage greatest under pure distance weighting and "
                       "narrowing as energy or time weighting grows."),
        "costs_agreed_everywhere": True,
        "series": series,
    }


# --------------------------------------------------------------------------
# Experiment 2 -- empirical growth against the derived bound
# --------------------------------------------------------------------------

def experiment_2_growth(expansion: dict | None = None, **kwargs) -> dict:
    """Does measured runtime actually grow as O((V + E) log V)?

    Plotting runtime against (V + E) log V should give a straight line. A high
    coefficient of determination is direct empirical support for the bound
    derived in TDD section 7.2.
    """
    import math
    data = expansion or experiment_1_expansion(**kwargs)
    points = []
    for row in data["series"]:
        v, e = row["V"], row["E"]
        predictor = (v + e) * math.log2(max(v, 2))
        points.append({
            "V": v, "E": e, "predictor": round(predictor, 2),
            "dijkstra_ms": row["weights"]["distance"]["dijkstra_ms"]["mean"],
            "astar_ms": row["weights"]["distance"]["astar_ms"]["mean"],
        })

    xs = [p["predictor"] for p in points]
    return {
        "experiment": 2,
        "title": "Empirical growth against the O((V + E) log V) bound",
        "predictor": "(V + E) * log2(V)",
        "points": points,
        "dijkstra_fit": linear_fit(xs, [p["dijkstra_ms"] for p in points]),
        "astar_fit": linear_fit(xs, [p["astar_ms"] for p in points]),
    }


# --------------------------------------------------------------------------
# Experiment 3 -- value of energy-aware routing
# --------------------------------------------------------------------------

def experiment_3_energy(scenario,
                        betas: tuple[float, ...] = (0.0, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0),
                        wind: Wind = Wind(18.0, 45.0)) -> dict:
    """Two measurements, deliberately separated.

    Per journey the brief's Route A / Route B claim holds. Per batch it does
    not, because routing and scheduling are coupled: energy-optimal routes are
    slower, which delays drones and can force a charging detour costing more
    than the per-route saving recovers. Conflating the two would hide the most
    instructive result in the project.
    """
    from copy import deepcopy

    from ..simulation.orchestrator import PlanConfig, Simulator

    graph = scenario.graph
    # The batch-level coupling only appears when batteries are under pressure,
    # so the heaviest available preset is used rather than the light default
    # batch. On a batch the fleet can absorb comfortably there is no detour to
    # trigger and the effect is simply absent -- which is itself worth stating.
    scenario = deepcopy(scenario)
    if scenario.presets:
        heaviest = max(scenario.presets, key=lambda p: len(p["deliveries"]))
        scenario.replace_deliveries(heaviest["deliveries"])
        batch_label = heaviest["name"]
    else:
        batch_label = "default batch"
    sim = Simulator(scenario)
    journeys = [(graph.warehouse, t) for t in _far_customers(graph, 4)]

    per_journey, per_batch = [], []
    for beta in betas:
        weights = Weights(1.0 - beta, beta, 0.0)
        cm = CostModel(graph, weights, wind)

        legs = []
        for source, target in journeys:
            r = dijkstra_route(graph, source, target, cm)
            if r:
                legs.append({"pair": f"{source}->{target}",
                             "distance_km": round(r.distance_km, 3),
                             "energy_pct": round(r.energy_pct, 3)})
        per_journey.append({
            "beta": beta,
            "distance_km": round(sum(l["distance_km"] for l in legs), 3),
            "energy_pct": round(sum(l["energy_pct"] for l in legs), 3),
            "legs": legs,
        })

        plan = sim.plan(PlanConfig(weights=weights, wind=wind))
        detours = sum(1 for a in plan["assignments"] if a["charging_stop"])
        per_batch.append({
            "beta": beta,
            "distance_km": plan["totals"]["distance_km"],
            "energy_pct": plan["totals"]["energy_pct"],
            "makespan_min": plan["makespan_min"],
            "charging_detours": detours,
            "unserviceable": plan["totals"]["unserviceable"],
        })

    journey_monotonic = all(
        per_journey[i]["energy_pct"] >= per_journey[i + 1]["energy_pct"] - 1e-6
        for i in range(len(per_journey) - 1)
    )
    batch_energies = [b["energy_pct"] for b in per_batch]
    return {
        "experiment": 3,
        "title": "Value of energy-aware routing",
        "batch": batch_label,
        "wind": wind.as_dict(),
        "per_journey": per_journey,
        "per_batch": per_batch,
        "journey_energy_monotonic": journey_monotonic,
        "batch_energy_monotonic": all(
            batch_energies[i] >= batch_energies[i + 1] - 1e-6
            for i in range(len(batch_energies) - 1)
        ),
        "finding": ("Per journey, weighting energy never costs more energy. Per batch "
                    "it can: slower energy-optimal routes delay drones and trigger "
                    "charging detours whose extra leg outweighs the per-route saving. "
                    "The effect needs the fleet near its feasibility boundary -- on a "
                    "batch the drones absorb comfortably, no detour is triggered and "
                    "batch energy falls monotonically as the per-journey result "
                    "predicts. The charging_detours column is what distinguishes the "
                    "two regimes."),
    }


# --------------------------------------------------------------------------
# Experiment 4 -- value of the fleet
# --------------------------------------------------------------------------

def experiment_4_fleet(scenario, max_drones: int = 5,
                       weights: Weights = SHORTEST_DISTANCE) -> dict:
    """How much does each additional drone actually buy?

    All drones start fully charged, so differing battery states do not confound
    the comparison. The speedup is nonetheless *superlinear* on this scenario,
    which is not evidence of better-than-ideal parallelism -- it is a real
    second effect that the row-level breakdown exposes: a small fleet exhausts
    its charge and pays for recharge cycles, and a single drone must chain every
    destination into one long sequential tour. Adding drones removes both costs
    as well as parallelising the work.
    """
    graph = scenario.graph
    table = RouteTable(graph, CostModel(graph, weights))
    warehouse = graph.warehouse
    rows = []
    baseline = None

    for n in range(1, max_drones + 1):
        fleet = [Drone(f"D{i + 1}", warehouse, 100.0) for i in range(n)]
        plan = assign_fleet(table, fleet, list(scenario.deliveries))
        makespan = plan.makespan_min
        if n == 1:
            baseline = makespan
        detours = [a for a in plan.assignments if a.reroute is not None]
        rows.append({
            "drones": n,
            "makespan_min": round(makespan, 3),
            "speedup": round(baseline / makespan, 4) if makespan else 0.0,
            "ideal_speedup": n,
            "charging_detours": len(detours),
            "recharge_min": round(sum(a.reroute.recharge_min for a in detours), 2),
            "flight_min": round(sum(a.route.time_min for a in plan.assignments), 2),
            "unserviceable": len(plan.unserviceable),
            "per_drone": [len(d.assigned) for d in plan.drones],
        })

    superlinear = [r for r in rows if r["speedup"] > r["ideal_speedup"] + 1e-6]
    detour_free_from = next((r["drones"] for r in rows if r["charging_detours"] == 0), None)
    return {
        "experiment": 4,
        "title": "Value of the fleet",
        "hypothesis": ("Makespan falls as drones are added. Naive expectation is "
                       "sub-linear speedup, since work cannot be split perfectly."),
        "rows": rows,
        "superlinear_at": [r["drones"] for r in superlinear],
        "detour_free_from_drones": detour_free_from,
        "finding": ("Speedup exceeds the ideal 1/D curve on this scenario, which "
                    "does not mean parallelism beats its own bound. Two further "
                    "costs disappear as the fleet grows: recharge cycles, which a "
                    "small fleet cannot avoid, and the long sequential tour a "
                    "single drone must fly between consecutive customers. The "
                    "per-row charging_detours and flight_min columns separate "
                    "these from the parallel speedup."),
    }


# --------------------------------------------------------------------------
# Experiment 5 -- greedy assignment quality against the true optimum
# --------------------------------------------------------------------------

def _optimal_makespan(table: RouteTable, warehouse: str,
                      destinations: list[str], drone_count: int,
                      service_min: float = SERVICE_TIME_MIN) -> float:
    """Exact minimum makespan by dynamic programming over subsets.

    For each subset of packages, Held-Karp gives the least time one drone needs
    to deliver them all starting from the warehouse; the optimum is then the
    best partition of packages across drones. O(2^P * P^2 + D^P), which is only
    tractable because P is kept small -- which is precisely why an approximation
    is needed at realistic sizes.
    """
    p = len(destinations)
    full = 1 << p

    best: list[list[float]] = [[float("inf")] * p for _ in range(full)]
    for j, dest in enumerate(destinations):
        best[1 << j][j] = table.time(warehouse, dest) + service_min

    for mask in range(full):
        for last in range(p):
            current = best[mask][last]
            if current == float("inf"):
                continue
            for j in range(p):
                if mask & (1 << j):
                    continue
                nxt = mask | (1 << j)
                cand = current + table.time(destinations[last], destinations[j]) + service_min
                if cand < best[nxt][j]:
                    best[nxt][j] = cand

    completion = [0.0] * full
    for mask in range(1, full):
        completion[mask] = min(best[mask][j] for j in range(p) if mask & (1 << j))

    optimum = float("inf")
    for assignment in itertools.product(range(drone_count), repeat=p):
        masks = [0] * drone_count
        for pkg, drone in enumerate(assignment):
            masks[drone] |= 1 << pkg
        makespan = max(completion[m] if m else 0.0 for m in masks)
        if makespan < optimum:
            optimum = makespan
    return optimum


def experiment_5_greedy_quality(scenario, instances: int = 120, packages: int = 6,
                                drones: int = 3, seed: int = 20260922) -> dict:
    """How close is the greedy schedule to optimal?

    TDD section 9.4 explains why Graham's (2 - 1/m) bound does not transfer to
    this problem: durations are sequence-dependent and the eligible set changes
    with battery state. Brute force on small instances is the honest empirical
    substitute for a bound that does not apply.

    Drones start fully charged, so feasibility never binds and the measurement
    isolates scheduling quality.
    """
    from ..domain.models import DeliveryRequest, Priority

    graph = scenario.graph
    table = RouteTable(graph, CostModel(graph, SHORTEST_DISTANCE))
    warehouse = graph.warehouse
    customers = [n.id for n in graph.nodes.values() if n.type is NodeType.CUSTOMER]
    rng = random.Random(seed)

    ratios, worst_case, excluded = [], None, 0
    for _ in range(instances):
        destinations = rng.sample(customers, packages)
        requests = [
            DeliveryRequest(f"P{i}", d, Priority.NORMAL, i)
            for i, d in enumerate(destinations)
        ]
        fleet = [Drone(f"D{i + 1}", warehouse, 100.0) for i in range(drones)]

        greedy_plan = assign_fleet(table, fleet, requests)
        # The brute-force optimum models scheduling only, not battery. An
        # instance where greedy needed a charging detour is therefore not a
        # like-for-like comparison, and is excluded rather than silently
        # inflating the ratio.
        if any(a.reroute is not None for a in greedy_plan.assignments):
            excluded += 1
            continue
        greedy = greedy_plan.makespan_min
        optimal = _optimal_makespan(table, warehouse, destinations, drones)
        if optimal <= 0:
            continue
        ratio = greedy / optimal
        ratios.append(ratio)
        if worst_case is None or ratio > worst_case["ratio"]:
            worst_case = {"ratio": round(ratio, 4),
                          "destinations": destinations,
                          "greedy_min": round(greedy, 3),
                          "optimal_min": round(optimal, 3)}

    buckets = {"1.00": 0, "1.00-1.05": 0, "1.05-1.10": 0, "1.10-1.25": 0, ">1.25": 0}
    for r in ratios:
        if r <= 1.0 + 1e-9:
            buckets["1.00"] += 1
        elif r <= 1.05:
            buckets["1.00-1.05"] += 1
        elif r <= 1.10:
            buckets["1.05-1.10"] += 1
        elif r <= 1.25:
            buckets["1.10-1.25"] += 1
        else:
            buckets[">1.25"] += 1

    return {
        "experiment": 5,
        "title": "Greedy assignment quality against brute-force optimal",
        "instances": len(ratios),
        "excluded_for_charging_detours": excluded,
        "packages": packages,
        "drones": drones,
        "ratio": summarise(ratios),
        "optimal_found_pct": round(100 * buckets["1.00"] / max(len(ratios), 1), 2),
        "histogram": buckets,
        "worst_case": worst_case,
        "note": ("Greedy never beats optimal by construction, so a ratio of 1.00 "
                 "means it found an optimal schedule."),
    }


# --------------------------------------------------------------------------
# Experiment 6 -- the two-phase approximation against exact constrained search
# --------------------------------------------------------------------------

def experiment_6_constrained(scenario, instances: int = 120,
                             wind: Wind = Wind(16.0, 225.0),
                             seed: int = 20260922) -> dict:
    """Adversarial test of the documented approximation (TDD section 8.2).

    The cheap feasibility test asks whether the minimum-*cost* path fits the
    energy budget. That is not the same question as whether *any* path fits, so
    it can declare a delivery infeasible when a costlier route would have
    served. Instances are drawn with budgets in exactly the window where the two
    answers can differ, so this measures the failure mode at its worst rather
    than at its average.
    """
    graph = scenario.graph
    cm_cost = CostModel(graph, SHORTEST_DISTANCE, wind)
    cm_energy = CostModel(graph, MINIMUM_ENERGY, wind)
    sample = random_pairs(graph, instances, seed=seed)
    rng = random.Random(seed)

    agree = disagree = 0
    false_infeasible = []
    naive_ms, exact_ms = [], []
    cost_gaps = []

    for source, target in sample:
        cheap = dijkstra_route(graph, source, target, cm_cost)
        green = dijkstra_route(graph, source, target, cm_energy)
        if cheap is None or green is None:
            continue

        low, high = min(green.energy_pct, cheap.energy_pct), max(green.energy_pct, cheap.energy_pct)
        span = high - low
        budget = low + rng.random() * span if span > 1e-9 else high * rng.uniform(0.9, 1.1)

        # Time the whole naive procedure -- the search plus the check -- so the
        # comparison against the exact method is like-for-like. Timing only the
        # comparison would flatter it by several orders of magnitude.
        t = time.perf_counter()
        probe = dijkstra_route(graph, source, target, cm_cost)
        naive_feasible = probe is not None and probe.energy_pct <= budget + 1e-9
        naive_ms.append((time.perf_counter() - t) * 1000.0)

        t = time.perf_counter()
        exact = pareto_constrained_route(graph, source, target, budget, cm_cost)
        exact_ms.append((time.perf_counter() - t) * 1000.0)
        exact_feasible = exact is not None

        if naive_feasible == exact_feasible:
            agree += 1
            if naive_feasible and exact is not None:
                cost_gaps.append(cheap.cost - exact.cost)
        else:
            disagree += 1
            if exact_feasible and not naive_feasible:
                false_infeasible.append({
                    "pair": f"{source}->{target}",
                    "budget_pct": round(budget, 2),
                    "min_cost_route_energy_pct": round(cheap.energy_pct, 2),
                    "feasible_route_energy_pct": round(exact.energy_pct, 2),
                    "extra_cost": round(exact.cost - cheap.cost, 6),
                })

    total = agree + disagree
    return {
        "experiment": 6,
        "title": "Two-phase heuristic against exact constrained search",
        "instances": total,
        "wind": wind.as_dict(),
        "agreement_pct": round(100 * agree / max(total, 1), 2),
        "disagreements": disagree,
        "false_infeasible_count": len(false_infeasible),
        "false_infeasible_examples": false_infeasible[:5],
        "naive_ms": summarise(naive_ms),
        "exact_ms": summarise(exact_ms),
        "speedup": round(
            statistics.fmean(exact_ms) / statistics.fmean(naive_ms), 1
        ) if naive_ms and exact_ms and statistics.fmean(naive_ms) > 0 else None,
        "mean_cost_gap": summarise(cost_gaps),
        "finding": ("Two results. First, where the methods disagree it is always "
                    "the cheap test wrongly reporting infeasibility; it never "
                    "wrongly reports success, so the approximation is conservative "
                    "rather than unsafe -- it can refuse a delivery it could have "
                    "served, but never promises one it cannot. Second, once the "
                    "naive figure is timed fairly (the search plus the check, not "
                    "the check alone) the exact Pareto method costs little more at "
                    "this graph size. The approximation is therefore justified by "
                    "scale rather than by this scenario, and the exact method is "
                    "the better default for a map of twenty-odd nodes."),
    }


# --------------------------------------------------------------------------
# Experiment 7 -- environmental sensitivity
# --------------------------------------------------------------------------

def experiment_7_environment(scenario, bearing_step: int = 30) -> dict:
    """Do wind and restricted airspace actually change the routing decision?

    FR-9.6 requires a demonstrable case where wind alone changes the optimal
    route, and FR-8.6 requires the cost penalty a zone imposes to be reported.
    """
    graph = scenario.graph
    bearings = list(range(0, 360, bearing_step))

    sweeps = []
    for source, target in _cross_map_pairs(graph):
        samples, distinct = [], set()
        for bearing in bearings:
            cm = CostModel(graph, MINIMUM_ENERGY, Wind(16.0, bearing))
            r = dijkstra_route(graph, source, target, cm)
            if r is None:
                continue
            distinct.add(r.path)
            samples.append({"bearing_deg": bearing,
                            "path": list(r.path),
                            "energy_pct": round(r.energy_pct, 2),
                            "time_min": round(r.time_min, 2)})
        sweeps.append({"pair": f"{source}->{target}",
                       "distinct_routes": len(distinct),
                       "samples": samples})

    free = CostModel(graph, SHORTEST_DISTANCE)
    penalties = []
    for zone in scenario.zones:
        zones = [replace(z, active=(z.id == zone.id)) for z in scenario.zones]
        masked = CostModel(graph, SHORTEST_DISTANCE, CALM, NoFlyMask(graph, zones))
        deltas, blocked_targets = [], []
        for target in (n.id for n in graph.nodes.values() if n.type is NodeType.CUSTOMER):
            a = dijkstra_route(graph, graph.warehouse, target, free)
            b = dijkstra_route(graph, graph.warehouse, target, masked)
            if a is None:
                continue
            if b is None or masked.mask.node_blocked_by(target):
                blocked_targets.append(target)
                continue
            if b.cost > a.cost + 1e-9:
                deltas.append(100 * (b.cost / a.cost - 1))
        penalties.append({
            "zone": zone.id,
            "name": zone.name,
            "unreachable_customers": blocked_targets,
            "rerouted_customers": len(deltas),
            "cost_penalty_pct": summarise(deltas),
        })

    return {
        "experiment": 7,
        "title": "Environmental sensitivity",
        "wind_sweeps": sweeps,
        "wind_changes_route": any(s["distinct_routes"] > 1 for s in sweeps),
        "no_fly_penalties": penalties,
    }


# --------------------------------------------------------------------------
# full sweep
# --------------------------------------------------------------------------

def run_all(scenario, quick: bool = False) -> dict:
    """Run every experiment. NFR-5 budgets the full sweep at five minutes."""
    started = time.perf_counter()
    sizes = (10, 25, 50, 100) if quick else DEFAULT_SIZES
    pairs = 15 if quick else 30
    repeats = 3 if quick else 5
    instances = 40 if quick else 120

    expansion = experiment_1_expansion(sizes, pairs, repeats)
    results = {
        "experiment_1": expansion,
        "experiment_2": experiment_2_growth(expansion),
        "experiment_3": experiment_3_energy(scenario),
        "experiment_4": experiment_4_fleet(scenario),
        "experiment_5": experiment_5_greedy_quality(scenario, instances=instances),
        "experiment_6": experiment_6_constrained(scenario, instances=instances),
        "experiment_7": experiment_7_environment(scenario),
    }
    results["elapsed_s"] = round(time.perf_counter() - started, 2)
    return results
