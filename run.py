#!/usr/bin/env python3
"""Command-line driver for the Drone Delivery Route Optimization System.

    python run.py plan                        plan the batch and print the schedule
    python run.py plan --beta 0.6             weight the objective toward energy
    python run.py plan --wind 9 --bearing 270 plan under a westerly wind
    python run.py plan --nfz nfz_airport      activate a no-fly zone
    python run.py compare W C8                Dijkstra against A* on one pair
    python run.py alternatives W C8           shortest route vs most efficient
    python run.py queue                       show the dispatch order
    python run.py benchmark --quick           run the experimental sweep
    python run.py benchmark --experiment 5    run one experiment
    python run.py serve                       launch the web dashboard
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ddros.analysis import benchmark as bench
from ddros.cost.cost_model import Weights
from ddros.domain.loader import load_scenario
from ddros.environment.wind import Wind
from ddros.scheduling.priority_queue import DeliveryQueue
from ddros.simulation.orchestrator import PlanConfig, Simulator

DATA = Path(__file__).resolve().parent / "data"


def build_config(args) -> PlanConfig:
    alpha, beta, gamma = args.alpha, args.beta, args.gamma
    total = alpha + beta + gamma
    if total <= 0:
        raise SystemExit("weights must not all be zero")
    weights = Weights(alpha / total, beta / total, gamma / total)
    return PlanConfig(
        weights=weights,
        wind=Wind(args.wind, args.bearing),
        active_zones=tuple(args.nfz),
        algorithm=args.algorithm,
        fleet_size=getattr(args, "drones", None),
    )


def _rule(width: int = 100) -> str:
    return "-" * width


def cmd_plan(args) -> None:
    sim = Simulator(load_scenario(DATA))
    result = sim.plan(build_config(args))

    if args.json:
        print(json.dumps(result, indent=2))
        return

    cfg = result["config"]
    w = cfg["weights"]
    print(f"\nScenario: {result['scenario']}    algorithm: {cfg['algorithm']}")
    print(f"Weights: distance {w['alpha']:.2f}  energy {w['beta']:.2f}  time {w['gamma']:.2f}"
          f"    wind: {cfg['wind']['speed_ms']:.1f} m/s toward {cfg['wind']['bearing_deg']:.0f} deg"
          f"    no-fly: {cfg['active_no_fly_zones'] or 'none'}")
    print(_rule())
    print(f"{'PKG':<9}{'PRI':<8}{'DRONE':<7}{'ROUTE':<34}"
          f"{'DIST':>7}{'ENERGY':>8}{'ARRIVE':>8}  ALGO")
    print(_rule())
    for a in result["assignments"]:
        path = " ".join(a["route"]["path"])
        if len(path) > 32:
            path = path[:29] + "..."
        print(f"{a['delivery_id']:<9}{a['priority']:<8}{a['drone_id']:<7}{path:<34}"
              f"{a['route']['distance_km']:>6.1f}k{a['route']['energy_pct']:>7.1f}%"
              f"{a['arrive_min']:>7.1f}m  {a['route']['algorithm']}")
    print(_rule())

    if result["unserviceable"]:
        print("\nUNSERVICEABLE")
        for u in result["unserviceable"]:
            print(f"  {u['delivery_id']} -> {u['destination']}: {u['reason']}")

    f = result["fleet"]
    print(f"\nFLEET  {f['chosen']} of {f['chosen'] + len(f['reserve'])} dispatched"
          + (f"  (reserve: {' '.join(f['reserve'])})" if f["reserve"] else ""))
    print(f"  {f['reason']}")
    if len(f["options"]) > 1:
        print("  " + "  ".join(
            f"{o['drones']}:{o['makespan_min']:.0f}m"
            + ("*" if o["unserviceable"] else "")
            for o in f["options"]) + "    (* leaves orders undelivered)")
    for d in result["drones"]:
        print(f"  {d['id']}  {d['deliveries']} deliveries  "
              f"{d['distance_km']:>5.1f} km  battery {d['battery_pct']:>5.1f}%  "
              f"busy {d['busy_min']:>5.1f} min  idle {d['idle_min']:>5.1f} min  "
              f"{' '.join(d['packages'])}")

    t = result["totals"]
    print(f"\nMakespan {result['makespan_min']:.1f} min   "
          f"total {t['distance_km']:.1f} km / {t['energy_pct']:.1f}% energy   "
          f"delivered {t['delivered']}  unserviceable {t['unserviceable']}")
    s = result["search"]
    print(f"All-pairs: {s['all_pairs_searches']} searches, "
          f"{s['all_pairs_nodes_expanded']} nodes expanded\n")


def cmd_compare(args) -> None:
    sim = Simulator(load_scenario(DATA))
    out = sim.compare(args.source, args.target, build_config(args))
    if out.get("unreachable"):
        print(f"{args.source} -> {args.target}: unreachable")
        return
    print(f"\n{args.source} -> {args.target}")
    print(_rule(72))
    for name in ("dijkstra", "astar"):
        r = out[name]
        print(f"{name:<10}{' '.join(r['path']):<34}"
              f"cost {r['cost']:.5f}  expanded {r['stats']['nodes_expanded']:>3}"
              f"  {r['stats']['runtime_ms']:>6.3f} ms")
    print(_rule(72))
    print(f"costs agree: {out['agree']}    "
          f"A* expanded {out['expansion_ratio'] * 100:.0f}% of Dijkstra's nodes\n")


def cmd_alternatives(args) -> None:
    sim = Simulator(load_scenario(DATA))
    out = sim.alternatives(args.source, args.target, build_config(args))
    print(f"\n{args.source} -> {args.target}")
    print(_rule(78))
    for label, key in (("shortest", "min_distance"), ("efficient", "min_energy")):
        r = out[key]
        if r is None:
            print(f"{label:<11}unreachable")
            continue
        print(f"{label:<11}{' '.join(r['path']):<34}"
              f"{r['distance_km']:>6.2f} km{r['energy_pct']:>8.2f}%{r['time_min']:>8.1f} min")
    print(_rule(78))
    print(f"{out.get('annotation', '')}\n")


def cmd_queue(args) -> None:
    scenario = load_scenario(DATA)
    print("\nDispatch order (priority, then arrival)")
    print(_rule(46))
    for i, p in enumerate(DeliveryQueue(scenario.deliveries).drain(), 1):
        print(f"{i:>3}. {p.id:<10}{p.priority.name:<8}-> {p.destination}")
    print()


def cmd_benchmark(args) -> None:
    scenario = load_scenario(DATA)
    if args.experiment:
        runner = {
            1: lambda: bench.experiment_1_expansion(
                sizes=(10, 25, 50, 100) if args.quick else bench.DEFAULT_SIZES),
            2: lambda: bench.experiment_2_growth(
                sizes=(10, 25, 50, 100) if args.quick else bench.DEFAULT_SIZES),
            3: lambda: bench.experiment_3_energy(scenario),
            4: lambda: bench.experiment_4_fleet(scenario),
            5: lambda: bench.experiment_5_greedy_quality(scenario),
            6: lambda: bench.experiment_6_constrained(scenario),
            7: lambda: bench.experiment_7_environment(scenario),
        }[args.experiment]
        results = {f"experiment_{args.experiment}": runner()}
    else:
        results = bench.run_all(scenario, quick=args.quick)

    if args.json:
        print(json.dumps(results, indent=2))
        return
    report_benchmark(results)


def report_benchmark(r: dict) -> None:
    e1 = r.get("experiment_1")
    if e1:
        print("\n[1] DIJKSTRA vs A*  -- nodes expanded, distance weighting")
        print(_rule(78))
        print(f"{'V':>6}{'E':>7}{'Dijkstra':>12}{'A*':>12}{'ratio':>9}"
              f"{'Dij ms':>10}{'A* ms':>10}")
        print(_rule(78))
        for row in e1["series"]:
            w = row["weights"]["distance"]
            print(f"{row['V']:>6}{row['E']:>7}"
                  f"{w['dijkstra_expanded']['mean']:>12.1f}"
                  f"{w['astar_expanded']['mean']:>12.1f}"
                  f"{w['expansion_ratio']['mean']:>8.1%}"
                  f"{w['dijkstra_ms']['mean']:>10.3f}{w['astar_ms']['mean']:>10.3f}")
        print(_rule(78))
        print("    expansion ratio by objective (lower is a better heuristic):")
        for label, _ in bench.WEIGHT_SETTINGS:
            vals = [row["weights"][label]["expansion_ratio"]["mean"] for row in e1["series"]]
            print(f"      {label:<10}{sum(vals)/len(vals):>7.1%}")
        print(f"    Dijkstra and A* agreed on cost for every pair: "
              f"{e1['costs_agreed_everywhere']}")

    e2 = r.get("experiment_2")
    if e2:
        print(f"\n[2] GROWTH vs {e2['predictor']}")
        print(f"    Dijkstra fit R^2 = {e2['dijkstra_fit']['r_squared']:.4f}"
              f"    A* fit R^2 = {e2['astar_fit']['r_squared']:.4f}")

    e3 = r.get("experiment_3")
    if e3:
        print("\n[3] ENERGY-AWARE ROUTING")
        print(_rule(78))
        print(f"{'beta':>6}{'journey km':>13}{'journey %':>12}"
              f"{'batch km':>11}{'batch %':>10}{'detours':>9}")
        print(_rule(78))
        for j, b_ in zip(e3["per_journey"], e3["per_batch"]):
            print(f"{j['beta']:>6.1f}{j['distance_km']:>13.2f}{j['energy_pct']:>12.2f}"
                  f"{b_['distance_km']:>11.2f}{b_['energy_pct']:>10.2f}"
                  f"{b_['charging_detours']:>9}")
        print(_rule(78))
        print(f"    per-journey energy monotonically falls: {e3['journey_energy_monotonic']}")
        print(f"    per-batch energy monotonically falls  : {e3['batch_energy_monotonic']}")

    e4 = r.get("experiment_4")
    if e4:
        print("\n[4] VALUE OF THE FLEET")
        print(_rule(72))
        print(f"{'drones':>8}{'makespan':>12}{'speedup':>10}{'ideal':>8}"
              f"{'detours':>10}{'recharge':>11}{'flight':>10}")
        print(_rule(72))
        for row in e4["rows"]:
            print(f"{row['drones']:>8}{row['makespan_min']:>11.1f}m"
                  f"{row['speedup']:>10.2f}{row['ideal_speedup']:>8}"
                  f"{row['charging_detours']:>10}{row['recharge_min']:>10.1f}m"
                  f"{row['flight_min']:>9.1f}m")
        print(_rule(72))
        if e4["superlinear_at"]:
            print(f"    superlinear at {e4['superlinear_at']} drones -- see detours "
                  f"and flight columns; recharge cost vanishes from "
                  f"{e4['detour_free_from_drones']} drones onward")

    e5 = r.get("experiment_5")
    if e5:
        print("\n[5] GREEDY ASSIGNMENT vs BRUTE-FORCE OPTIMAL")
        print(f"    {e5['instances']} instances, {e5['packages']} packages, "
              f"{e5['drones']} drones")
        print(f"    ratio greedy/optimal: mean {e5['ratio']['mean']:.4f}  "
              f"sd {e5['ratio']['stdev']:.4f}  worst "
              f"{e5['worst_case']['ratio'] if e5['worst_case'] else 0:.4f}")
        print(f"    optimal schedule found in {e5['optimal_found_pct']:.1f}% of instances")
        print(f"    excluded for charging detours: "
              f"{e5['excluded_for_charging_detours']} (not like-for-like)")
        print(f"    histogram: {e5['histogram']}")

    e6 = r.get("experiment_6")
    if e6:
        print("\n[6] TWO-PHASE HEURISTIC vs EXACT CONSTRAINED SEARCH")
        print(f"    {e6['instances']} adversarial instances")
        print(f"    agreement {e6['agreement_pct']:.1f}%   "
              f"wrongly called infeasible: {e6['false_infeasible_count']}")
        print(f"    exact search is {e6['speedup']}x slower than the cheap test"
              if e6["speedup"] else "")
        for ex in e6["false_infeasible_examples"][:3]:
            print(f"      {ex['pair']}: budget {ex['budget_pct']}%, "
                  f"min-cost route needs {ex['min_cost_route_energy_pct']}%, "
                  f"a feasible route needs {ex['feasible_route_energy_pct']}%")

    e7 = r.get("experiment_7")
    if e7:
        print("\n[7] ENVIRONMENTAL SENSITIVITY")
        for s_ in e7["wind_sweeps"]:
            print(f"    {s_['pair']:<14}{s_['distinct_routes']} distinct optimal "
                  f"routes across wind bearings")
        for p_ in e7["no_fly_penalties"]:
            print(f"    {p_['name']:<32}{p_['rerouted_customers']} customers rerouted, "
                  f"mean penalty {p_['cost_penalty_pct']['mean']:.1f}%, "
                  f"{len(p_['unreachable_customers'])} cut off")

    if "elapsed_s" in r:
        print(f"\n    full sweep completed in {r['elapsed_s']}s\n")
    else:
        print()


def cmd_serve(args) -> None:
    from ddros.web.app import create_app
    app = create_app(DATA)
    print(f"\n  Dashboard running at http://{args.host}:{args.port}\n"
          f"  Press Ctrl-C to stop.\n")
    app.run(host=args.host, port=args.port, debug=args.debug)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Drone Delivery Route Optimization System",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    def shared(p):
        p.add_argument("--alpha", type=float, default=1.0, help="distance weight")
        p.add_argument("--beta", type=float, default=0.0, help="energy weight")
        p.add_argument("--gamma", type=float, default=0.0, help="time weight")
        p.add_argument("--wind", type=float, default=0.0, help="wind speed, m/s")
        p.add_argument("--bearing", type=float, default=0.0,
                       help="direction the wind blows toward, degrees")
        p.add_argument("--nfz", action="append", default=[],
                       help="activate a no-fly zone by id (repeatable)")
        p.add_argument("--algorithm", choices=("astar", "dijkstra"), default="astar")
        p.add_argument("--drones", type=int, default=None,
                       help="force a fleet size; omit to choose it automatically")
        return p

    p_plan = shared(sub.add_parser("plan", help="plan the delivery batch"))
    p_plan.add_argument("--json", action="store_true", help="emit raw JSON")
    p_plan.set_defaults(func=cmd_plan)

    p_cmp = shared(sub.add_parser("compare", help="Dijkstra against A*"))
    p_cmp.add_argument("source"); p_cmp.add_argument("target")
    p_cmp.set_defaults(func=cmd_compare)

    p_alt = shared(sub.add_parser("alternatives", help="shortest vs most efficient"))
    p_alt.add_argument("source"); p_alt.add_argument("target")
    p_alt.set_defaults(func=cmd_alternatives)

    sub.add_parser("queue", help="show dispatch order").set_defaults(func=cmd_queue)

    p_bench = sub.add_parser("benchmark", help="run the experimental sweep")
    p_bench.add_argument("--experiment", type=int, choices=range(1, 8),
                         help="run a single experiment")
    p_bench.add_argument("--quick", action="store_true",
                         help="smaller graphs and fewer repetitions")
    p_bench.add_argument("--json", action="store_true", help="emit raw JSON")
    p_bench.set_defaults(func=cmd_benchmark)

    p_serve = sub.add_parser("serve", help="launch the web dashboard")
    p_serve.add_argument("--host", default="127.0.0.1",
                         help="bind address; loopback by default")
    p_serve.add_argument("--port", type=int, default=5000)
    p_serve.add_argument("--debug", action="store_true")
    p_serve.set_defaults(func=cmd_serve)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
