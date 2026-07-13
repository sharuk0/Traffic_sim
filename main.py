#!/usr/bin/env python3
"""Punto de entrada del simulador.

Ejemplos:
    python main.py --scenario four_way
    python main.py --scenario roundabout --seed 7 --speed 4
    python main.py --scenario left_turn_only --headless --duration 900 --export output/
"""

from __future__ import annotations

import argparse
import logging
import sys

from traffic_sim import SCENARIOS, Simulation, SimulationConfig, build
from traffic_sim.config import ArrivalProcess, DemandPeriod


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulador microscópico de tráfico urbano (modelo tipo IDM)."
    )
    parser.add_argument("--scenario", default="four_way", choices=sorted(SCENARIOS),
                        help="escenario a simular")
    parser.add_argument("--seed", type=int, default=42,
                        help="semilla aleatoria (misma semilla ⇒ misma ejecución)")
    parser.add_argument("--vph", type=float, default=None,
                        help="demanda total en vehículos/hora (sobrescribe la del escenario)")
    parser.add_argument("--period", choices=[p.value for p in DemandPeriod], default=None,
                        help="hora punta o valle")
    parser.add_argument("--arrival", choices=[a.value for a in ArrivalProcess], default=None,
                        help="proceso de llegadas")
    parser.add_argument("--warmup", type=float, default=30.0,
                        help="segundos simulados excluidos de las métricas")
    parser.add_argument("--probe-route", default=None,
                        help="ruta del vehículo de prueba (por defecto, la del escenario)")
    parser.add_argument("--probe-time", type=float, default=60.0,
                        help="instante de inyección del vehículo de prueba")
    parser.add_argument("--headless", action="store_true",
                        help="ejecuta sin ventana y muestra las métricas al terminar")
    parser.add_argument("--duration", type=float, default=600.0,
                        help="duración simulada en modo headless (s)")
    parser.add_argument("--export", metavar="DIR", default=None,
                        help="exporta las métricas a CSV en el directorio indicado")
    parser.add_argument("--verbose", action="store_true", help="logging detallado")
    return parser.parse_args(argv)


def build_config(args: argparse.Namespace, scenario) -> SimulationConfig:
    demand = scenario.default_demand
    if args.vph is not None:
        demand.vehicles_per_hour = args.vph
    if args.period is not None:
        demand.period = DemandPeriod(args.period)
    if args.arrival is not None:
        demand.arrival = ArrivalProcess(args.arrival)
    return SimulationConfig(
        seed=args.seed,
        warmup=args.warmup,
        demand=demand,
        probe_route=args.probe_route or scenario.default_probe_route,
        probe_time=args.probe_time,
    )


def print_report(sim: Simulation) -> None:
    print(f"\n=== {sim.scenario.name} — {sim.scenario.description} ===")
    for key, value in sim.metrics.summary().items():
        print(f"  {key:<20} {value}")
    by_route = sim.metrics.by_route()
    if by_route:
        print("\n  ruta            n   t.viaje   t.espera   vel")
        for route, stats in by_route.items():
            print(
                f"  {route:<12} {int(stats['count']):>3}   {stats['avg_travel_time']:>7.1f}s "
                f"{stats['avg_stopped_time']:>9.1f}s {stats['avg_speed']:>5.1f}"
            )
    for r in sim.metrics.probe_records:
        print(
            f"\n  Vehículo de prueba #{r.id} · ruta {r.route}"
            f"\n    ingreso {r.entry_time:.1f}s · salida {r.exit_time:.1f}s"
            f"\n    viaje {r.travel_time:.1f}s · detenido {r.stopped_time:.1f}s"
            f"\n    distancia {r.distance:.0f} m · vel. media {r.mean_speed:.1f} m/s"
        )
    if sim.overlap_fixes:
        print(f"\n  AVISO: {sim.overlap_fixes} correcciones de solapamiento (revisar calibración)")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    scenario = build(args.scenario)
    config = build_config(args, scenario)
    sim = Simulation(scenario, config)

    if args.headless:
        sim.run(args.duration)
        print_report(sim)
    else:
        from traffic_sim.rendering import Window

        Window(sim, config, export_dir=args.export or "output").run()
        print_report(sim)

    if args.export:
        paths = sim.metrics.export_csv(args.export, prefix=scenario.name)
        print("\nCSV exportados:")
        for p in paths:
            print(f"  {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
