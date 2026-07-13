#!/usr/bin/env python3
"""Genera casos de prueba REPRODUCIBLES a partir de la simulación.

Sustituye al `test_cases.csv` heredado, cuya procedencia no pudo verificarse (ver README,
sección "Sobre test_cases.csv"). Cada fila de la salida es una ejecución determinista
completamente descrita por (escenario, periodo, demanda, semilla): volver a ejecutar este
script con los mismos argumentos produce exactamente el mismo CSV.

Uso:
    python scripts/generate_test_cases.py --out output/test_cases.csv --seeds 20
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from traffic_sim import Simulation, SimulationConfig, build
from traffic_sim.config import DemandConfig, DemandPeriod

SCENARIOS = ("four_way", "roundabout", "left_turn_only", "right_turn_only", "t_junction")
FIELDS = [
    "scenario", "period", "demand_vph", "seed", "duration_s",
    "probe_route", "probe_entry_s", "probe_exit_s", "probe_travel_time_s",
    "probe_stopped_time_s", "probe_distance_m", "probe_mean_speed_ms",
    "throughput_vph", "avg_travel_time_s", "avg_wait_time_s", "avg_speed_ms",
    "max_queue_len", "hard_brakes",
]


def run_case(scenario_name: str, period: DemandPeriod, vph: float, seed: int,
             duration: float, warmup: float) -> dict[str, object] | None:
    scenario = build(scenario_name)
    config = SimulationConfig(
        seed=seed,
        warmup=warmup,
        demand=DemandConfig(vehicles_per_hour=vph, period=period),
        probe_route=scenario.default_probe_route,
        probe_time=warmup + 30.0,
    )
    sim = Simulation(scenario, config)
    sim.run(duration)

    if not sim.metrics.probe_records:
        return None  # el vehículo de prueba no terminó su ruta dentro del horizonte
    probe = sim.metrics.probe_records[-1]
    m = sim.metrics.summary()
    return {
        "scenario": scenario_name,
        "period": period.value,
        "demand_vph": vph,
        "seed": seed,
        "duration_s": duration,
        "probe_route": probe.route,
        "probe_entry_s": probe.entry_time,
        "probe_exit_s": probe.exit_time,
        "probe_travel_time_s": probe.travel_time,
        "probe_stopped_time_s": probe.stopped_time,
        "probe_distance_m": probe.distance,
        "probe_mean_speed_ms": probe.mean_speed,
        "throughput_vph": m["throughput_vph"],
        "avg_travel_time_s": m["avg_travel_time_s"],
        "avg_wait_time_s": m["avg_wait_time_s"],
        "avg_speed_ms": m["avg_speed_ms"],
        "max_queue_len": m["max_queue_len"],
        "hard_brakes": m["hard_brakes"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="output/test_cases.csv")
    parser.add_argument("--seeds", type=int, default=10)
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--warmup", type=float, default=60.0)
    parser.add_argument("--vph", type=float, nargs="+", default=[600.0, 900.0, 1200.0])
    args = parser.parse_args()

    rows: list[dict[str, object]] = []
    skipped = 0
    for scenario in SCENARIOS:
        for period in (DemandPeriod.OFF_PEAK, DemandPeriod.PEAK):
            for vph in args.vph:
                for seed in range(1, args.seeds + 1):
                    row = run_case(scenario, period, vph, seed, args.duration, args.warmup)
                    if row is None:
                        skipped += 1
                        continue
                    rows.append(row)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"{len(rows)} casos escritos en {out} ({skipped} descartados: el vehículo "
          f"de prueba no completó la ruta dentro del horizonte)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
