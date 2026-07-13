"""Recolección de métricas. Todo el estado es de INSTANCIA (no hay contadores de clase).

Las métricas ignoran el periodo de calentamiento (`warmup`): sólo se contabilizan los
vehículos cuyo `entry_time` es posterior al calentamiento, de modo que los resultados
describen el régimen estacionario y no el llenado inicial de la red.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

from .vehicle import Vehicle


@dataclass
class VehicleRecord:
    """Registro inmutable de un vehículo que completó su ruta."""

    id: int
    vtype: str
    route: str
    origin: str
    destination: str
    movement: str
    entry_time: float
    exit_time: float
    travel_time: float
    stopped_time: float
    distance: float
    mean_speed: float
    hard_brakes: int
    is_probe: bool

    @classmethod
    def from_vehicle(cls, v: Vehicle) -> VehicleRecord:
        assert v.entry_time is not None and v.exit_time is not None
        return cls(
            id=v.id,
            vtype=v.vtype.name,
            route=v.route.id,
            origin=v.route.origin,
            destination=v.route.destination,
            movement=v.route.movement.value,
            entry_time=round(v.entry_time, 3),
            exit_time=round(v.exit_time, 3),
            travel_time=round(v.exit_time - v.entry_time, 3),
            stopped_time=round(v.stopped_time, 3),
            distance=round(v.distance_travelled, 2),
            mean_speed=round(v.mean_speed, 3),
            hard_brakes=v.hard_brakes,
            is_probe=v.is_probe,
        )


@dataclass
class Metrics:
    """Acumulador de métricas de una única `Simulation`."""

    warmup: float = 0.0

    generated: int = 0
    """Vehículos creados por el generador (incluidos los que aún esperan en cola)."""
    entered: int = 0
    """Vehículos que ya se insertaron físicamente en la red."""
    finished: int = 0
    """Vehículos que llegaron al final de su ruta (incluye los del calentamiento)."""
    dropped: int = 0
    """Vehículos rechazados porque la cola de espera de su acceso estaba llena."""
    completed: int = 0
    """Vehículos finalizados que SÍ cuentan para las métricas (tras el calentamiento)."""
    present: int = 0
    queued: int = 0

    records: list[VehicleRecord] = field(default_factory=list)
    probe_records: list[VehicleRecord] = field(default_factory=list)

    max_queue_length: int = 0
    _queue_samples: list[int] = field(default_factory=list, repr=False)
    _speed_samples: list[float] = field(default_factory=list, repr=False)
    hard_brakes: int = 0
    sim_time: float = 0.0

    # -- registro ------------------------------------------------------------

    def on_generate(self) -> None:
        self.generated += 1

    def on_enter(self) -> None:
        self.entered += 1

    def on_drop(self) -> None:
        self.dropped += 1

    def on_complete(self, vehicle: Vehicle, t: float) -> None:
        vehicle.exit_time = t
        self.finished += 1
        if vehicle.entry_time is None or vehicle.entry_time < self.warmup:
            return
        record = VehicleRecord.from_vehicle(vehicle)
        self.records.append(record)
        self.completed += 1
        if vehicle.is_probe:
            self.probe_records.append(record)

    def sample(self, t: float, vehicles: list[Vehicle], queue_length: int, queued: int) -> None:
        """Muestrea el estado agregado de la red en el paso actual."""
        self.sim_time = t
        self.present = len(vehicles)
        self.queued = queued
        if t < self.warmup:
            return
        self._queue_samples.append(queue_length)
        self.max_queue_length = max(self.max_queue_length, queue_length)
        for v in vehicles:
            self._speed_samples.append(v.v)

    # -- agregados -----------------------------------------------------------

    @property
    def elapsed(self) -> float:
        return max(self.sim_time - self.warmup, 1e-9)

    @property
    def throughput_vph(self) -> float:
        """Vehículos completados por hora (tras el calentamiento)."""
        return self.completed / self.elapsed * 3600.0

    @property
    def avg_travel_time(self) -> float:
        return _mean([r.travel_time for r in self.records])

    @property
    def avg_wait_time(self) -> float:
        return _mean([r.stopped_time for r in self.records])

    @property
    def avg_speed(self) -> float:
        return _mean(self._speed_samples)

    @property
    def avg_queue_length(self) -> float:
        return _mean([float(q) for q in self._queue_samples])

    def by_route(self) -> dict[str, dict[str, float]]:
        """Tiempo medio de viaje / espera y conteo, desagregados por ruta."""
        groups: dict[str, list[VehicleRecord]] = {}
        for r in self.records:
            groups.setdefault(r.route, []).append(r)
        return {
            route: {
                "count": float(len(rs)),
                "avg_travel_time": _mean([r.travel_time for r in rs]),
                "avg_stopped_time": _mean([r.stopped_time for r in rs]),
                "avg_speed": _mean([r.mean_speed for r in rs]),
            }
            for route, rs in sorted(groups.items())
        }

    def summary(self) -> dict[str, float | int]:
        return {
            "sim_time_s": round(self.sim_time, 2),
            "generated": self.generated,
            "entered": self.entered,
            "dropped": self.dropped,
            "completed": self.completed,
            "present": self.present,
            "queued": self.queued,
            "throughput_vph": round(self.throughput_vph, 1),
            "avg_travel_time_s": round(self.avg_travel_time, 2),
            "avg_wait_time_s": round(self.avg_wait_time, 2),
            "avg_speed_ms": round(self.avg_speed, 2),
            "avg_queue_len": round(self.avg_queue_length, 2),
            "max_queue_len": self.max_queue_length,
            "hard_brakes": self.hard_brakes,
        }

    # -- exportación ---------------------------------------------------------

    def export_csv(self, directory: str | Path, prefix: str = "run") -> list[Path]:
        """Escribe tres CSV: por vehículo, por ruta y resumen. Devuelve las rutas creadas."""
        out = Path(directory)
        out.mkdir(parents=True, exist_ok=True)
        paths: list[Path] = []

        vehicles_path = out / f"{prefix}_vehicles.csv"
        fields = list(VehicleRecord.__dataclass_fields__)
        with vehicles_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=fields)
            writer.writeheader()
            for r in self.records:
                writer.writerow({f: getattr(r, f) for f in fields})
        paths.append(vehicles_path)

        routes_path = out / f"{prefix}_routes.csv"
        with routes_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["route", "count", "avg_travel_time", "avg_stopped_time", "avg_speed"])
            for route, stats in self.by_route().items():
                writer.writerow(
                    [
                        route,
                        int(stats["count"]),
                        round(stats["avg_travel_time"], 3),
                        round(stats["avg_stopped_time"], 3),
                        round(stats["avg_speed"], 3),
                    ]
                )
        paths.append(routes_path)

        summary_path = out / f"{prefix}_summary.csv"
        with summary_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["metric", "value"])
            for key, value in self.summary().items():
                writer.writerow([key, value])
        paths.append(summary_path)

        return paths


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0
