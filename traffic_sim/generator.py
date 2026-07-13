"""Generación de demanda con cola de espera y aleatoriedad reproducible.

Corrige el fallo del generador original: allí, si el carril de entrada estaba ocupado,
el vehículo pendiente **se descartaba y se sorteaba otro**, lo que perdía demanda en
silencio y sesgaba la distribución de rutas hacia los accesos menos congestionados.

Aquí cada vehículo generado entra en una **cola de espera por carril de acceso** y se
inserta en cuanto hay hueco físico. La demanda efectiva se conserva y la mezcla de rutas
es la configurada. Sólo se descarta un vehículo si la cola supera `max_queue` (lo que se
contabiliza como `dropped`, no se oculta).

Todo el azar proviene de un único `random.Random(seed)` → misma semilla, misma ejecución.
"""

from __future__ import annotations

import random
from collections import deque

from .config import ArrivalProcess, DemandConfig, VehicleType
from .network import Network, Route
from .vehicle import Vehicle


class VehicleGenerator:
    """Genera vehículos según un proceso de llegadas y los encola por carril de acceso."""

    def __init__(self, network: Network, demand: DemandConfig, seed: int) -> None:
        self.network = network
        self.demand = demand
        self.rng = random.Random(seed)
        self._next_id = 1
        self._time_to_next = 0.0
        self.queues: dict[str, deque[Vehicle]] = {}

        routes = list(network.routes.values())
        weights = demand.route_weights
        if weights is None:
            self._routes = routes
            self._weights = [1.0] * len(routes)
        else:
            unknown = set(weights) - set(network.routes)
            if unknown:
                raise ValueError(f"route_weights referencia rutas inexistentes: {sorted(unknown)}")
            positive = {rid: w for rid, w in weights.items() if w > 0}
            if not positive:
                raise ValueError("route_weights no contiene ningún peso positivo")
            self._routes = [network.route(rid) for rid in positive]
            self._weights = list(positive.values())

        self._fleet_types = [t for t, _ in demand.fleet]
        self._fleet_weights = [w for _, w in demand.fleet]
        if not self._fleet_types or sum(self._fleet_weights) <= 0:
            raise ValueError("La flota debe tener al menos un tipo con peso positivo")

        self._schedule_next()

    # -- API -----------------------------------------------------------------

    @property
    def mean_headway(self) -> float:
        """Intervalo medio entre llegadas (s)."""
        vph = max(self.demand.effective_vph, 1e-6)
        return 3600.0 / vph

    @property
    def queued(self) -> int:
        return sum(len(q) for q in self.queues.values())

    def update(self, dt: float, t: float) -> tuple[list[Vehicle], int]:
        """Avanza el proceso de llegadas y devuelve (vehículos generados, descartados)."""
        created: list[Vehicle] = []
        dropped = 0
        self._time_to_next -= dt
        while self._time_to_next <= 0.0:
            vehicle = self._make_vehicle(t)
            queue = self.queues.setdefault(vehicle.route.entry_lane, deque())
            if len(queue) >= self.demand.max_queue:
                dropped += 1
            else:
                queue.append(vehicle)
                created.append(vehicle)
            self._time_to_next += self._schedule_next()
        return created, dropped

    def pop_ready(self, lane_id: str) -> Vehicle | None:
        """Devuelve (sin extraer) el primer vehículo en espera de un carril."""
        queue = self.queues.get(lane_id)
        if not queue:
            return None
        return queue[0]

    def commit(self, lane_id: str) -> None:
        """Confirma que el primer vehículo de la cola ya fue insertado en la red."""
        self.queues[lane_id].popleft()

    def make_probe(self, route_id: str, t: float) -> Vehicle:
        """Crea el 'vehículo de prueba' (mismo tipo base, marcado para seguimiento)."""
        route = self.network.route(route_id)
        vehicle = self._new_vehicle(route, self._fleet_types[0], t)
        vehicle.is_probe = True
        self.queues.setdefault(route.entry_lane, deque()).append(vehicle)
        return vehicle

    # -- interno -------------------------------------------------------------

    def _schedule_next(self) -> float:
        """Programa el siguiente instante de llegada y devuelve el headway usado."""
        mean = self.mean_headway
        if self.demand.arrival is ArrivalProcess.POISSON:
            # Llegadas de Poisson ⇒ headways exponenciales. Se acota inferiormente para
            # evitar headways de 0 s (que crearían vehículos superpuestos en la cola).
            headway = max(self.rng.expovariate(1.0 / mean), 0.25)
        else:
            headway = mean
        self._time_to_next = headway
        return headway

    def _make_vehicle(self, t: float) -> Vehicle:
        route = self.rng.choices(self._routes, weights=self._weights, k=1)[0]
        vtype = self.rng.choices(self._fleet_types, weights=self._fleet_weights, k=1)[0]
        return self._new_vehicle(route, vtype, t)

    def _new_vehicle(self, route: Route, vtype: VehicleType, t: float) -> Vehicle:
        vehicle = Vehicle(id=self._next_id, vtype=vtype, route=route, spawn_time=t)
        vehicle.lane_history.append(route.entry_lane)
        self._next_id += 1
        return vehicle
