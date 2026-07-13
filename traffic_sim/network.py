"""Red vial: carriles con identificador, grafo dirigido de conexiones y rutas validadas.

Reemplaza por completo la construcción de rutas mediante índices numéricos del proyecto
original. Aquí una ruta es una secuencia de IDs legibles (`"W_in_0" -> "W_S_r" -> "S_out_0"`)
y `Network.validate()` rechaza cualquier ruta inválida ANTES de arrancar el escenario.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

from .config import GEOMETRIC_TOLERANCE, Movement
from .geometry import Geometry, Point


class NetworkError(ValueError):
    """La topología de la red es inconsistente."""


class RouteError(ValueError):
    """Una ruta es inválida (desconectada, con carriles inexistentes, sin salida...)."""


class LaneKind(StrEnum):
    """Rol de un carril dentro de la red."""

    APPROACH = "approach"   # acceso desde el exterior hacia la intersección
    CONNECTOR = "connector"  # dentro de la intersección (recto o giro)
    EXIT = "exit"           # salida hacia el exterior
    RING = "ring"           # arco del anillo de la rotonda
    ENTRY = "entry"         # empalme acceso → anillo (cede el paso)


@dataclass
class Lane:
    """Un carril con geometría continua y estado de circulación propio.

    `vehicles` se mantiene ordenada de adelante hacia atrás: el índice 0 es el vehículo
    más avanzado (mayor `s`). Cada `Simulation` construye su propia red, de modo que no
    existe estado compartido entre instancias.
    """

    id: str
    geometry: Geometry
    kind: LaneKind = LaneKind.APPROACH
    successors: tuple[str, ...] = ()
    speed_limit: float = 13.9
    signal_group: str | None = None
    yield_to: tuple[tuple[str, float], ...] = ()
    """Carriles con prioridad sobre éste, como (lane_id, distancia_extra).

    `distancia_extra` es la distancia desde el FINAL de ese carril hasta el punto de
    conflicto real. Permite mirar varios arcos aguas arriba en la rotonda y estimar
    correctamente el tiempo de llegada (TTA) de los vehículos con prioridad."""
    merge_conflicts: tuple[str, ...] = ()
    """Carriles que se incorporan AL FINAL de éste (fusión). Un vehículo de este carril
    trata como líder virtual a todo vehículo ya comprometido en esos carriles: aunque la
    prioridad sea suya, si el otro ya invadió el punto de fusión hay que frenar."""
    approach: str | None = None
    """Nombre del acceso de origen (para métricas por origen)."""
    vehicles: list = field(default_factory=list, repr=False)

    @property
    def length(self) -> float:
        return self.geometry.length

    @property
    def has_signal(self) -> bool:
        return self.signal_group is not None

    def reset(self) -> None:
        self.vehicles.clear()


@dataclass(frozen=True)
class Route:
    """Ruta admisible: secuencia de carriles conectados desde un acceso hasta una salida."""

    id: str
    lanes: tuple[str, ...]
    origin: str
    destination: str
    movement: Movement

    @property
    def entry_lane(self) -> str:
        return self.lanes[0]

    @property
    def exit_lane(self) -> str:
        return self.lanes[-1]


class Network:
    """Contenedor de carriles y rutas, con validación estructural."""

    def __init__(self, lanes: list[Lane], routes: list[Route]) -> None:
        self.lanes: dict[str, Lane] = {}
        for lane in lanes:
            if lane.id in self.lanes:
                raise NetworkError(f"Carril duplicado: '{lane.id}'")
            self.lanes[lane.id] = lane
        self.routes: dict[str, Route] = {}
        for route in routes:
            if route.id in self.routes:
                raise RouteError(f"Ruta duplicada: '{route.id}'")
            self.routes[route.id] = route
        self.validate()

    # -- consultas -----------------------------------------------------------

    def lane(self, lane_id: str) -> Lane:
        try:
            return self.lanes[lane_id]
        except KeyError as exc:
            raise NetworkError(f"Carril inexistente: '{lane_id}'") from exc

    def route(self, route_id: str) -> Route:
        try:
            return self.routes[route_id]
        except KeyError as exc:
            raise RouteError(f"Ruta inexistente: '{route_id}'") from exc

    @property
    def exit_lanes(self) -> set[str]:
        """Carriles terminales: los que no tienen sucesores."""
        return {lid for lid, lane in self.lanes.items() if not lane.successors}

    def reset(self) -> None:
        for lane in self.lanes.values():
            lane.reset()

    # -- validación ----------------------------------------------------------

    def validate(self) -> None:
        """Comprueba la coherencia de la red y de todas las rutas.

        Lanza `NetworkError` o `RouteError` con un mensaje explícito ante:
        carriles sucesores inexistentes, discontinuidad geométrica, rutas vacías,
        rutas con saltos no conectados, o rutas que no terminan en una salida válida.
        """
        for lane in self.lanes.values():
            for succ in lane.successors:
                if succ not in self.lanes:
                    raise NetworkError(
                        f"El carril '{lane.id}' declara como sucesor a '{succ}', que no existe"
                    )
                gap = _dist(lane.geometry.end, self.lanes[succ].geometry.start)
                if gap > GEOMETRIC_TOLERANCE:
                    raise NetworkError(
                        f"Discontinuidad geométrica entre '{lane.id}' y '{succ}': "
                        f"{gap:.2f} m > {GEOMETRIC_TOLERANCE} m"
                    )
            for prio, _extra in lane.yield_to:
                if prio not in self.lanes:
                    raise NetworkError(
                        f"El carril '{lane.id}' cede el paso a '{prio}', que no existe"
                    )
            for merging in lane.merge_conflicts:
                if merging not in self.lanes:
                    raise NetworkError(
                        f"El carril '{lane.id}' declara la fusión de '{merging}', que no existe"
                    )

        exits = self.exit_lanes
        if not self.routes:
            raise RouteError("La red no define ninguna ruta")

        for route in self.routes.values():
            if len(route.lanes) < 2:
                raise RouteError(f"Ruta '{route.id}': necesita al menos dos carriles")
            for lane_id in route.lanes:
                if lane_id not in self.lanes:
                    raise RouteError(
                        f"Ruta '{route.id}': el carril '{lane_id}' no existe en la red"
                    )
            for a, b in zip(route.lanes, route.lanes[1:], strict=False):
                if b not in self.lanes[a].successors:
                    raise RouteError(
                        f"Ruta '{route.id}': '{a}' no conecta con '{b}' (ruta desconectada)"
                    )
            if route.exit_lane not in exits:
                raise RouteError(
                    f"Ruta '{route.id}': no termina en una salida válida "
                    f"('{route.exit_lane}' tiene sucesores)"
                )


def _dist(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])
