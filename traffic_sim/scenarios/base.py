"""Bloques reutilizables para construir escenarios.

Un escenario NO copia coordenadas: describe sus accesos (`ArmSpec`) y el constructor
`build_intersection` genera carriles, conectores y rutas mediante geometría paramétrica.
Añadir una intersección en T es declarar tres accesos en lugar de cuatro.

Convenio de coordenadas: mundo en metros, X hacia el este, Y hacia el norte. El ángulo
de un acceso es la dirección DESDE el centro HACIA la boca del acceso.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..config import LANE_WIDTH, DemandConfig, Movement, SignalPlanConfig
from ..geometry import Line, Point, bezier_connector
from ..network import Lane, LaneKind, Network, Route


@dataclass(frozen=True)
class ArmSpec:
    """Un acceso (brazo) de la intersección.

    Attributes:
        name: nombre corto ("W", "N", "E", "S").
        angle_deg: dirección desde el centro hacia la boca del acceso.
        in_lanes: número de carriles de entrada.
        out_lanes: número de carriles de salida.
        movements: movimientos permitidos por carril de entrada (índice 0 = el más a la
            derecha). Si es None, todos los carriles admiten todos los movimientos.
        speed_limit: límite de velocidad del acceso (m/s).
    """

    name: str
    angle_deg: float
    in_lanes: int = 2
    out_lanes: int = 2
    movements: dict[int, frozenset[Movement]] | None = None
    speed_limit: float = 13.9

    @property
    def angle(self) -> float:
        return math.radians(self.angle_deg)

    @property
    def radial(self) -> Point:
        """Unitario desde el centro hacia la boca del acceso."""
        return (math.cos(self.angle), math.sin(self.angle))

    @property
    def inbound_dir(self) -> Point:
        """Dirección de circulación de entrada (hacia el centro)."""
        rx, ry = self.radial
        return (-rx, -ry)

    @property
    def right_normal(self) -> Point:
        """Normal a la derecha de la circulación de entrada (tráfico por la derecha)."""
        ux, uy = self.inbound_dir
        return (uy, -ux)

    def allowed(self, lane_index: int) -> frozenset[Movement]:
        if self.movements is None:
            return frozenset({Movement.LEFT, Movement.STRAIGHT, Movement.RIGHT})
        return self.movements.get(lane_index, frozenset())


@dataclass(frozen=True)
class Scenario:
    """Escenario completo: red validada + planes semafóricos + demanda por defecto."""

    name: str
    description: str
    network: Network
    signal_plans: tuple[tuple[str, SignalPlanConfig], ...] = ()
    default_demand: DemandConfig = field(default_factory=DemandConfig)
    default_probe_route: str | None = None


# --------------------------------------------------------------------------
# Helpers geométricos
# --------------------------------------------------------------------------


def _offset(point: Point, normal: Point, distance: float) -> Point:
    return (point[0] + normal[0] * distance, point[1] + normal[1] * distance)


def classify_movement(origin: ArmSpec, destination: ArmSpec) -> Movement:
    """Clasifica el movimiento origen → destino por el signo del producto cruz.

    Con tráfico por la derecha, un producto cruz negativo entre la dirección de entrada
    y la dirección de salida corresponde a un giro a la derecha.
    """
    if origin.name == destination.name:
        return Movement.UTURN
    ux, uy = origin.inbound_dir
    ox, oy = destination.radial  # dirección de salida = radial del brazo destino
    cross = ux * oy - uy * ox
    if abs(cross) < 1e-6:
        return Movement.STRAIGHT
    return Movement.LEFT if cross > 0 else Movement.RIGHT


def _target_lane_index(movement: Movement, source_index: int, out_lanes: int) -> int:
    """Carril de salida: derecha → el más a la derecha; izquierda → el más a la izquierda."""
    if movement is Movement.RIGHT:
        return 0
    if movement is Movement.LEFT:
        return out_lanes - 1
    return min(source_index, out_lanes - 1)


# --------------------------------------------------------------------------
# Constructor de intersecciones
# --------------------------------------------------------------------------


def build_intersection(
    arms: list[ArmSpec],
    *,
    approach_length: float = 110.0,
    signal_groups: bool = True,
    protected_left: bool = True,
) -> tuple[list[Lane], list[Route]]:
    """Construye carriles y rutas de una intersección con N accesos.

    Args:
        arms: accesos de la intersección (2 o más).
        approach_length: longitud de los tramos de acceso y salida (m).
        signal_groups: si True, los carriles de acceso reciben un grupo semafórico.
        protected_left: si True, los carriles que permiten girar a la izquierda forman un
            grupo semafórico propio (`<ARM>_L`), lo que permite giros protegidos.

    Returns:
        (lanes, routes) listos para construir un `Network`.
    """
    if len(arms) < 2:
        raise ValueError("Una intersección necesita al menos dos accesos")

    max_lanes = max(max(a.in_lanes, a.out_lanes) for a in arms)
    radius = LANE_WIDTH * max_lanes + 6.0  # radio de la caja de la intersección

    lanes: list[Lane] = []
    routes: list[Route] = []
    in_end: dict[tuple[str, int], Point] = {}
    out_start: dict[tuple[str, int], Point] = {}

    for arm in arms:
        rx, ry = arm.radial
        nx, ny = arm.right_normal
        far = (rx * approach_length, ry * approach_length)
        near = (rx * radius, ry * radius)

        for i in range(arm.in_lanes):
            d = LANE_WIDTH * (0.5 + i)
            start = _offset(far, (nx, ny), d)
            end = _offset(near, (nx, ny), d)
            in_end[(arm.name, i)] = end
            group = _signal_group(arm, i, signal_groups, protected_left)
            lanes.append(
                Lane(
                    id=f"{arm.name}_in_{i}",
                    geometry=Line(start, end),
                    kind=LaneKind.APPROACH,
                    speed_limit=arm.speed_limit,
                    signal_group=group,
                    approach=arm.name,
                )
            )

        for j in range(arm.out_lanes):
            d = LANE_WIDTH * (0.5 + j)
            start = _offset(near, (-nx, -ny), d)
            end = _offset(far, (-nx, -ny), d)
            out_start[(arm.name, j)] = start
            lanes.append(
                Lane(
                    id=f"{arm.name}_out_{j}",
                    geometry=Line(start, end),
                    kind=LaneKind.EXIT,
                    speed_limit=arm.speed_limit,
                    approach=arm.name,
                )
            )

    lane_by_id = {lane.id: lane for lane in lanes}

    for origin in arms:
        for i in range(origin.in_lanes):
            allowed = origin.allowed(i)
            for destination in arms:
                movement = classify_movement(origin, destination)
                if movement is Movement.UTURN or movement not in allowed:
                    continue
                j = _target_lane_index(movement, i, destination.out_lanes)
                p0 = in_end[(origin.name, i)]
                p1 = out_start[(destination.name, j)]
                connector = bezier_connector(
                    p0, origin.inbound_dir, p1, destination.radial
                )
                cid = f"{origin.name}{i}_{destination.name}{j}_{movement.value[:1]}"
                lanes.append(
                    Lane(
                        id=cid,
                        geometry=connector,
                        kind=LaneKind.CONNECTOR,
                        successors=(f"{destination.name}_out_{j}",),
                        speed_limit=_connector_speed(movement, origin.speed_limit),
                        approach=origin.name,
                    )
                )
                src = lane_by_id[f"{origin.name}_in_{i}"]
                src.successors = (*src.successors, cid)
                routes.append(
                    Route(
                        id=f"{origin.name}{i}->{destination.name}",
                        lanes=(f"{origin.name}_in_{i}", cid, f"{destination.name}_out_{j}"),
                        origin=origin.name,
                        destination=destination.name,
                        movement=movement,
                    )
                )

    return lanes, routes


def _signal_group(
    arm: ArmSpec, lane_index: int, signal_groups: bool, protected_left: bool
) -> str | None:
    if not signal_groups:
        return None
    allowed = arm.allowed(lane_index)
    if protected_left and allowed == frozenset({Movement.LEFT}):
        return f"{arm.name}_L"
    return f"{arm.name}_TR"


def _connector_speed(movement: Movement, approach_speed: float) -> float:
    """Los giros se recorren más despacio que los tramos rectos."""
    if movement is Movement.STRAIGHT:
        return approach_speed
    return min(approach_speed, 7.0)  # ≈ 25 km/h en curva


def make_network(lanes: list[Lane], routes: list[Route]) -> Network:
    """Atajo: construye y valida la red."""
    return Network(lanes, routes)
