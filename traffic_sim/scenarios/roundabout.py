"""Escenario: rotonda de cuatro accesos con prioridad al anillo.

Geometría del anillo
--------------------
Circulación ANTIHORARIA (tráfico por la derecha). Por cada acceso `i` situado en el
ángulo θᵢ se definen dos nodos sobre el anillo:

    E_i = θᵢ − 20°   nodo de SALIDA (divergencia)
    N_i = θᵢ + 20°   nodo de ENTRADA (convergencia)

Recorriendo el anillo en sentido antihorario (ángulo creciente), un vehículo pasa
primero por el punto de salida y después por el de entrada, igual que en la realidad.
El anillo son entonces 8 arcos exactos (`Arc`, no polilíneas):

    A_i : E_i → N_i   (40°)   contiene el punto de divergencia en su inicio
    B_i : N_i → E_i₊₁ (50°)   tramo entre accesos

Prioridad
---------
El carril de acceso cede el paso (`yield_to`) a los vehículos que circulan por el anillo
y se dirigen a su punto de convergencia N_i: el arco A_i y, aguas arriba, el arco B_i₋₁
(con la distancia extra que le falta para llegar a N_i). Un vehículo entra sólo si el
tiempo de llegada (TTA) de todo vehículo prioritario supera la brecha crítica.

Aproximaciones declaradas: la brecha crítica es homogénea entre conductores, no se modela
la aceleración del vehículo entrante durante la maniobra, y los vehículos del anillo no
ceden el paso jamás (prioridad absoluta al anillo).
"""

from __future__ import annotations

import math

from ..config import LANE_WIDTH, DemandConfig, DemandPeriod
from ..geometry import Arc, Line, bezier_connector
from ..network import Lane, LaneKind, Route
from .base import ArmSpec, Scenario, classify_movement, make_network

RING_RADIUS: float = 22.0
RING_SPEED: float = 8.0
ENTRY_SPEED: float = 6.0
APPROACH_LENGTH: float = 110.0
GIVE_WAY_RADIUS: float = RING_RADIUS + 11.0
"""Distancia al centro de la línea de ceda-el-paso."""
YIELD_ZONE_LENGTH: float = 40.0
"""Zona de deceleración previa a la línea de ceda-el-paso (m).

Los conductores reducen la velocidad AL APROXIMARSE a una rotonda, aunque el anillo esté
libre. Modelarlo con un tramo de límite reducido evita que un vehículo llegue a la línea
a 50 km/h y tenga que frenar en pánico si aparece un vehículo en el anillo."""
YIELD_ZONE_SPEED: float = 7.0
NODE_OFFSET_DEG: float = 20.0


def _ring_point(angle_deg: float) -> tuple[float, float]:
    a = math.radians(angle_deg)
    return (RING_RADIUS * math.cos(a), RING_RADIUS * math.sin(a))


def _ring_tangent(angle_deg: float) -> tuple[float, float]:
    """Tangente antihoraria del anillo en el ángulo dado."""
    a = math.radians(angle_deg)
    return (-math.sin(a), math.cos(a))


def roundabout() -> Scenario:
    """Rotonda de cuatro accesos, un carril por sentido y un carril en el anillo."""
    # Ordenados por ángulo creciente = orden de recorrido antihorario.
    arms = [
        ArmSpec("E", 0.0, in_lanes=1, out_lanes=1, speed_limit=13.9),
        ArmSpec("N", 90.0, in_lanes=1, out_lanes=1, speed_limit=13.9),
        ArmSpec("W", 180.0, in_lanes=1, out_lanes=1, speed_limit=13.9),
        ArmSpec("S", 270.0, in_lanes=1, out_lanes=1, speed_limit=13.9),
    ]
    n = len(arms)
    lanes: list[Lane] = []

    exit_angle = [arm.angle_deg - NODE_OFFSET_DEG for arm in arms]
    entry_angle = [arm.angle_deg + NODE_OFFSET_DEG for arm in arms]

    # ---- arcos del anillo --------------------------------------------------
    for i, arm in enumerate(arms):
        lanes.append(
            Lane(
                id=f"ring_A{arm.name}",
                geometry=Arc((0.0, 0.0), RING_RADIUS, math.radians(exit_angle[i]),
                             math.radians(entry_angle[i])),
                kind=LaneKind.RING,
                successors=(f"ring_B{arm.name}",),
                speed_limit=RING_SPEED,
                merge_conflicts=(f"entry_{arm.name}",),
            )
        )
        nxt = arms[(i + 1) % n]
        # El siguiente nodo de salida puede requerir sumar una vuelta completa para que
        # el barrido del arco sea siempre positivo (antihorario).
        end_deg = exit_angle[(i + 1) % n]
        if end_deg <= entry_angle[i]:
            end_deg += 360.0
        lanes.append(
            Lane(
                id=f"ring_B{arm.name}",
                geometry=Arc((0.0, 0.0), RING_RADIUS, math.radians(entry_angle[i]),
                             math.radians(end_deg)),
                kind=LaneKind.RING,
                successors=(f"ring_A{nxt.name}", f"exit_{nxt.name}"),
                speed_limit=RING_SPEED,
            )
        )

    # ---- accesos, entradas, salidas ---------------------------------------
    for i, arm in enumerate(arms):
        rx, ry = arm.radial
        nx, ny = arm.right_normal
        half = LANE_WIDTH / 2.0

        yield_radius = GIVE_WAY_RADIUS + YIELD_ZONE_LENGTH
        far_in = (rx * APPROACH_LENGTH + nx * half, ry * APPROACH_LENGTH + ny * half)
        mid_in = (rx * yield_radius + nx * half, ry * yield_radius + ny * half)
        near_in = (rx * GIVE_WAY_RADIUS + nx * half, ry * GIVE_WAY_RADIUS + ny * half)
        near_out = (rx * GIVE_WAY_RADIUS - nx * half, ry * GIVE_WAY_RADIUS - ny * half)
        far_out = (rx * APPROACH_LENGTH - nx * half, ry * APPROACH_LENGTH - ny * half)

        prev = arms[(i - 1) % n]
        arc_a_len = math.radians(2 * NODE_OFFSET_DEG) * RING_RADIUS

        lanes.append(
            Lane(
                id=f"{arm.name}_in_0",
                geometry=Line(far_in, mid_in),
                kind=LaneKind.APPROACH,
                successors=(f"{arm.name}_yield_0",),
                speed_limit=arm.speed_limit,
                approach=arm.name,
            )
        )
        lanes.append(
            Lane(
                id=f"{arm.name}_yield_0",
                geometry=Line(mid_in, near_in),
                kind=LaneKind.APPROACH,
                successors=(f"entry_{arm.name}",),
                speed_limit=YIELD_ZONE_SPEED,
                # La línea de ceda-el-paso está al final del ACCESO (antes de la curva de
                # incorporación). Es un carril largo, así que el frenado hasta la línea es
                # progresivo. La distancia extra hasta el punto real de convergencia N_i es
                # la longitud de la curva de entrada.
                yield_to=(
                    (f"ring_A{arm.name}", 0.0),
                    (f"ring_B{prev.name}", arc_a_len),
                ),
                approach=arm.name,
            )
        )
        lanes.append(
            Lane(
                id=f"entry_{arm.name}",
                geometry=bezier_connector(
                    near_in,
                    arm.inbound_dir,
                    _ring_point(entry_angle[i]),
                    _ring_tangent(entry_angle[i]),
                    tension=0.45,
                ),
                kind=LaneKind.ENTRY,
                successors=(f"ring_B{arm.name}",),
                speed_limit=ENTRY_SPEED,
                approach=arm.name,
            )
        )
        lanes.append(
            Lane(
                id=f"exit_{arm.name}",
                geometry=bezier_connector(
                    _ring_point(exit_angle[i]),
                    _ring_tangent(exit_angle[i]),
                    near_out,
                    arm.radial,
                    tension=0.45,
                ),
                kind=LaneKind.CONNECTOR,
                successors=(f"{arm.name}_out_0",),
                speed_limit=ENTRY_SPEED,
                approach=arm.name,
            )
        )
        lanes.append(
            Lane(
                id=f"{arm.name}_out_0",
                geometry=Line(near_out, far_out),
                kind=LaneKind.EXIT,
                speed_limit=arm.speed_limit,
                approach=arm.name,
            )
        )

    # ---- rutas -------------------------------------------------------------
    routes: list[Route] = []
    for i, origin in enumerate(arms):
        for k in range(1, n):  # se excluye el giro en U (k = 0)
            j = (i + k) % n
            destination = arms[j]
            ring: list[str] = [f"ring_B{origin.name}"]
            for m in range(1, k):
                mid = arms[(i + m) % n]
                ring.append(f"ring_A{mid.name}")
                ring.append(f"ring_B{mid.name}")
            routes.append(
                Route(
                    id=f"{origin.name}->{destination.name}",
                    lanes=(
                        f"{origin.name}_in_0",
                        f"{origin.name}_yield_0",
                        f"entry_{origin.name}",
                        *ring,
                        f"exit_{destination.name}",
                        f"{destination.name}_out_0",
                    ),
                    origin=origin.name,
                    destination=destination.name,
                    movement=classify_movement(origin, destination),
                )
            )

    return Scenario(
        name="roundabout",
        description="Rotonda de 4 accesos; prioridad al anillo con aceptación de brecha",
        network=make_network(lanes, routes),
        signal_plans=(),
        default_demand=DemandConfig(vehicles_per_hour=1000.0, period=DemandPeriod.OFF_PEAK),
        default_probe_route="W->E",
    )
