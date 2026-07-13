"""Escenarios de intersección señalizada construidos con `build_intersection`."""

from __future__ import annotations

from ..config import (
    DemandConfig,
    DemandPeriod,
    Movement,
    PhaseConfig,
    SignalPlanConfig,
)
from .base import ArmSpec, Scenario, build_intersection, make_network

# Ángulos: W=180°, N=90°, E=0°, S=270° (dirección desde el centro hacia la boca del acceso)
_W, _N, _E, _S = 180.0, 90.0, 0.0, 270.0

_TR = frozenset({Movement.STRAIGHT, Movement.RIGHT})
_L = frozenset({Movement.LEFT})
_ONLY_LEFT = frozenset({Movement.LEFT})
_ONLY_RIGHT = frozenset({Movement.RIGHT})


def four_way() -> Scenario:
    """Intersección clásica de cuatro accesos, dos carriles por sentido.

    Carril 0 (derecho): recto + giro a la derecha.  Carril 1 (izquierdo): giro a la
    izquierda protegido. El plan tiene cuatro fases:

        1. W+E recto/derecha    2. W+E izquierda protegida
        3. N+S recto/derecha    4. N+S izquierda protegida

    Todos los movimientos en verde simultáneo son compatibles (no se cruzan), por lo que
    la intersección es libre de conflictos por construcción. Cada fase termina con ámbar
    y un intervalo de todo-rojo para despejar la caja.
    """
    movements = {0: _TR, 1: _L}
    arms = [
        ArmSpec("W", _W, in_lanes=2, out_lanes=2, movements=movements),
        ArmSpec("N", _N, in_lanes=2, out_lanes=2, movements=movements),
        ArmSpec("E", _E, in_lanes=2, out_lanes=2, movements=movements),
        ArmSpec("S", _S, in_lanes=2, out_lanes=2, movements=movements),
    ]
    lanes, routes = build_intersection(arms)
    plan = SignalPlanConfig(
        phases=(
            PhaseConfig(green_groups=("W_TR", "E_TR"), green=22.0),
            PhaseConfig(green_groups=("W_L", "E_L"), green=10.0),
            PhaseConfig(green_groups=("N_TR", "S_TR"), green=22.0),
            PhaseConfig(green_groups=("N_L", "S_L"), green=10.0),
        )
    )
    return Scenario(
        name="four_way",
        description="4 accesos, 2 carriles/sentido, giros a la izquierda protegidos",
        network=make_network(lanes, routes),
        signal_plans=(("main", plan),),
        default_demand=DemandConfig(vehicles_per_hour=900.0, period=DemandPeriod.PEAK),
        default_probe_route="W0->E",
    )


def left_turn_only() -> Scenario:
    """Intersección donde SÓLO se permite girar a la izquierda.

    Dos giros a la izquierda de accesos opuestos SÍ se cruzan en el centro, así que el
    plan da verde a un acceso por vez (giro protegido puro). No se genera ninguna ruta
    prohibida: `build_intersection` sólo crea conectores para los movimientos permitidos.
    """
    movements = {0: _ONLY_LEFT}
    arms = [
        ArmSpec("W", _W, in_lanes=1, out_lanes=1, movements=movements),
        ArmSpec("N", _N, in_lanes=1, out_lanes=1, movements=movements),
        ArmSpec("E", _E, in_lanes=1, out_lanes=1, movements=movements),
        ArmSpec("S", _S, in_lanes=1, out_lanes=1, movements=movements),
    ]
    lanes, routes = build_intersection(arms, protected_left=False)
    plan = SignalPlanConfig(
        phases=tuple(
            PhaseConfig(green_groups=(f"{arm.name}_TR",), green=14.0) for arm in arms
        )
    )
    return Scenario(
        name="left_turn_only",
        description="Sólo giros a la izquierda; una fase protegida por acceso",
        network=make_network(lanes, routes),
        signal_plans=(("main", plan),),
        default_demand=DemandConfig(vehicles_per_hour=700.0),
        default_probe_route="W0->N",
    )


def right_turn_only() -> Scenario:
    """Intersección donde SÓLO se permite girar a la derecha.

    Los giros a la derecha de distintos accesos no se cruzan entre sí (tráfico por la
    derecha), por lo que la intersección NO necesita semáforo: se resuelve por prioridad
    y por el propio modelo de seguimiento. Es el caso de "flujo continuo".
    """
    movements = {0: _ONLY_RIGHT}
    arms = [
        ArmSpec("W", _W, in_lanes=1, out_lanes=1, movements=movements),
        ArmSpec("N", _N, in_lanes=1, out_lanes=1, movements=movements),
        ArmSpec("E", _E, in_lanes=1, out_lanes=1, movements=movements),
        ArmSpec("S", _S, in_lanes=1, out_lanes=1, movements=movements),
    ]
    lanes, routes = build_intersection(arms, signal_groups=False)
    return Scenario(
        name="right_turn_only",
        description="Sólo giros a la derecha; sin semáforo (movimientos no conflictivos)",
        network=make_network(lanes, routes),
        signal_plans=(),
        default_demand=DemandConfig(vehicles_per_hour=900.0),
        default_probe_route="W0->S",
    )


def t_junction() -> Scenario:
    """Intersección en T (tres accesos). Demuestra la extensibilidad del constructor.

    El acceso Sur no existe: basta con no declararlo. Los movimientos imposibles no
    generan conectores ni rutas.
    """
    arms = [
        ArmSpec("W", _W, in_lanes=1, out_lanes=1),
        ArmSpec("E", _E, in_lanes=1, out_lanes=1),
        ArmSpec("N", _N, in_lanes=1, out_lanes=1),
    ]
    lanes, routes = build_intersection(arms, protected_left=False)
    plan = SignalPlanConfig(
        phases=(
            PhaseConfig(green_groups=("W_TR",), green=16.0),
            PhaseConfig(green_groups=("E_TR",), green=16.0),
            PhaseConfig(green_groups=("N_TR",), green=12.0),
        )
    )
    return Scenario(
        name="t_junction",
        description="Intersección en T de 3 accesos, una fase por acceso",
        network=make_network(lanes, routes),
        signal_plans=(("main", plan),),
        default_demand=DemandConfig(vehicles_per_hour=800.0),
        default_probe_route="W0->N",
    )
