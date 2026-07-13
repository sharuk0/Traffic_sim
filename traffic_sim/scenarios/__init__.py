"""Registro de escenarios seleccionables desde la línea de comandos o la ventana."""

from __future__ import annotations

from collections.abc import Callable

from .base import ArmSpec, Scenario, build_intersection, make_network
from .intersections import four_way, left_turn_only, right_turn_only, t_junction
from .roundabout import roundabout

SCENARIOS: dict[str, Callable[[], Scenario]] = {
    "four_way": four_way,
    "roundabout": roundabout,
    "left_turn_only": left_turn_only,
    "right_turn_only": right_turn_only,
    "t_junction": t_junction,
}


def build(name: str) -> Scenario:
    """Construye un escenario por nombre. Lanza `KeyError` con la lista de opciones."""
    try:
        factory = SCENARIOS[name]
    except KeyError as exc:
        options = ", ".join(sorted(SCENARIOS))
        raise KeyError(f"Escenario desconocido '{name}'. Opciones: {options}") from exc
    return factory()


__all__ = [
    "SCENARIOS",
    "ArmSpec",
    "Scenario",
    "build",
    "build_intersection",
    "four_way",
    "left_turn_only",
    "make_network",
    "right_turn_only",
    "roundabout",
    "t_junction",
]
