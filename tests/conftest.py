"""Fixtures y redes mínimas para las pruebas."""

from __future__ import annotations

import pytest

from traffic_sim import (
    CAR,
    DemandConfig,
    Lane,
    Movement,
    Network,
    PhaseConfig,
    Route,
    SignalPlanConfig,
    Simulation,
    SimulationConfig,
    Vehicle,
)
from traffic_sim.geometry import Line
from traffic_sim.network import LaneKind
from traffic_sim.scenarios.base import Scenario

LANE_LENGTH = 60.0


def straight_network(signal_group: str | None = None) -> Network:
    """Tres carriles rectos en fila: acceso → conector → salida (300 m en total)."""
    lanes = [
        Lane(
            id="A_in",
            geometry=Line((-120.0, 0.0), (-60.0, 0.0)),
            kind=LaneKind.APPROACH,
            successors=("A_mid",),
            speed_limit=13.9,
            signal_group=signal_group,
            approach="A",
        ),
        Lane(
            id="A_mid",
            geometry=Line((-60.0, 0.0), (0.0, 0.0)),
            kind=LaneKind.CONNECTOR,
            successors=("A_out",),
            speed_limit=13.9,
        ),
        Lane(
            id="A_out",
            geometry=Line((0.0, 0.0), (60.0, 0.0)),
            kind=LaneKind.EXIT,
            speed_limit=13.9,
        ),
    ]
    routes = [
        Route(
            id="A->A",
            lanes=("A_in", "A_mid", "A_out"),
            origin="A",
            destination="A",
            movement=Movement.STRAIGHT,
        )
    ]
    return Network(lanes, routes)


def straight_scenario(signal_group: str | None = None, offset: float = 0.0) -> Scenario:
    """Escenario recto. Si hay grupo semafórico, el plan alterna entre ese grupo y otro."""
    plans: tuple[tuple[str, SignalPlanConfig], ...] = ()
    if signal_group:
        plans = (
            (
                "ctrl",
                SignalPlanConfig(
                    phases=(
                        PhaseConfig(green_groups=(signal_group,), green=30.0,
                                    yellow=3.0, all_red=2.0),
                        PhaseConfig(green_groups=("OTHER",), green=30.0,
                                    yellow=3.0, all_red=2.0),
                    ),
                    offset=offset,
                ),
            ),
        )
    return Scenario(
        name="straight",
        description="Carril recto de prueba",
        network=straight_network(signal_group),
        signal_plans=plans,
        default_demand=DemandConfig(vehicles_per_hour=0.1),
    )


def quiet_sim(signal_group: str | None = None, offset: float = 0.0, **kwargs) -> Simulation:
    """Simulación sin demanda automática: los vehículos se insertan a mano."""
    config = SimulationConfig(
        seed=1,
        warmup=0.0,
        demand=DemandConfig(vehicles_per_hour=1e-6),
        **kwargs,
    )
    return Simulation(straight_scenario(signal_group, offset), config)


def place(sim: Simulation, lane_id: str, s: float, v: float, vid: int = 1) -> Vehicle:
    """Inserta un vehículo directamente en un carril (sin pasar por el generador)."""
    route = sim.network.route("A->A")
    lane_index = route.lanes.index(lane_id)
    vehicle = Vehicle(id=vid, vtype=CAR, route=route, spawn_time=sim.t)
    vehicle.lane_index = lane_index
    vehicle.s = s
    vehicle.v = v
    vehicle.entry_time = sim.t
    vehicle.lane_history.append(lane_id)
    sim.vehicles.append(vehicle)
    sim.network.lane(lane_id).vehicles.append(vehicle)
    sim.network.lane(lane_id).vehicles.sort(key=lambda x: -x.s)
    sim._update_pose(vehicle, initial=True)
    return vehicle


@pytest.fixture
def sim() -> Simulation:
    return quiet_sim()
