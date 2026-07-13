"""Simulador microscópico de tráfico urbano (modelo de seguimiento vehicular tipo IDM)."""

from __future__ import annotations

from .config import (
    BUS,
    CAR,
    COMBI,
    TAXI,
    ArrivalProcess,
    DemandConfig,
    DemandPeriod,
    Movement,
    PhaseConfig,
    SignalPlanConfig,
    SignalState,
    SimulationConfig,
    VehicleType,
)
from .metrics import Metrics
from .network import Lane, LaneKind, Network, NetworkError, Route, RouteError
from .scenarios import SCENARIOS, Scenario, build
from .simulation import Simulation
from .vehicle import Vehicle

__version__ = "2.0.0"

__all__ = [
    "BUS",
    "CAR",
    "COMBI",
    "SCENARIOS",
    "TAXI",
    "ArrivalProcess",
    "DemandConfig",
    "DemandPeriod",
    "Lane",
    "LaneKind",
    "Metrics",
    "Movement",
    "Network",
    "NetworkError",
    "PhaseConfig",
    "Route",
    "RouteError",
    "Scenario",
    "SignalPlanConfig",
    "SignalState",
    "Simulation",
    "SimulationConfig",
    "Vehicle",
    "VehicleType",
    "build",
    "__version__",
]
