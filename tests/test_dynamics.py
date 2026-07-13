"""Requisitos 1, 2, 4, 5: velocidad, separación, identidad y distancia sobrante."""

from __future__ import annotations

import math

import pytest

from traffic_sim import CAR, SimulationConfig
from traffic_sim.car_following import idm_acceleration
from traffic_sim.scenarios import build
from traffic_sim.simulation import Simulation

from .conftest import place, quiet_sim


def test_speed_is_never_negative_when_braking_hard() -> None:
    """1. Un vehículo nunca alcanza velocidad negativa, ni frenando al máximo."""
    sim = quiet_sim()
    leader = place(sim, "A_in", s=20.0, v=0.0, vid=1)
    follower = place(sim, "A_in", s=1.0, v=13.9, vid=2)

    for _ in range(600):  # 30 s
        sim.step()
        assert follower.v >= 0.0
        assert leader.v >= 0.0


def test_speed_never_negative_across_all_scenarios() -> None:
    """1 (bis). Ninguna velocidad negativa en ningún escenario, bajo demanda real."""
    for name in ("four_way", "roundabout", "t_junction"):
        scenario = build(name)
        sim = Simulation(
            scenario, SimulationConfig(seed=11, demand=scenario.default_demand)
        )
        for _ in range(4000):  # 200 s
            sim.step()
            assert all(v.v >= 0.0 for v in sim.vehicles), name


def test_vehicles_never_overlap_in_the_same_lane() -> None:
    """2. Dos vehículos nunca se superponen: el hueco libre es siempre ≥ 0."""
    for name in ("four_way", "roundabout", "right_turn_only", "t_junction"):
        scenario = build(name)
        sim = Simulation(
            scenario, SimulationConfig(seed=5, demand=scenario.default_demand)
        )
        for _ in range(6000):  # 300 s
            sim.step()
            for lane in sim.network.lanes.values():
                for lead, follower in zip(lane.vehicles, lane.vehicles[1:], strict=False):
                    gap = lead.s - lead.length - follower.s
                    assert gap >= -1e-6, f"{name}/{lane.id}: solapamiento de {-gap:.3f} m"
        # La red de seguridad no debió activarse: el propio modelo evita el solapamiento.
        assert sim.overlap_fixes == 0, name


def test_vehicle_keeps_identity_across_lanes() -> None:
    """4. El mismo objeto continúa la ruta: ID, tiempos e historial sobreviven."""
    sim = quiet_sim()
    vehicle = place(sim, "A_in", s=55.0, v=10.0)
    original = id(vehicle)
    vehicle.hard_brakes = 3  # marca arbitraria que debe sobrevivir

    for _ in range(400):
        sim.step()
        if vehicle.lane_index > 0:
            break

    assert vehicle.lane_index == 1
    assert vehicle.lane_id == "A_mid"
    assert id(vehicle) == original
    assert vehicle.id == 1
    assert vehicle.entry_time == 0.0
    assert vehicle.hard_brakes == 3
    assert vehicle.lane_history == ["A_in", "A_mid"]
    # El vehículo en el carril nuevo es EL MISMO objeto (no una copia).
    assert sim.network.lane("A_mid").vehicles[0] is vehicle


def test_leftover_distance_is_preserved_on_lane_change() -> None:
    """5. La distancia sobrante del paso se aplica sobre la vía siguiente."""
    sim = quiet_sim()
    lane = sim.network.lane("A_in")
    vehicle = place(sim, "A_in", s=lane.length - 0.05, v=10.0)

    before = vehicle.distance_travelled
    sim.step()

    assert vehicle.lane_index == 1
    step_distance = vehicle.v * sim.config.fixed_dt
    expected_leftover = (lane.length - 0.05) + step_distance - lane.length
    assert vehicle.s == pytest.approx(expected_leftover, abs=1e-9)
    # No se pierde ni se teletransporta movimiento alguno.
    assert vehicle.distance_travelled - before == pytest.approx(step_distance, abs=1e-9)


def test_leftover_across_multiple_short_lanes() -> None:
    """5 (bis). Un vehículo puede atravesar varios carriles cortos en un solo paso."""
    sim = quiet_sim()
    mid = sim.network.lane("A_mid")
    # Colocado casi al final de A_in con una velocidad enorme: el paso lo lleva más allá
    # de A_mid completo. La distancia debe repartirse, no perderse.
    vehicle = place(sim, "A_in", s=sim.network.lane("A_in").length - 0.1, v=13.9)
    vehicle.v = 13.9
    sim.step()
    assert vehicle.lane_index >= 1
    assert 0.0 <= vehicle.s < mid.length + 1e-9
    assert vehicle.distance_travelled > 0.0
    assert math.isfinite(vehicle.s)


def test_idm_never_divides_by_zero_nor_returns_infinity() -> None:
    """El modelo está acotado incluso con hueco nulo o negativo."""
    for gap in (-5.0, 0.0, 1e-9, 0.5, 1000.0):
        a = idm_acceleration(13.9, 13.9, gap, 13.9, CAR)
        assert math.isfinite(a)
        assert -CAR.b_max <= a <= CAR.a_max


def test_speed_limit_is_respected() -> None:
    """La velocidad nunca supera el límite del carril ni el máximo del vehículo."""
    scenario = build("right_turn_only")
    sim = Simulation(
        scenario, SimulationConfig(seed=3, demand=scenario.default_demand)
    )
    for _ in range(4000):
        sim.step()
        for lane in sim.network.lanes.values():
            cap = min(lane.speed_limit, CAR.v_max) + 1e-6
            for v in lane.vehicles:
                assert v.v <= min(lane.speed_limit, v.vtype.v_max) + 1e-6, (lane.id, v.v, cap)
