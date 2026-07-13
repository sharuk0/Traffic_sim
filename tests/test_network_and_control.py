"""Requisitos 3, 6, 7, 8, 9, 10, 11, 12: red, semáforos, rotonda, determinismo, métricas."""

from __future__ import annotations

import pytest

from traffic_sim import (
    CAR,
    SCENARIOS,
    DemandConfig,
    Lane,
    Movement,
    Network,
    NetworkError,
    Route,
    RouteError,
    SignalState,
    Simulation,
    SimulationConfig,
    Vehicle,
    build,
)
from traffic_sim.geometry import Line
from traffic_sim.network import LaneKind

from .conftest import place, quiet_sim, straight_network

# ---------------------------------------------------------------- 3. rutas


def test_disconnected_route_is_rejected() -> None:
    """3. Una ruta cuyos carriles no están conectados en el grafo se rechaza al construir."""
    lanes = [
        Lane(id="a", geometry=Line((0, 0), (10, 0)), successors=(), kind=LaneKind.APPROACH),
        Lane(id="b", geometry=Line((10, 0), (20, 0)), successors=(), kind=LaneKind.EXIT),
    ]
    route = Route(
        id="bad", lanes=("a", "b"), origin="a", destination="b", movement=Movement.STRAIGHT
    )
    with pytest.raises(RouteError, match="no conecta"):
        Network(lanes, [route])


def test_route_with_unknown_lane_is_rejected() -> None:
    lanes = [Lane(id="a", geometry=Line((0, 0), (10, 0)), kind=LaneKind.EXIT)]
    route = Route(
        id="bad", lanes=("a", "ghost"), origin="a", destination="z", movement=Movement.STRAIGHT
    )
    with pytest.raises(RouteError, match="no existe"):
        Network(lanes, [route])


def test_geometric_discontinuity_is_rejected() -> None:
    """Un sucesor que no arranca donde termina el carril anterior es un error explícito."""
    lanes = [
        Lane(id="a", geometry=Line((0, 0), (10, 0)), successors=("b",), kind=LaneKind.APPROACH),
        Lane(id="b", geometry=Line((30, 0), (40, 0)), kind=LaneKind.EXIT),
    ]
    route = Route(
        id="r", lanes=("a", "b"), origin="a", destination="b", movement=Movement.STRAIGHT
    )
    with pytest.raises(NetworkError, match="Discontinuidad"):
        Network(lanes, [route])


def test_route_must_end_in_an_exit_lane() -> None:
    net = straight_network()
    bad = Route(
        id="short",
        lanes=("A_in", "A_mid"),
        origin="A",
        destination="A",
        movement=Movement.STRAIGHT,
    )
    with pytest.raises(RouteError, match="salida válida"):
        Network(list(net.lanes.values()), [bad])


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_all_routes_end_in_a_valid_exit(name: str) -> None:
    """11. Toda ruta generada termina en una salida válida y usa carriles existentes."""
    network = build(name).network
    exits = network.exit_lanes
    for route in network.routes.values():
        assert route.exit_lane in exits
        assert all(lid in network.lanes for lid in route.lanes)
        for a, b in zip(route.lanes, route.lanes[1:], strict=False):
            assert b in network.lane(a).successors


def test_turn_only_scenarios_generate_no_forbidden_routes() -> None:
    """11 (bis). Los escenarios de giro exclusivo no producen rutas prohibidas."""
    for name, allowed in (
        ("left_turn_only", Movement.LEFT),
        ("right_turn_only", Movement.RIGHT),
    ):
        network = build(name).network
        movements = {r.movement for r in network.routes.values()}
        assert movements == {allowed}, name


# ---------------------------------------------------------------- 6/7. semáforos


def test_red_light_stops_the_vehicle_before_the_stop_line() -> None:
    """6. Una luz roja detiene al vehículo ANTES de la línea de detención."""
    # offset=35 ⇒ en t=0 el plan ya está en la 2ª fase, con el grupo "A" en rojo.
    sim = quiet_sim(signal_group="A", offset=35.0)
    lane = sim.network.lane("A_in")
    assert sim.signal_state("A") is SignalState.RED

    vehicle = place(sim, "A_in", s=0.0, v=13.9)
    for _ in range(500):  # 25 s, dentro de la fase roja
        sim.step()
        assert sim.signal_state("A") is SignalState.RED

    assert vehicle.lane_id == "A_in"
    assert vehicle.v < 0.3, "el vehículo debe quedar detenido"
    assert vehicle.s < lane.length, "no debe rebasar la línea de detención"
    assert vehicle.s > lane.length - 6.0, "debe detenerse junto a la línea, no lejos"


def test_green_light_lets_the_vehicle_through() -> None:
    """7. Una luz verde permite continuar sin detenerse."""
    sim = quiet_sim(signal_group="A", offset=0.0)
    assert sim.signal_state("A") is SignalState.GREEN

    vehicle = place(sim, "A_in", s=0.0, v=13.9)
    for _ in range(400):  # 20 s (la fase verde dura 30 s)
        sim.step()

    assert vehicle.lane_index > 0, "debió cruzar la línea"
    assert vehicle.stopped_time == 0.0
    assert vehicle.v > 5.0


def test_signal_cycles_through_green_yellow_and_all_red() -> None:
    """El semáforo es una FSM real: pasa por verde, ámbar y todo-rojo."""
    sim = quiet_sim(signal_group="A", offset=0.0)
    seen = set()
    for _ in range(int(70 / sim.config.fixed_dt)):
        sim.step()
        seen.add(sim.signal_state("A"))
    assert seen == {SignalState.GREEN, SignalState.YELLOW, SignalState.RED}


def test_signal_supports_a_number_of_groups_other_than_four() -> None:
    """El controlador no asume 4 accesos (la T tiene 3 y funciona)."""
    scenario = build("t_junction")
    sim = Simulation(scenario, SimulationConfig(seed=2, demand=scenario.default_demand))
    assert len(sim.signals) == 1
    assert len(sim.signals[0].plan.phases) == 3
    sim.run(120.0)
    assert sim.metrics.completed > 0


# ---------------------------------------------------------------- 9. rotonda


def test_ring_traffic_has_priority_over_entering_vehicles() -> None:
    """9. Un vehículo del anillo tiene prioridad: el que entra cede el paso."""
    scenario = build("roundabout")
    config = SimulationConfig(seed=1, warmup=0.0, demand=DemandConfig(vehicles_per_hour=1e-6))
    sim = Simulation(scenario, config)

    ring_lane = sim.network.lane("ring_AW")
    yield_lane = sim.network.lane("W_yield_0")

    # Vehículo circulando por el anillo, a punto de pasar por el punto de convergencia.
    ring_route = sim.network.route("N->S")  # circula por el anillo pasando frente a W
    assert "ring_AW" in ring_route.lanes
    ring_vehicle = Vehicle(id=100, vtype=CAR, route=ring_route, spawn_time=0.0)
    ring_vehicle.lane_index = ring_route.lanes.index("ring_AW")
    ring_vehicle.s = 1.0
    ring_vehicle.v = 8.0
    ring_vehicle.entry_time = 0.0
    sim.vehicles.append(ring_vehicle)
    ring_lane.vehicles.append(ring_vehicle)

    # Vehículo esperando en la línea de ceda-el-paso del acceso W.
    entering = Vehicle(id=200, vtype=CAR, route=sim.network.route("W->E"), spawn_time=0.0)
    entering.lane_index = entering.route.lanes.index("W_yield_0")
    entering.s = yield_lane.length - 3.0
    entering.v = 0.0
    entering.entry_time = 0.0
    sim.vehicles.append(entering)
    yield_lane.vehicles.append(entering)

    for _ in range(20):  # 1 s: el vehículo del anillo aún no ha despejado N_W
        sim.step()

    assert entering.lane_id == "W_yield_0", "el que entra no debe invadir el anillo"
    assert entering.v < 1.0, "debe seguir cediendo el paso"
    assert ring_vehicle.v > 6.0, "el vehículo del anillo no debe frenar por el que cede"


def test_entering_vehicle_proceeds_once_the_ring_is_clear() -> None:
    """La aceptación de brecha no bloquea indefinidamente: con el anillo libre, entra."""
    scenario = build("roundabout")
    config = SimulationConfig(seed=1, warmup=0.0, demand=DemandConfig(vehicles_per_hour=1e-6))
    sim = Simulation(scenario, config)

    yield_lane = sim.network.lane("W_yield_0")
    entering = Vehicle(id=1, vtype=CAR, route=sim.network.route("W->E"), spawn_time=0.0)
    entering.lane_index = entering.route.lanes.index("W_yield_0")
    entering.s = yield_lane.length - 3.0
    entering.v = 0.0
    entering.entry_time = 0.0
    sim.vehicles.append(entering)
    yield_lane.vehicles.append(entering)

    sim.run(15.0)
    assert entering.lane_index > entering.route.lanes.index("W_yield_0")


# ---------------------------------------------------------------- 8. determinismo


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_same_seed_gives_identical_runs(name: str) -> None:
    """8. La simulación es reproducible con la misma semilla."""

    def run() -> list[tuple[int, str, float, float]]:
        scenario = build(name)
        sim = Simulation(
            scenario,
            SimulationConfig(seed=1234, demand=scenario.default_demand, probe_route=None),
        )
        sim.run(180.0)
        return sorted(
            (v.id, v.lane_id, round(v.s, 9), round(v.v, 9)) for v in sim.vehicles
        )

    first, second = run(), run()
    assert first == second


def test_different_seeds_give_different_runs() -> None:
    """El azar es real: semillas distintas producen trayectorias distintas."""
    scenario = build("four_way")
    results = []
    for seed in (1, 2):
        sim = Simulation(
            build("four_way"), SimulationConfig(seed=seed, demand=scenario.default_demand)
        )
        sim.run(180.0)
        results.append([(v.id, v.lane_id) for v in sim.vehicles])
    assert results[0] != results[1]


def test_simulation_does_not_depend_on_the_wall_clock() -> None:
    """La lógica no usa la hora real del computador: el periodo se elige explícitamente."""
    import traffic_sim.generator as generator_module
    import traffic_sim.simulation as simulation_module

    for module in (generator_module, simulation_module):
        source = module.__file__
        with open(source, encoding="utf-8") as fh:
            text = fh.read()
        assert "datetime.now" not in text
        assert "time.time" not in text


# ---------------------------------------------------------------- 10. escenarios


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_each_scenario_runs_for_minutes_without_exceptions(name: str) -> None:
    """10. Cada escenario corre varios minutos simulados sin lanzar excepciones."""
    scenario = build(name)
    config = SimulationConfig(
        seed=99,
        demand=scenario.default_demand,
        probe_route=scenario.default_probe_route,
    )
    sim = Simulation(scenario, config)
    sim.run(600.0)  # 10 minutos simulados

    assert sim.t == pytest.approx(600.0, abs=0.1)
    assert sim.metrics.completed > 0, name
    assert sim.overlap_fixes == 0, name
    assert sim.metrics.probe_records, f"{name}: el vehículo de prueba no completó su ruta"


# ---------------------------------------------------------------- 12. métricas


def test_metrics_reset_with_a_new_simulation() -> None:
    """12. Las métricas se reinician al crear una nueva simulación (no hay estado de clase)."""
    scenario = build("four_way")
    config = SimulationConfig(seed=4, demand=scenario.default_demand)

    first = Simulation(scenario, config)
    first.run(180.0)
    assert first.metrics.completed > 0

    second = Simulation(build("four_way"), config)
    assert second.metrics.completed == 0
    assert second.metrics.generated == 0
    assert second.metrics.entered == 0
    assert second.t == 0.0
    assert second.vehicles == []

    # Y la primera instancia conserva su propio estado: no se comparte nada.
    assert first.metrics.completed > 0


def test_reset_clears_state_in_place() -> None:
    scenario = build("t_junction")
    sim = Simulation(scenario, SimulationConfig(seed=4, demand=scenario.default_demand))
    sim.run(120.0)
    assert sim.metrics.entered > 0

    sim.reset()
    assert sim.t == 0.0
    assert sim.metrics.generated == 0
    assert sim.metrics.entered == 0
    assert sim.metrics.completed == 0
    assert sim.vehicles == []
    assert all(not lane.vehicles for lane in sim.network.lanes.values())


def test_metrics_export_csv(tmp_path) -> None:
    scenario = build("four_way")
    sim = Simulation(scenario, SimulationConfig(seed=8, demand=scenario.default_demand))
    sim.run(240.0)

    paths = sim.metrics.export_csv(tmp_path, prefix="test")
    assert len(paths) == 3
    for path in paths:
        assert path.exists()
        assert path.read_text(encoding="utf-8").count("\n") >= 2


def test_generator_queues_instead_of_dropping_demand() -> None:
    """7 (requisito): un vehículo no desaparece por no poder entrar; espera en cola."""
    scenario = build("four_way")
    demand = DemandConfig(vehicles_per_hour=6000.0)  # demanda muy superior a la capacidad
    sim = Simulation(scenario, SimulationConfig(seed=6, demand=demand))
    sim.run(300.0)

    assert sim.generator.queued > 0, "la demanda excedente debe acumularse en cola"
    assert sim.metrics.dropped == 0 or sim.metrics.queued > 0
    # Conservación exacta: nada desaparece en silencio.
    m = sim.metrics
    assert m.generated == m.entered + m.queued
    assert m.entered == m.present + m.finished
