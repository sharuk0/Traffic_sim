"""Benchmark ligero: mide el coste por paso sin exigir resultados dependientes del hardware.

No se afirma un rendimiento absoluto; sólo se comprueba que el simulador se mantiene en un
orden de magnitud razonable (útil para detectar regresiones catastróficas, por ejemplo un
`deepcopy` reintroducido por accidente). El umbral es deliberadamente laxo.
"""

from __future__ import annotations

import time

from traffic_sim import Simulation, SimulationConfig, build

MIN_STEPS_PER_SECOND = 200.0
"""Umbral muy conservador: en una máquina moderna se superan varios miles."""


def test_step_throughput_is_reasonable(capsys) -> None:
    scenario = build("four_way")
    sim = Simulation(
        scenario, SimulationConfig(seed=1, demand=scenario.default_demand)
    )
    sim.run(120.0)  # llenar la red antes de medir

    steps = 4000
    start = time.perf_counter()
    for _ in range(steps):
        sim.step()
    elapsed = time.perf_counter() - start

    sps = steps / elapsed
    realtime_factor = sps * sim.config.fixed_dt
    with capsys.disabled():
        print(
            f"\n  {sps:,.0f} pasos/s · {realtime_factor:,.0f}× tiempo real "
            f"· {sim.metrics.present} vehículos en red"
        )
    assert sps > MIN_STEPS_PER_SECOND
