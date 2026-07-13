"""Semáforo: máquina de estados configurable.

Cada controlador recorre una lista de fases. Una fase concede VERDE a un conjunto
arbitrario de grupos, luego ÁMBAR a esos mismos grupos, y termina con un intervalo
de TODO-ROJO (despeje de la intersección).

Diferencias con el proyecto original:

* El número de fases y de grupos es arbitrario (antes se asumían exactamente 4 accesos
  y había un hack `if len(self.roads) < 4: index = 3` que dejaba en rojo permanente a
  cualquier grupo distinto del 0).
* Existen ÁMBAR y TODO-ROJO.
* Las duraciones son configurables por fase y hay `offset` para coordinar controladores.
* El estado se deriva del tiempo simulado, no del reloj del sistema.
"""

from __future__ import annotations

from .config import PhaseConfig, SignalPlanConfig, SignalState


class TrafficSignal:
    """Controlador semafórico basado en un plan de fases cíclico."""

    def __init__(self, name: str, plan: SignalPlanConfig) -> None:
        if not plan.phases:
            raise ValueError(f"El plan del semáforo '{name}' no tiene fases")
        if plan.cycle_length <= 0:
            raise ValueError(f"El plan del semáforo '{name}' tiene duración de ciclo nula")
        self.name = name
        self.plan = plan
        self.groups: set[str] = {g for phase in plan.phases for g in phase.green_groups}
        self._phase_index = 0
        self._phase_elapsed = 0.0
        self._states: dict[str, SignalState] = {g: SignalState.RED for g in self.groups}
        self.update(0.0)

    @property
    def current_phase(self) -> PhaseConfig:
        return self.plan.phases[self._phase_index]

    @property
    def phase_index(self) -> int:
        return self._phase_index

    @property
    def phase_elapsed(self) -> float:
        return self._phase_elapsed

    @property
    def time_to_phase_end(self) -> float:
        return self.current_phase.duration - self._phase_elapsed

    def state(self, group: str) -> SignalState:
        """Estado actual de un grupo. Un grupo desconocido se considera en VERDE
        (carriles sin control semafórico)."""
        return self._states.get(group, SignalState.GREEN)

    def update(self, t: float) -> None:
        """Recalcula la fase activa a partir del tiempo simulado (sin acumular deriva)."""
        cycle = self.plan.cycle_length
        tau = (t + self.plan.offset) % cycle

        acc = 0.0
        for i, phase in enumerate(self.plan.phases):
            if tau < acc + phase.duration:
                self._phase_index = i
                self._phase_elapsed = tau - acc
                break
            acc += phase.duration
        else:  # pragma: no cover - sólo por errores de redondeo en el módulo
            self._phase_index = len(self.plan.phases) - 1
            self._phase_elapsed = self.plan.phases[-1].duration

        phase = self.current_phase
        if self._phase_elapsed < phase.green:
            active = SignalState.GREEN
        elif self._phase_elapsed < phase.green + phase.yellow:
            active = SignalState.YELLOW
        else:
            active = SignalState.RED  # intervalo de todo-rojo

        for group in self.groups:
            self._states[group] = active if group in phase.green_groups else SignalState.RED
