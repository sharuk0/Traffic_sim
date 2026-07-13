"""Núcleo de la simulación microscópica.

Orden de actualización de cada paso (paso de tiempo fijo `fixed_dt`):

1. Actualizar los semáforos con el tiempo simulado.
2. Generar demanda e insertar los vehículos en cola que tengan hueco físico.
3. Calcular la aceleración de TODOS los vehículos a partir del estado actual
   (fase de sólo lectura: nadie ve un estado a medio actualizar).
4. Integrar con Euler semiimplícito y transferir carriles conservando el sobrante.
5. Muestrear métricas y avanzar el reloj.

Integración (Euler semiimplícito, incondicionalmente más estable que el explícito
para sistemas disipativos como el seguimiento vehicular):

    a_{n}   = IDM(estado_n)
    v_{n+1} = clamp(v_n + a_n·Δt, 0, v_max)
    s_{n+1} = s_n + v_{n+1}·Δt

La aceleración interviene UNA sola vez (el código original la aplicaba a la velocidad
y otra vez en el término ½·a·Δt² de la posición, sobreestimando el desplazamiento).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from .car_following import can_stop_before, idm_acceleration
from .config import (
    HARD_BRAKE_THRESHOLD,
    LEADER_LOOKAHEAD,
    STOPPED_SPEED,
    SignalState,
    SimulationConfig,
)
from .generator import VehicleGenerator
from .metrics import Metrics
from .network import Lane, LaneKind, Network
from .signals import TrafficSignal
from .vehicle import Vehicle

logger = logging.getLogger(__name__)

_STOP_BUFFER: float = 0.3
"""Holgura extra antes de una línea de detención (m)."""

_YIELD_BLOCK_DISTANCE: float = 3.0
"""Un vehículo prioritario a menos de esta distancia del punto de conflicto lo bloquea (m)."""

_MERGE_ZONE: float = 8.0
"""Distancia al punto de fusión dentro de la cual un vehículo comprometido es visible
para quien circula por la vía prioritaria (m)."""

_YIELD_COMMIT_DISTANCE: float = 6.0
"""Una vez aceptada la brecha a menos de esta distancia de la línea, la maniobra se
compromete y ya no se reevalúa (evita oscilaciones aceptar/rechazar entre pasos)."""


@dataclass(frozen=True)
class Obstacle:
    """Líder virtual detenido (línea de detención o punto de ceda-el-paso)."""

    gap: float


class Simulation:
    """Una simulación independiente. No comparte NINGÚN estado con otras instancias."""

    def __init__(self, scenario, config: SimulationConfig | None = None) -> None:
        self.scenario = scenario
        self.config = config or SimulationConfig()
        self.network: Network = scenario.network
        self.paused: bool = False
        self.overlap_fixes: int = 0
        """Veces que la red de seguridad tuvo que corregir un solapamiento.
        Debe permanecer en 0: si crece, el modelo de seguimiento está mal calibrado."""
        self.reset()

    # -- ciclo de vida -------------------------------------------------------

    def reset(self) -> None:
        """Reinicia por completo el estado (métricas incluidas)."""
        self.t = 0.0
        self.step_count = 0
        self.overlap_fixes = 0
        self.network.reset()
        self.vehicles: list[Vehicle] = []
        self.signals: list[TrafficSignal] = [
            TrafficSignal(name, plan) for name, plan in self.scenario.signal_plans
        ]
        self._group_signal: dict[str, TrafficSignal] = {}
        for sig in self.signals:
            for group in sig.groups:
                self._group_signal[group] = sig
        self._check_signal_groups()
        self.generator = VehicleGenerator(
            self.network, self.config.demand, self.config.seed
        )
        self.metrics = Metrics(warmup=self.config.warmup)
        self._probe_injected = False

    def _check_signal_groups(self) -> None:
        """Todo grupo declarado por un carril debe existir en algún plan."""
        for lane in self.network.lanes.values():
            if lane.signal_group and lane.signal_group not in self._group_signal:
                raise ValueError(
                    f"El carril '{lane.id}' pertenece al grupo semafórico "
                    f"'{lane.signal_group}', que ningún plan controla"
                )

    # -- bucle ---------------------------------------------------------------

    def run(self, duration: float) -> None:
        """Ejecuta `duration` segundos simulados (modo headless)."""
        steps = int(round(duration / self.config.fixed_dt))
        for _ in range(steps):
            self.step()

    def step(self) -> None:
        """Avanza exactamente un paso de `fixed_dt` segundos."""
        dt = self.config.fixed_dt

        for signal in self.signals:
            signal.update(self.t)

        self._spawn(dt)

        # Fase de sólo lectura: se calculan todas las aceleraciones sobre el estado
        # actual antes de mover a nadie (evita que un vehículo vea a su líder ya movido).
        accelerations: dict[int, float] = {}
        for lane in self.network.lanes.values():
            for idx, vehicle in enumerate(lane.vehicles):
                accelerations[id(vehicle)] = self._acceleration(vehicle, lane, idx)

        for vehicle in self.vehicles:
            self._integrate(vehicle, accelerations[id(vehicle)], dt)

        self._transfer()
        self._rebuild_lanes()
        self._sample_metrics()

        self.t += dt
        self.step_count += 1

    # -- generación ----------------------------------------------------------

    def _spawn(self, dt: float) -> None:
        created, dropped = self.generator.update(dt, self.t)
        for _ in created:
            self.metrics.on_generate()
        for _ in range(dropped):
            self.metrics.on_drop()

        if (
            self.config.probe_route
            and not self._probe_injected
            and self.t >= self.config.probe_time
        ):
            self.generator.make_probe(self.config.probe_route, self.t)
            self.metrics.on_generate()
            self._probe_injected = True
            logger.info("Vehículo de prueba inyectado en t=%.1fs", self.t)

        for lane_id in list(self.generator.queues):
            lane = self.network.lane(lane_id)
            while True:
                pending = self.generator.pop_ready(lane_id)
                if pending is None or not self._has_room(lane, pending):
                    break
                self.generator.commit(lane_id)
                self._insert(lane, pending)

    def _has_room(self, lane: Lane, vehicle: Vehicle) -> bool:
        """¿Cabe el vehículo al inicio del carril sin solaparse con el último de la cola?"""
        if not lane.vehicles:
            return True
        rear = lane.vehicles[-1]
        return rear.s - rear.length >= vehicle.vtype.s0

    def _insert(self, lane: Lane, vehicle: Vehicle) -> None:
        v_free = min(vehicle.vtype.v_max, lane.speed_limit)
        if lane.vehicles:
            v_free = min(v_free, lane.vehicles[-1].v)
        vehicle.s = 0.0
        vehicle.v = v_free
        vehicle.entry_time = self.t
        lane.vehicles.append(vehicle)
        self.vehicles.append(vehicle)
        self._update_pose(vehicle, initial=True)
        self.metrics.on_enter()

    # -- dinámica ------------------------------------------------------------

    def _acceleration(self, vehicle: Vehicle, lane: Lane, index: int) -> float:
        p = vehicle.vtype
        v0 = min(p.v_max, lane.speed_limit)

        lead, gap = self._find_leader(vehicle, lane, index)
        if lead is None:
            a = idm_acceleration(vehicle.v, v0, None, 0.0, p)
        else:
            a = idm_acceleration(vehicle.v, v0, gap, vehicle.v - lead.v, p)

        obstacle = self._obstacle(vehicle, lane)
        if obstacle is not None:
            # Líder virtual DETENIDO en la línea: el frenado usa la misma dinámica que
            # ante un vehículo, por lo que es progresivo y anticipado. Un frenado
            # puramente cinemático ("lo justo para parar") sería más brusco para la fila
            # de atrás, porque el líder frenaría demasiado tarde.
            a = min(a, idm_acceleration(vehicle.v, v0, obstacle.gap, vehicle.v, p))

        a = min(a, self._speed_limit_anticipation(vehicle, lane))
        a = min(a, self._merge_constraint(vehicle, lane))
        return a

    def _merge_constraint(self, vehicle: Vehicle, lane: Lane) -> float:
        """Frenado ante un vehículo ya comprometido a incorporarse al final de este carril.

        La prioridad del anillo se expresa en la DECISIÓN de entrar (`_gap_accepted`), no
        en la negativa a frenar: si otro conductor ya invadió el punto de fusión, frenamos.
        Sin esto, dos vehículos podrían ocupar el mismo punto del anillo.
        """
        if not lane.merge_conflicts:
            return float("inf")
        p = vehicle.vtype
        v0 = min(p.v_max, lane.speed_limit)
        best = float("inf")
        for merging_id in lane.merge_conflicts:
            merging = self.network.lane(merging_id)
            for other in merging.vehicles:
                # Todo vehículo en el carril de incorporación ya cruzó la línea de
                # ceda-el-paso: está comprometido por construcción.
                if merging.length - other.s > _MERGE_ZONE:
                    continue
                gap = (lane.length - vehicle.s) - other.length
                best = min(
                    best,
                    idm_acceleration(vehicle.v, v0, gap, vehicle.v - other.v, p),
                )
        return best

    def _speed_limit_anticipation(self, vehicle: Vehicle, lane: Lane) -> float:
        """Deceleración necesaria para llegar al siguiente carril ya a su límite.

        Sin esto, un vehículo llega a 13.9 m/s a un conector limitado a 7 m/s y sufre un
        salto de velocidad instantáneo (contabilizado como frenada fuerte). Se usa la
        cinemática exacta: a = (v_lim² − v²) / (2·d), saturada a −b_max.
        """
        next_id = vehicle.next_lane_id
        if next_id is None:
            return float("inf")
        v_limit = min(vehicle.vtype.v_max, self.network.lane(next_id).speed_limit)
        if vehicle.v <= v_limit:
            return float("inf")
        distance = max(lane.length - vehicle.s, 0.1)
        required = (v_limit**2 - vehicle.v**2) / (2.0 * distance)
        return max(required, -vehicle.vtype.b_max)

    def _find_leader(
        self, vehicle: Vehicle, lane: Lane, index: int
    ) -> tuple[Vehicle | None, float]:
        """Busca el líder en el mismo carril y, si no hay, aguas abajo de la RUTA.

        Esta búsqueda entre carriles es lo que impide que dos vehículos se superpongan
        en un empalme: la restricción existe antes de que el vehículo cruce la frontera.
        """
        if index > 0:
            lead = lane.vehicles[index - 1]
            return lead, lead.s - lead.length - vehicle.s

        distance = lane.length - vehicle.s
        i = vehicle.lane_index
        route_lanes = vehicle.route.lanes
        while distance < LEADER_LOOKAHEAD and i + 1 < len(route_lanes):
            i += 1
            nxt = self.network.lane(route_lanes[i])
            if nxt.vehicles:
                lead = nxt.vehicles[-1]
                return lead, distance + lead.s - lead.length
            distance += nxt.length
        return None, 0.0

    def _obstacle(self, vehicle: Vehicle, lane: Lane) -> Obstacle | None:
        """Línea de detención activa (semáforo o ceda-el-paso) para este vehículo."""
        distance = lane.length - vehicle.s - _STOP_BUFFER
        if distance < -vehicle.length:
            return None
        if not can_stop_before(distance, vehicle.v, vehicle.vtype, decel=vehicle.vtype.b_max):
            # Punto de no retorno: frenar aquí dejaría el vehículo detenido DENTRO de la
            # intersección. Es más seguro despejarla.
            return None

        if lane.has_signal:
            state = self._group_signal[lane.signal_group].state(lane.signal_group)
            if state is SignalState.RED:
                return Obstacle(gap=distance)
            if state is SignalState.YELLOW and can_stop_before(
                distance, vehicle.v, vehicle.vtype
            ):
                return Obstacle(gap=distance)

        if lane.yield_to and not vehicle.yield_committed:
            if not self._gap_accepted(lane):
                return Obstacle(gap=distance)
            if distance <= _YIELD_COMMIT_DISTANCE:
                vehicle.yield_committed = True

        return None

    def _gap_accepted(self, lane: Lane) -> bool:
        """Aceptación de brecha en una entrada de rotonda.

        Se aceptan las brechas cuyo tiempo de llegada al punto de conflicto (TTA) de
        todo vehículo prioritario supere la brecha crítica configurada. Un vehículo
        prioritario muy próximo al punto de conflicto bloquea la entrada siempre.

        Es una aproximación: no modela la aceleración del vehículo entrante ni brechas
        críticas heterogéneas por conductor. Se documenta como tal en el README.
        """
        critical = self.config.critical_gap
        for prio_id, extra in lane.yield_to:
            prio = self.network.lane(prio_id)
            for other in prio.vehicles:
                distance_to_conflict = (prio.length - other.s) + extra
                if distance_to_conflict < _YIELD_BLOCK_DISTANCE:
                    return False
                tta = distance_to_conflict / max(other.v, 0.5)
                if tta < critical:
                    return False
        return True

    def _integrate(self, vehicle: Vehicle, a: float, dt: float) -> None:
        """Euler semiimplícito con saturación de velocidad y conteo de frenadas fuertes."""
        lane = self.network.lane(vehicle.lane_id)
        v_cap = min(vehicle.vtype.v_max, lane.speed_limit)

        if a <= HARD_BRAKE_THRESHOLD and vehicle.a > HARD_BRAKE_THRESHOLD:
            vehicle.hard_brakes += 1
            if self.t >= self.config.warmup:
                self.metrics.hard_brakes += 1

        vehicle.a = a
        v_new = vehicle.v + a * dt
        vehicle.v = min(max(v_new, 0.0), v_cap)  # nunca negativa, nunca sobre el límite

        step = vehicle.v * dt
        vehicle.s += step
        vehicle.distance_travelled += step
        if vehicle.v < STOPPED_SPEED and self.t >= self.config.warmup:
            vehicle.stopped_time += dt

    # -- transferencia entre carriles ---------------------------------------

    def _transfer(self) -> None:
        """Pasa los vehículos al siguiente carril CONSERVANDO la distancia sobrante.

        Sin `deepcopy`: el mismo objeto continúa la ruta, así que ID, tiempos y métricas
        sobreviven. El bucle `while` admite que un vehículo atraviese varios carriles
        cortos en un solo paso, algo que el código original no contemplaba.
        """
        finished: list[Vehicle] = []
        for vehicle in self.vehicles:
            lane = self.network.lane(vehicle.lane_id)
            while vehicle.s >= lane.length:
                leftover = vehicle.s - lane.length
                if vehicle.is_last_lane:
                    finished.append(vehicle)
                    vehicle.s = lane.length
                    break
                vehicle.advance_lane(leftover)
                lane = self.network.lane(vehicle.lane_id)

        for vehicle in finished:
            self.metrics.on_complete(vehicle, self.t)
        if finished:
            done = set(map(id, finished))
            self.vehicles = [v for v in self.vehicles if id(v) not in done]

    def _rebuild_lanes(self) -> None:
        """Reconstruye la ocupación de cada carril (ordenada de adelante hacia atrás)."""
        for lane in self.network.lanes.values():
            lane.vehicles.clear()
        for vehicle in self.vehicles:
            self.network.lane(vehicle.lane_id).vehicles.append(vehicle)
        for lane in self.network.lanes.values():
            lane.vehicles.sort(key=lambda v: -v.s)
            self._enforce_separation(lane)
        for vehicle in self.vehicles:
            self._update_pose(vehicle)

    def _enforce_separation(self, lane: Lane) -> None:
        """Red de seguridad numérica: impide un solapamiento físico si el IDM fallara.

        En condiciones normales NUNCA se activa (`overlap_fixes == 0`); existe para que
        un error de calibración no produzca estados imposibles. Cada activación se cuenta.
        """
        for i in range(1, len(lane.vehicles)):
            lead = lane.vehicles[i - 1]
            veh = lane.vehicles[i]
            limit = lead.s - lead.length
            if veh.s > limit:
                veh.s = max(limit, 0.0)
                veh.v = min(veh.v, lead.v)
                self.overlap_fixes += 1

    # -- pose y métricas -----------------------------------------------------

    def _update_pose(self, vehicle: Vehicle, initial: bool = False) -> None:
        lane = self.network.lane(vehicle.lane_id)
        s = min(vehicle.s, lane.length)
        x, y = lane.geometry.point_at(s)
        heading = lane.geometry.heading_at(s)
        vehicle.prev_pose = vehicle.pose if not initial else (x, y, heading)
        vehicle.pose = (x, y, heading)

    def _sample_metrics(self) -> None:
        queue = sum(
            1
            for lane in self.network.lanes.values()
            if lane.kind is LaneKind.APPROACH
            for v in lane.vehicles
            if v.is_stopped
        )
        self.metrics.sample(self.t, self.vehicles, queue, self.generator.queued)

    # -- utilidades ----------------------------------------------------------

    @property
    def probe(self) -> Vehicle | None:
        """El vehículo de prueba mientras siga circulando."""
        for v in self.vehicles:
            if v.is_probe:
                return v
        return None

    def signal_state(self, group: str) -> SignalState:
        signal = self._group_signal.get(group)
        return signal.state(group) if signal else SignalState.GREEN
