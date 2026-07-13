"""Configuración tipada del simulador.

Todas las magnitudes están en unidades SI: metros, segundos, metros/segundo.
No se usan argumentos mutables por defecto: se emplean dataclasses y `field`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

# --------------------------------------------------------------------------
# Constantes de dominio (evitan números mágicos dispersos por el código)
# --------------------------------------------------------------------------

LANE_WIDTH: float = 3.5
"""Ancho de carril en metros (estándar urbano peruano ≈ 3.0–3.5 m)."""

GEOMETRIC_TOLERANCE: float = 0.5
"""Discontinuidad máxima admitida entre el fin de un carril y el inicio del siguiente (m)."""

STOPPED_SPEED: float = 0.3
"""Umbral de velocidad por debajo del cual se considera que el vehículo está detenido (m/s)."""

HARD_BRAKE_THRESHOLD: float = -3.0
"""Aceleración por debajo de la cual se contabiliza una frenada fuerte (m/s²)."""

LEADER_LOOKAHEAD: float = 120.0
"""Distancia máxima de búsqueda del vehículo líder aguas abajo de la ruta (m)."""


class Movement(StrEnum):
    """Tipo de movimiento en una intersección (tráfico por la derecha)."""

    LEFT = "left"
    STRAIGHT = "straight"
    RIGHT = "right"
    UTURN = "uturn"


class SignalState(StrEnum):
    """Estado de un grupo semafórico."""

    GREEN = "green"
    YELLOW = "yellow"
    RED = "red"


class ArrivalProcess(StrEnum):
    """Proceso de llegada de vehículos al sistema.

    DETERMINISTIC: headways constantes (3600 / vph). Útil para depurar y comparar.
    POISSON:       headways exponenciales de media 3600 / vph. Más realista para
                   llegadas urbanas no coordinadas; introduce ráfagas y colas.
    """

    DETERMINISTIC = "deterministic"
    POISSON = "poisson"


class DemandPeriod(StrEnum):
    """Periodo de demanda. No depende del reloj del sistema: se elige explícitamente."""

    OFF_PEAK = "off_peak"
    PEAK = "peak"


# --------------------------------------------------------------------------
# Vehículos
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class VehicleType:
    """Parámetros del modelo de seguimiento vehicular (IDM adaptado).

    Attributes:
        name: Identificador legible del tipo.
        length: Longitud física del vehículo (m).
        v_max: Velocidad deseada en flujo libre (m/s).
        a_max: Aceleración máxima deseada (m/s²).
        b_comfort: Deceleración cómoda usada por el IDM (m/s²).
        b_max: Deceleración máxima física; satura la salida del modelo (m/s²).
        s0: Separación mínima en parada (m).
        T: Headway temporal deseado (s).
        delta: Exponente de aceleración libre del IDM (adimensional).
        color: Color RGB para el render.
    """

    name: str = "car"
    length: float = 4.5
    v_max: float = 13.9  # ≈ 50 km/h
    a_max: float = 1.5
    b_comfort: float = 2.0
    b_max: float = 6.0
    s0: float = 2.0
    T: float = 1.4
    delta: float = 4.0
    color: tuple[int, int, int] = (46, 74, 140)


CAR = VehicleType()
TAXI = VehicleType(name="taxi", v_max=15.3, a_max=1.8, T=1.1, color=(214, 158, 46))
BUS = VehicleType(
    name="bus",
    length=10.5,
    v_max=11.1,
    a_max=0.9,
    b_comfort=1.5,
    s0=2.5,
    T=1.8,
    color=(66, 122, 88),
)
COMBI = VehicleType(
    name="combi",
    length=6.0,
    v_max=14.0,
    a_max=1.7,
    b_comfort=2.4,
    T=1.0,
    color=(150, 60, 60),
)

DEFAULT_FLEET: tuple[tuple[VehicleType, float], ...] = (
    (CAR, 0.55),
    (TAXI, 0.20),
    (COMBI, 0.17),
    (BUS, 0.08),
)


# --------------------------------------------------------------------------
# Demanda
# --------------------------------------------------------------------------


@dataclass
class DemandConfig:
    """Configuración de la generación de vehículos.

    `vehicles_per_hour` es la demanda TOTAL del escenario; se reparte entre las rutas
    según `route_weights` (si es None, se reparte uniformemente entre todas las rutas).
    """

    vehicles_per_hour: float = 900.0
    arrival: ArrivalProcess = ArrivalProcess.POISSON
    period: DemandPeriod = DemandPeriod.OFF_PEAK
    peak_factor: float = 1.8
    """Multiplicador aplicado a la demanda cuando `period is PEAK`."""
    fleet: tuple[tuple[VehicleType, float], ...] = DEFAULT_FLEET
    route_weights: dict[str, float] | None = None
    max_queue: int = 60
    """Tamaño máximo de la cola de espera por carril de entrada (evita crecimiento ilimitado)."""

    @property
    def effective_vph(self) -> float:
        """Demanda efectiva tras aplicar el factor de hora pico."""
        factor = self.peak_factor if self.period is DemandPeriod.PEAK else 1.0
        return self.vehicles_per_hour * factor


# --------------------------------------------------------------------------
# Semáforos
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class PhaseConfig:
    """Una fase del plan semafórico.

    Los grupos listados en `green_groups` reciben VERDE durante `green` segundos,
    luego ÁMBAR durante `yellow`, y finalmente hay `all_red` segundos con todos los
    grupos en ROJO (intervalo de seguridad / despeje de la intersección).
    """

    green_groups: tuple[str, ...]
    green: float = 20.0
    yellow: float = 3.0
    all_red: float = 2.0

    @property
    def duration(self) -> float:
        return self.green + self.yellow + self.all_red


@dataclass(frozen=True)
class SignalPlanConfig:
    """Plan semafórico completo de un controlador.

    `offset` desfasa el inicio del ciclo (permite coordinar controladores).
    """

    phases: tuple[PhaseConfig, ...]
    offset: float = 0.0

    @property
    def cycle_length(self) -> float:
        return sum(p.duration for p in self.phases)


# --------------------------------------------------------------------------
# Simulación
# --------------------------------------------------------------------------


@dataclass
class SimulationConfig:
    """Parámetros globales de la simulación."""

    fixed_dt: float = 0.05
    """Paso de integración fijo (s). Independiente del framerate del render."""
    seed: int = 42
    warmup: float = 30.0
    """Tiempo simulado que se descarta de las métricas (llenado de la red)."""
    demand: DemandConfig = field(default_factory=DemandConfig)
    probe_route: str | None = None
    """Ruta del 'vehículo de prueba'; se inyecta en `probe_time`."""
    probe_time: float = 60.0
    critical_gap: float = 4.5
    """Brecha crítica de aceptación en las entradas de la rotonda (s)."""
