"""El agente vehículo.

Un vehículo es UN objeto durante toda su vida. Al cambiar de carril NO se copia
(`deepcopy` eliminado): sólo se actualizan `lane_index` y `s`. Por eso su ID, su
tiempo de creación, su tiempo detenido y su historial sobreviven a los empalmes,
lo que hace posible el "vehículo de prueba" y las métricas por vehículo.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .config import STOPPED_SPEED, VehicleType
from .network import Route

Pose = tuple[float, float, float]  # (x, y, heading)


@dataclass(eq=False)  # identidad por objeto, no por valor
class Vehicle:
    """Estado completo de un vehículo.

    Attributes:
        id: identificador único e inmutable durante toda la vida del vehículo.
        s: distancia recorrida sobre el carril actual (m).
        v: velocidad (m/s), garantizada ≥ 0.
        a: aceleración del último paso (m/s²).
        pose / prev_pose: poses en el mundo del paso actual y el anterior; el renderer
            interpola entre ambas, lo que desacopla el suavizado visual del paso físico.
    """

    id: int
    vtype: VehicleType
    route: Route
    spawn_time: float

    lane_index: int = 0
    s: float = 0.0
    v: float = 0.0
    a: float = 0.0

    entry_time: float | None = None
    exit_time: float | None = None
    stopped_time: float = 0.0
    distance_travelled: float = 0.0
    hard_brakes: int = 0
    is_probe: bool = False
    yield_committed: bool = False
    """Una vez el conductor decide entrar al anillo, no reevalúa la brecha: sin esta
    'memoria' la aceptación oscilaría entre pasos y produciría frenadas de pánico."""

    pose: Pose = (0.0, 0.0, 0.0)
    prev_pose: Pose = (0.0, 0.0, 0.0)
    lane_history: list[str] = field(default_factory=list, repr=False)

    # -- consultas -----------------------------------------------------------

    @property
    def length(self) -> float:
        return self.vtype.length

    @property
    def lane_id(self) -> str:
        return self.route.lanes[self.lane_index]

    @property
    def is_last_lane(self) -> bool:
        return self.lane_index >= len(self.route.lanes) - 1

    @property
    def next_lane_id(self) -> str | None:
        if self.is_last_lane:
            return None
        return self.route.lanes[self.lane_index + 1]

    @property
    def is_stopped(self) -> bool:
        return self.v < STOPPED_SPEED

    @property
    def travel_time(self) -> float | None:
        if self.entry_time is None or self.exit_time is None:
            return None
        return self.exit_time - self.entry_time

    @property
    def mean_speed(self) -> float:
        t = self.travel_time
        if not t:
            return 0.0
        return self.distance_travelled / t

    # -- mutación ------------------------------------------------------------

    def advance_lane(self, leftover: float) -> None:
        """Pasa al siguiente carril de la ruta conservando la distancia sobrante."""
        self.lane_index += 1
        self.s = leftover
        self.yield_committed = False
        self.lane_history.append(self.lane_id)
