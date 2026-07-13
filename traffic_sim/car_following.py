"""Modelo de seguimiento vehicular: IDM adaptado.

Ecuación implementada (Treiber, Hennecke & Helbing, 2000):

    a = a_max · [ 1 − (v / v₀)^δ − (s* / s)² ]

    s* = s₀ + max(0, v·T + v·Δv / (2·√(a_max·b_conf)))

donde
    v    velocidad actual del vehículo               [m/s]
    v₀   velocidad deseada (mín. entre v_max y el límite del carril) [m/s]
    s    separación libre (gap) con el líder          [m]
    Δv   v − v_líder (positivo si nos acercamos)      [m/s]
    s₀   separación mínima en parada                  [m]
    T    headway temporal deseado                     [s]
    δ    exponente de aceleración libre               [-]

ADAPTACIONES respecto al IDM canónico (declaradas explícitamente, no es IDM puro):

1. La salida se satura a [−b_max, a_max]. El IDM original no acota la deceleración,
   lo que produce frenados infinitos cuando s → 0.
2. `s` se acota inferiormente (`_MIN_GAP`) para eliminar la división entre cero del
   código original.
3. Los semáforos y el ceda-el-paso se modelan como un "líder virtual" detenido situado
   en la línea de detención (v_líder = 0). Así el frenado ante un rojo usa la MISMA
   dinámica que el frenado ante un vehículo: es progresivo y consistente, en lugar del
   override `a = −b_max·v/v_max` del proyecto original, que decaía exponencialmente y
   nunca llegaba a detener el vehículo.
"""

from __future__ import annotations

import math

from .config import VehicleType

_MIN_GAP: float = 0.1
"""Separación mínima usada en el denominador; evita divisiones entre cero."""


def desired_gap(v: float, delta_v: float, p: VehicleType) -> float:
    """Separación deseada s* del IDM."""
    dynamic = v * delta_v / (2.0 * math.sqrt(p.a_max * p.b_comfort))
    return p.s0 + max(0.0, v * p.T + dynamic)


def idm_acceleration(
    v: float,
    v_desired: float,
    gap: float | None,
    delta_v: float,
    p: VehicleType,
) -> float:
    """Aceleración del IDM adaptado, saturada a [−b_max, a_max].

    Args:
        v: velocidad actual (m/s), siempre ≥ 0.
        v_desired: velocidad deseada (m/s).
        gap: separación libre al líder (m). `None` = flujo libre.
        delta_v: v − v_líder (m/s).
        p: parámetros del tipo de vehículo.
    """
    v0 = max(v_desired, 0.1)
    free_term = 1.0 - (max(v, 0.0) / v0) ** p.delta

    if gap is None:
        interaction = 0.0
    else:
        s = max(gap, _MIN_GAP)
        s_star = desired_gap(v, delta_v, p)
        interaction = (s_star / s) ** 2

    a = p.a_max * (free_term - interaction)
    return max(-p.b_max, min(a, p.a_max))


def stop_line_acceleration(v: float, gap: float, p: VehicleType) -> float:
    """Deceleración MÍNIMA necesaria para detenerse `s₀` metros antes de una línea.

        a = − v² / (2·(gap − s₀))

    Se usa en lugar del término de interacción del IDM para líneas de detención y
    ceda-el-paso. Motivo: la separación deseada s* del IDM (s₀ + v·T + …) está calibrada
    para SEGUIR a otro vehículo, no para detenerse ante una línea; aplicada a un semáforo
    exige holguras de más de 10 m y produce frenados de pánico innecesarios. La fórmula
    cinemática frena exactamente lo justo, con una trayectoria de parada suave.
    """
    effective = max(gap - p.s0, 0.05)
    required = -(v * v) / (2.0 * effective)
    return max(required, -p.b_max)


def can_stop_before(distance: float, v: float, p: VehicleType, decel: float | None = None) -> bool:
    """¿Puede el vehículo detenerse antes de `distance` con la deceleración dada?

    Con `decel = b_comfort` (por defecto) decide el comportamiento ante un ÁMBAR
    (dilemma zone). Con `decel = b_max` decide si la parada es FÍSICAMENTE posible: si no
    lo es, el vehículo ya está comprometido y despejar la intersección es más seguro que
    frenar en seco dentro de ella.
    """
    b = decel if decel is not None else p.b_comfort
    braking_distance = (v * v) / (2.0 * b)
    return braking_distance <= max(distance, 0.0)
