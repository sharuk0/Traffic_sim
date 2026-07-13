"""Geometrías continuas parametrizadas por longitud de arco.

Cada geometría expone:

* `length`            longitud total (m)
* `point_at(s)`       posición en el mundo a distancia `s` desde el inicio
* `tangent_at(s)`     vector tangente unitario en `s` (dirección de avance)
* `polyline(n)`       muestreo para el renderer

Parametrizar por longitud de arco (y no por el parámetro nativo `t` de la curva) es
lo que permite que un vehículo avance `v·dt` metros reales sobre una curva, y que su
orientación sea la tangente exacta de la vía. Con esto una curva es UN carril, no una
cadena de segmentos rectos.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from bisect import bisect_right

Point = tuple[float, float]


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


class Geometry(ABC):
    """Interfaz común de todas las geometrías de carril."""

    @property
    @abstractmethod
    def length(self) -> float:
        """Longitud total de la geometría en metros."""

    @abstractmethod
    def point_at(self, s: float) -> Point:
        """Punto del mundo a distancia de arco `s` (se satura a [0, length])."""

    @abstractmethod
    def tangent_at(self, s: float) -> Point:
        """Tangente unitaria a distancia de arco `s`."""

    def heading_at(self, s: float) -> float:
        """Rumbo en radianes a distancia de arco `s`."""
        tx, ty = self.tangent_at(s)
        return math.atan2(ty, tx)

    def polyline(self, n: int = 24) -> list[Point]:
        """Muestreo uniforme en longitud de arco, para dibujar."""
        n = max(2, n)
        return [self.point_at(self.length * i / (n - 1)) for i in range(n)]

    @property
    def start(self) -> Point:
        return self.point_at(0.0)

    @property
    def end(self) -> Point:
        return self.point_at(self.length)


class Line(Geometry):
    """Segmento recto entre dos puntos."""

    def __init__(self, start: Point, end: Point) -> None:
        dx, dy = end[0] - start[0], end[1] - start[1]
        self._length = math.hypot(dx, dy)
        if self._length <= 0.0:
            raise ValueError(f"Line degenerada: start == end == {start}")
        self._p0 = start
        self._dir = (dx / self._length, dy / self._length)

    @property
    def length(self) -> float:
        return self._length

    def point_at(self, s: float) -> Point:
        s = _clamp(s, 0.0, self._length)
        return (self._p0[0] + self._dir[0] * s, self._p0[1] + self._dir[1] * s)

    def tangent_at(self, s: float) -> Point:  # noqa: ARG002 - tangente constante
        return self._dir

    def polyline(self, n: int = 24) -> list[Point]:  # noqa: ARG002
        return [self.start, self.end]


class Arc(Geometry):
    """Arco de circunferencia.

    Los ángulos se miden en radianes desde el eje +X, en sentido antihorario.
    Si `end_angle > start_angle` el recorrido es antihorario; en caso contrario, horario.
    """

    def __init__(self, center: Point, radius: float, start_angle: float, end_angle: float) -> None:
        if radius <= 0.0:
            raise ValueError("El radio del arco debe ser positivo")
        if math.isclose(start_angle, end_angle):
            raise ValueError("Arco degenerado: start_angle == end_angle")
        self._center = center
        self._radius = radius
        self._a0 = start_angle
        self._sweep = end_angle - start_angle
        self._sign = 1.0 if self._sweep > 0 else -1.0
        self._length = radius * abs(self._sweep)

    @property
    def length(self) -> float:
        return self._length

    def _angle_at(self, s: float) -> float:
        s = _clamp(s, 0.0, self._length)
        return self._a0 + self._sign * (s / self._radius)

    def point_at(self, s: float) -> Point:
        a = self._angle_at(s)
        return (
            self._center[0] + self._radius * math.cos(a),
            self._center[1] + self._radius * math.sin(a),
        )

    def tangent_at(self, s: float) -> Point:
        a = self._angle_at(s)
        # d/ds (cos a, sin a) ∝ (-sin a, cos a) · sign
        return (-math.sin(a) * self._sign, math.cos(a) * self._sign)


class _Sampled(Geometry):
    """Base para geometrías cuya longitud de arco se obtiene por muestreo.

    Construye una tabla (t_i, longitud_acumulada_i) y resuelve `s → t` por búsqueda
    binaria + interpolación lineal. El punto y la tangente se evalúan de forma EXACTA
    en el `t` recuperado, no sobre la poligonal: por eso la orientación es suave.
    """

    def __init__(self, samples: int = 96) -> None:
        self._ts: list[float] = []
        self._cum: list[float] = []
        n = max(8, samples)
        prev = self._eval(0.0)
        self._ts.append(0.0)
        self._cum.append(0.0)
        total = 0.0
        for i in range(1, n + 1):
            t = i / n
            cur = self._eval(t)
            total += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            self._ts.append(t)
            self._cum.append(total)
            prev = cur
        self._length = total
        if self._length <= 0.0:
            raise ValueError("Geometría muestreada degenerada (longitud nula)")

    @abstractmethod
    def _eval(self, t: float) -> Point:
        """Punto de la curva en el parámetro nativo t ∈ [0, 1]."""

    @abstractmethod
    def _deriv(self, t: float) -> Point:
        """Derivada dP/dt en el parámetro nativo t ∈ [0, 1]."""

    @property
    def length(self) -> float:
        return self._length

    def _t_at(self, s: float) -> float:
        s = _clamp(s, 0.0, self._length)
        i = bisect_right(self._cum, s) - 1
        i = min(max(i, 0), len(self._cum) - 2)
        seg = self._cum[i + 1] - self._cum[i]
        frac = 0.0 if seg <= 1e-12 else (s - self._cum[i]) / seg
        return self._ts[i] + frac * (self._ts[i + 1] - self._ts[i])

    def point_at(self, s: float) -> Point:
        return self._eval(self._t_at(s))

    def tangent_at(self, s: float) -> Point:
        dx, dy = self._deriv(self._t_at(s))
        norm = math.hypot(dx, dy)
        if norm <= 1e-12:
            return (1.0, 0.0)
        return (dx / norm, dy / norm)


class CubicBezier(_Sampled):
    """Curva de Bézier cúbica. Es la geometría usada para los conectores (giros)."""

    def __init__(self, p0: Point, c1: Point, c2: Point, p3: Point, samples: int = 96) -> None:
        self._p0, self._c1, self._c2, self._p3 = p0, c1, c2, p3
        super().__init__(samples=samples)

    def _eval(self, t: float) -> Point:
        u = 1.0 - t
        b0, b1, b2, b3 = u * u * u, 3 * u * u * t, 3 * u * t * t, t * t * t
        return (
            b0 * self._p0[0] + b1 * self._c1[0] + b2 * self._c2[0] + b3 * self._p3[0],
            b0 * self._p0[1] + b1 * self._c1[1] + b2 * self._c2[1] + b3 * self._p3[1],
        )

    def _deriv(self, t: float) -> Point:
        u = 1.0 - t
        d0, d1, d2 = 3 * u * u, 6 * u * t, 3 * t * t
        return (
            d0 * (self._c1[0] - self._p0[0])
            + d1 * (self._c2[0] - self._c1[0])
            + d2 * (self._p3[0] - self._c2[0]),
            d0 * (self._c1[1] - self._p0[1])
            + d1 * (self._c2[1] - self._c1[1])
            + d2 * (self._p3[1] - self._c2[1]),
        )


class Polyline(Geometry):
    """Polilínea: cadena de segmentos rectos recorrida por longitud acumulada."""

    def __init__(self, points: list[Point]) -> None:
        if len(points) < 2:
            raise ValueError("Una polilínea necesita al menos dos puntos")
        self._pts = list(points)
        self._cum = [0.0]
        for a, b in zip(self._pts, self._pts[1:], strict=False):
            self._cum.append(self._cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self._length = self._cum[-1]
        if self._length <= 0.0:
            raise ValueError("Polilínea degenerada (longitud nula)")

    @property
    def length(self) -> float:
        return self._length

    def _locate(self, s: float) -> tuple[int, float]:
        s = _clamp(s, 0.0, self._length)
        i = bisect_right(self._cum, s) - 1
        i = min(max(i, 0), len(self._pts) - 2)
        seg = self._cum[i + 1] - self._cum[i]
        frac = 0.0 if seg <= 1e-12 else (s - self._cum[i]) / seg
        return i, frac

    def point_at(self, s: float) -> Point:
        i, frac = self._locate(s)
        a, b = self._pts[i], self._pts[i + 1]
        return (a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac)

    def tangent_at(self, s: float) -> Point:
        i, _ = self._locate(s)
        a, b = self._pts[i], self._pts[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        norm = math.hypot(dx, dy)
        return (dx / norm, dy / norm)

    def polyline(self, n: int = 24) -> list[Point]:  # noqa: ARG002
        return list(self._pts)


def bezier_connector(
    p0: Point,
    dir_in: Point,
    p1: Point,
    dir_out: Point,
    tension: float = 0.55,
) -> CubicBezier:
    """Conector suave entre dos carriles, tangente a las direcciones dadas.

    Los puntos de control se colocan sobre las tangentes de entrada y salida, a una
    fracción `tension` de la distancia entre extremos. Esto garantiza continuidad G1
    en ambos empalmes para CUALQUIER par de ángulos: sustituye a la heurística del
    proyecto original, que sólo funcionaba con geometría alineada a los ejes.
    """
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    k = max(tension * dist, 1e-3)
    c1 = (p0[0] + dir_in[0] * k, p0[1] + dir_in[1] * k)
    c2 = (p1[0] - dir_out[0] * k, p1[1] - dir_out[1] * k)
    return CubicBezier(p0, c1, c2, p1)
