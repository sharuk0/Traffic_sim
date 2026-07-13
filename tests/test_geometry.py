"""Pruebas de las geometrías continuas (longitud de arco, tangentes, continuidad)."""

from __future__ import annotations

import math

import pytest

from traffic_sim.geometry import Arc, CubicBezier, Line, Polyline, bezier_connector


def test_line_length_and_tangent() -> None:
    line = Line((0.0, 0.0), (3.0, 4.0))
    assert line.length == pytest.approx(5.0)
    assert line.point_at(2.5) == pytest.approx((1.5, 2.0))
    assert line.tangent_at(0.0) == pytest.approx((0.6, 0.8))


def test_line_rejects_degenerate_geometry() -> None:
    with pytest.raises(ValueError, match="degenerada"):
        Line((1.0, 1.0), (1.0, 1.0))


def test_arc_is_exact() -> None:
    arc = Arc((0.0, 0.0), 10.0, 0.0, math.pi / 2)
    assert arc.length == pytest.approx(10.0 * math.pi / 2)
    assert arc.point_at(0.0) == pytest.approx((10.0, 0.0))
    assert arc.point_at(arc.length) == pytest.approx((0.0, 10.0), abs=1e-9)
    # Tangente antihoraria en el punto inicial: hacia +Y.
    assert arc.tangent_at(0.0) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_clockwise_arc_reverses_the_tangent() -> None:
    arc = Arc((0.0, 0.0), 10.0, math.pi / 2, 0.0)
    assert arc.tangent_at(0.0) == pytest.approx((1.0, 0.0), abs=1e-9)


def test_bezier_arclength_parametrisation_is_uniform() -> None:
    """Avanzar `s` metros sobre la curva recorre `s` metros reales (± error de muestreo)."""
    curve = CubicBezier((0, 0), (10, 0), (10, 10), (20, 10))
    step = curve.length / 40
    previous = curve.point_at(0.0)
    for i in range(1, 41):
        point = curve.point_at(step * i)
        d = math.hypot(point[0] - previous[0], point[1] - previous[1])
        assert d == pytest.approx(step, rel=0.03)
        previous = point


def test_bezier_tangent_is_continuous() -> None:
    """El rumbo cambia de forma suave: no hay saltos entre 'segmentos'."""
    curve = CubicBezier((0, 0), (15, 0), (15, 15), (0, 20))
    headings = [curve.heading_at(curve.length * i / 200) for i in range(201)]
    for a, b in zip(headings, headings[1:], strict=False):
        delta = abs((b - a + math.pi) % (2 * math.pi) - math.pi)
        assert delta < 0.05, "salto de orientación en la curva"


def test_connector_is_tangent_to_both_ends() -> None:
    """El conector empalma con continuidad G1: la tangente coincide en ambos extremos."""
    curve = bezier_connector((0.0, 0.0), (1.0, 0.0), (20.0, 20.0), (0.0, 1.0))
    assert curve.tangent_at(0.0) == pytest.approx((1.0, 0.0), abs=1e-6)
    assert curve.tangent_at(curve.length) == pytest.approx((0.0, 1.0), abs=1e-6)


def test_connector_handles_a_straight_line_without_collapsing() -> None:
    """Regresión del bug original: un conector 'recto' no debe degenerar."""
    curve = bezier_connector((0.0, 0.0), (1.0, 0.0), (30.0, 0.0), (1.0, 0.0))
    assert curve.length == pytest.approx(30.0, rel=1e-3)
    assert curve.point_at(15.0)[0] == pytest.approx(15.0, rel=1e-2)


def test_polyline_locates_points_by_cumulative_length() -> None:
    poly = Polyline([(0, 0), (10, 0), (10, 10)])
    assert poly.length == pytest.approx(20.0)
    assert poly.point_at(15.0) == pytest.approx((10.0, 5.0))
    assert poly.tangent_at(15.0) == pytest.approx((0.0, 1.0))


def test_point_at_is_clamped_outside_the_domain() -> None:
    line = Line((0.0, 0.0), (10.0, 0.0))
    assert line.point_at(-5.0) == pytest.approx((0.0, 0.0))
    assert line.point_at(50.0) == pytest.approx((10.0, 0.0))
