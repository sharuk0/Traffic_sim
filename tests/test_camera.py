"""La cámara: `to_world` debe ser la inversa exacta de `to_screen`."""

from __future__ import annotations

import pytest

from traffic_sim.rendering.camera import Camera


def test_to_world_is_the_exact_inverse_of_to_screen() -> None:
    cam = Camera(1280, 800, zoom=4.7, center=(12.0, -3.5))
    for point in ((0.0, 0.0), (100.0, -40.0), (-33.3, 71.9)):
        assert cam.to_world(cam.to_screen(point)) == pytest.approx(point, abs=1e-9)


def test_screen_centre_maps_to_camera_centre() -> None:
    cam = Camera(800, 600, zoom=2.0, center=(5.0, 7.0))
    assert cam.to_screen((5.0, 7.0)) == pytest.approx((400.0, 300.0))


def test_north_is_drawn_upwards() -> None:
    cam = Camera(800, 600, zoom=2.0)
    assert cam.to_screen((0.0, 10.0))[1] < cam.to_screen((0.0, 0.0))[1]


def test_zoom_keeps_the_point_under_the_cursor_fixed() -> None:
    cam = Camera(1000, 800, zoom=3.0, center=(0.0, 0.0))
    pixel = (250.0, 600.0)
    anchor = cam.to_world(pixel)
    cam.zoom_at(1.5, pixel)
    assert cam.to_world(pixel) == pytest.approx(anchor, abs=1e-9)
    assert cam.zoom == pytest.approx(4.5)


def test_zoom_is_clamped() -> None:
    cam = Camera(800, 600, zoom=3.0)
    for _ in range(50):
        cam.zoom_at(2.0, (400.0, 300.0))
    assert cam.zoom <= 30.0
    for _ in range(80):
        cam.zoom_at(0.5, (400.0, 300.0))
    assert cam.zoom >= 0.8
