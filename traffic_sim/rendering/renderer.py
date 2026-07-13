"""Dibujo de la simulación. El renderer NO contiene lógica de dominio: sólo lee estado.

La geometría de la red es estática, así que se cachea en una `Surface` y sólo se
redibuja cuando cambia la cámara. Los vehículos se dibujan interpolando entre
`prev_pose` y `pose` con el factor `alpha` del acumulador de tiempo fijo: la animación
es suave aunque la simulación avance a pasos discretos de `fixed_dt`.
"""

from __future__ import annotations

import math

import pygame

from ..config import LANE_WIDTH, SignalState
from ..network import LaneKind
from ..simulation import Simulation
from .camera import Camera

BACKGROUND = (238, 238, 234)
ASPHALT = (108, 112, 120)
ASPHALT_EDGE = (86, 90, 98)
MARKING = (232, 232, 226)
ARROW = (188, 192, 200)
STOP_LINE = (245, 245, 245)
PROBE_COLOR = (222, 60, 120)
PANEL_BG = (24, 26, 32)
PANEL_FG = (232, 234, 240)
PANEL_DIM = (150, 156, 168)

SIGNAL_COLORS = {
    SignalState.GREEN: (60, 190, 90),
    SignalState.YELLOW: (240, 190, 60),
    SignalState.RED: (220, 70, 70),
}


class Renderer:
    """Dibuja una `Simulation` sobre una superficie de pygame."""

    def __init__(self, sim: Simulation, camera: Camera) -> None:
        self.sim = sim
        self.camera = camera
        self.show_debug = False
        self._static: pygame.Surface | None = None
        self._static_key: tuple[float, float, float, int, int] | None = None
        self.font = pygame.font.SysFont("dejavusansmono,consolas,monospace", 14)
        self.font_big = pygame.font.SysFont("dejavusans,arial", 18, bold=True)

    # -- red estática --------------------------------------------------------

    def _camera_key(self) -> tuple[float, float, float, int, int]:
        return (
            round(self.camera.zoom, 4),
            round(self.camera.center[0], 3),
            round(self.camera.center[1], 3),
            self.camera.width,
            self.camera.height,
        )

    def _network_surface(self) -> pygame.Surface:
        key = self._camera_key()
        if self._static is not None and self._static_key == key:
            return self._static

        surf = pygame.Surface((self.camera.width, self.camera.height))
        surf.fill(BACKGROUND)
        cam = self.camera
        width_px = max(int(cam.scale(LANE_WIDTH)), 2)

        for lane in self.sim.network.lanes.values():
            pts = [cam.to_screen(p) for p in lane.geometry.polyline(_samples(lane.length))]
            if len(pts) >= 2:
                pygame.draw.lines(surf, ASPHALT_EDGE, False, pts, width_px + 2)
        for lane in self.sim.network.lanes.values():
            pts = [cam.to_screen(p) for p in lane.geometry.polyline(_samples(lane.length))]
            if len(pts) >= 2:
                pygame.draw.lines(surf, ASPHALT, False, pts, width_px)

        for lane in self.sim.network.lanes.values():
            self._draw_arrows(surf, lane, width_px)

        self._static = surf
        self._static_key = key
        return surf

    def _draw_arrows(self, surf: pygame.Surface, lane, width_px: int) -> None:
        if lane.length < 12.0 or width_px < 6:
            return
        cam = self.camera
        spacing = 25.0
        n = max(int(lane.length // spacing), 1)
        for i in range(n):
            s = lane.length * (i + 0.5) / n
            px, py = cam.to_screen(lane.geometry.point_at(s))
            tx, ty = lane.geometry.tangent_at(s)
            angle = math.atan2(-ty, tx)  # pantalla: Y invertida
            size = cam.scale(1.6)
            tip = (px + math.cos(angle) * size, py + math.sin(angle) * size)
            left = (
                px + math.cos(angle + 2.5) * size * 0.8,
                py + math.sin(angle + 2.5) * size * 0.8,
            )
            right = (
                px + math.cos(angle - 2.5) * size * 0.8,
                py + math.sin(angle - 2.5) * size * 0.8,
            )
            pygame.draw.polygon(surf, ARROW, [tip, left, right])

    # -- capas dinámicas -----------------------------------------------------

    def _draw_stop_lines(self, screen: pygame.Surface) -> None:
        cam = self.camera
        for lane in self.sim.network.lanes.values():
            if lane.kind is not LaneKind.APPROACH:
                continue
            if not lane.has_signal and not lane.yield_to:
                continue
            end = lane.geometry.end
            tx, ty = lane.geometry.tangent_at(lane.length)
            nx, ny = -ty, tx  # normal en el mundo
            half = LANE_WIDTH / 2.0
            a = cam.to_screen((end[0] + nx * half, end[1] + ny * half))
            b = cam.to_screen((end[0] - nx * half, end[1] - ny * half))
            if lane.has_signal:
                color = SIGNAL_COLORS[self.sim.signal_state(lane.signal_group)]
                thickness = max(int(cam.scale(0.9)), 3)
            else:
                color = STOP_LINE
                thickness = max(int(cam.scale(0.5)), 2)
            pygame.draw.line(screen, color, a, b, thickness)

    def _draw_vehicles(self, screen: pygame.Surface, alpha: float) -> None:
        cam = self.camera
        for vehicle in self.sim.vehicles:
            x0, y0, h0 = vehicle.prev_pose
            x1, y1, h1 = vehicle.pose
            x = x0 + (x1 - x0) * alpha
            y = y0 + (y1 - y0) * alpha
            heading = h0 + _angle_delta(h0, h1) * alpha

            length = vehicle.length
            width = 1.9
            cos_h, sin_h = math.cos(heading), math.sin(heading)
            corners = []
            for dl, dw in ((0.5, 0.5), (0.5, -0.5), (-0.5, -0.5), (-0.5, 0.5)):
                wx = x + cos_h * dl * length - sin_h * dw * width
                wy = y + sin_h * dl * length + cos_h * dw * width
                corners.append(cam.to_screen((wx, wy)))
            color = PROBE_COLOR if vehicle.is_probe else vehicle.vtype.color
            pygame.draw.polygon(screen, color, corners)
            if cam.zoom > 3.0:
                pygame.draw.polygon(screen, (20, 20, 24), corners, 1)

    def _draw_debug(self, screen: pygame.Surface) -> None:
        cam = self.camera
        for lane in self.sim.network.lanes.values():
            mid = lane.geometry.point_at(lane.length / 2.0)
            label = self.font.render(lane.id, True, (40, 40, 48))
            screen.blit(label, cam.to_screen(mid))
        for vehicle in self.sim.vehicles:
            x, y, _ = vehicle.pose
            label = self.font.render(f"{vehicle.v:.1f}", True, (10, 10, 10))
            screen.blit(label, cam.to_screen((x, y + 2.0)))

    def _draw_panel(self, screen: pygame.Surface, speed: float, fps: float) -> None:
        sim = self.sim
        m = sim.metrics.summary()
        lines = [
            (f"{sim.scenario.name}", self.font_big, PANEL_FG),
            (f"t = {sim.t:8.1f} s   x{speed:g}   {'PAUSA' if sim.paused else 'CORRIENDO'}",
             self.font, PANEL_DIM),
            ("", self.font, PANEL_DIM),
            (f"generados     {m['generated']:>8}", self.font, PANEL_FG),
            (f"ingresados    {m['entered']:>8}", self.font, PANEL_FG),
            (f"completados   {m['completed']:>8}", self.font, PANEL_FG),
            (f"presentes     {m['present']:>8}", self.font, PANEL_FG),
            (f"en cola       {m['queued']:>8}", self.font, PANEL_FG),
            (f"descartados   {m['dropped']:>8}", self.font, PANEL_FG),
            ("", self.font, PANEL_DIM),
            (f"throughput    {m['throughput_vph']:>8.0f} veh/h", self.font, PANEL_FG),
            (f"t. viaje      {m['avg_travel_time_s']:>8.1f} s", self.font, PANEL_FG),
            (f"t. espera     {m['avg_wait_time_s']:>8.1f} s", self.font, PANEL_FG),
            (f"vel. media    {m['avg_speed_ms']:>8.1f} m/s", self.font, PANEL_FG),
            (f"cola media    {m['avg_queue_len']:>8.1f}", self.font, PANEL_FG),
            (f"cola máxima   {m['max_queue_len']:>8}", self.font, PANEL_FG),
            (f"frenadas      {m['hard_brakes']:>8}", self.font, PANEL_FG),
            ("", self.font, PANEL_DIM),
            (f"render        {fps:>8.0f} fps", self.font, PANEL_DIM),
        ]
        probe = sim.probe
        if probe is not None:
            lines.append((f"prueba #{probe.id}  {probe.v:.1f} m/s", self.font, PROBE_COLOR))
        elif sim.metrics.probe_records:
            r = sim.metrics.probe_records[-1]
            lines.append(
                (f"prueba #{r.id}: {r.travel_time:.1f}s ({r.stopped_time:.1f}s parado)",
                 self.font, PROBE_COLOR)
            )

        pad = 12
        w = 268
        h = pad * 2 + sum(f.get_height() + 3 for _, f, _ in lines)
        panel = pygame.Surface((w, h))
        panel.set_alpha(232)
        panel.fill(PANEL_BG)
        y = pad
        for text, font, color in lines:
            if text:
                panel.blit(font.render(text, True, color), (pad, y))
            y += font.get_height() + 3
        screen.blit(panel, (12, 12))

        help_text = (
            "ESPACIO pausa · R reinicia · N paso · +/- velocidad · "
            "TAB escenario · D debug · E exporta CSV"
        )
        label = self.font.render(help_text, True, (70, 74, 84))
        screen.blit(label, (12, self.camera.height - label.get_height() - 10))

    # -- API -----------------------------------------------------------------

    def draw(self, screen: pygame.Surface, alpha: float, speed: float, fps: float) -> None:
        screen.blit(self._network_surface(), (0, 0))
        self._draw_stop_lines(screen)
        self._draw_vehicles(screen, alpha)
        if self.show_debug:
            self._draw_debug(screen)
        self._draw_panel(screen, speed, fps)

    def invalidate(self) -> None:
        """Fuerza el redibujado de la capa estática (al cambiar de escenario)."""
        self._static = None
        self._static_key = None


def _samples(length: float) -> int:
    return max(2, min(int(length / 1.5) + 2, 48))


def _angle_delta(a: float, b: float) -> float:
    """Diferencia angular más corta entre dos rumbos (evita el giro de 2π)."""
    d = (b - a + math.pi) % (2 * math.pi) - math.pi
    return d
