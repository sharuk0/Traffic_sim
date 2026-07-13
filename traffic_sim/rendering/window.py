"""Ventana de pygame con bucle de tiempo fijo y renderizado interpolado.

La simulación y el render están DESACOPLADOS:

    accumulator += frame_time · speed
    while accumulator >= fixed_dt:
        sim.step()
        accumulator -= fixed_dt
    alpha = accumulator / fixed_dt
    renderer.draw(alpha)     # interpola entre el estado anterior y el actual

Consecuencias: la velocidad de la simulación NO depende del framerate (el proyecto
original ejecutaba `steps_per_update` pasos por frame, así que una máquina lenta
simulaba más despacio), y el movimiento se ve fluido aunque el FPS fluctúe. El número
de pasos por cuadro puede ser 0, 1 o varios; se acota `max_steps_per_frame` para que la
ventana nunca se congele intentando alcanzar el tiempo real ("spiral of death").
"""

from __future__ import annotations

import logging
from pathlib import Path

import pygame

from ..config import SimulationConfig
from ..scenarios import SCENARIOS, build
from ..simulation import Simulation
from .camera import Camera
from .renderer import Renderer

logger = logging.getLogger(__name__)

SPEEDS: tuple[float, ...] = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0)
MAX_STEPS_PER_FRAME: int = 400


class Window:
    """Ventana interactiva. Sólo gestiona entrada, tiempo y dibujo."""

    def __init__(
        self,
        sim: Simulation,
        config: SimulationConfig,
        width: int = 1280,
        height: int = 800,
        fps: int = 60,
        export_dir: str | Path = "output",
    ) -> None:
        pygame.init()
        pygame.display.set_caption(f"Simulador de Tráfico · {sim.scenario.name}")
        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        self.sim = sim
        self.config = config
        self.fps = fps
        self.export_dir = Path(export_dir)
        self.camera = Camera(width, height, zoom=3.2)
        self.renderer = Renderer(sim, self.camera)

        self._speed_index = SPEEDS.index(1.0)
        self._accumulator = 0.0
        self._dragging = False
        self._drag_origin = (0, 0)
        self._scenario_names = sorted(SCENARIOS)
        self._scenario_index = self._scenario_names.index(sim.scenario.name)
        self._single_step = False

    @property
    def speed(self) -> float:
        return SPEEDS[self._speed_index]

    # -- bucle principal -----------------------------------------------------

    def run(self) -> None:
        running = True
        while running:
            frame_time = min(self.clock.tick(self.fps) / 1000.0, 0.25)
            running = self._handle_events()

            if self._single_step:
                self.sim.step()
                self._single_step = False
                self._accumulator = 0.0
            elif not self.sim.paused:
                self._accumulator += frame_time * self.speed
                steps = 0
                while self._accumulator >= self.config.fixed_dt and steps < MAX_STEPS_PER_FRAME:
                    self.sim.step()
                    self._accumulator -= self.config.fixed_dt
                    steps += 1
                if steps == MAX_STEPS_PER_FRAME:
                    self._accumulator = 0.0  # no intentar recuperar el tiempo perdido

            alpha = 0.0 if self.sim.paused else self._accumulator / self.config.fixed_dt
            self.renderer.draw(self.screen, min(alpha, 1.0), self.speed, self.clock.get_fps())
            pygame.display.flip()

        pygame.quit()

    # -- eventos -------------------------------------------------------------

    def _handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.VIDEORESIZE:
                self.camera.resize(event.w, event.h)
                self.renderer.invalidate()
            elif event.type == pygame.KEYDOWN:
                if not self._handle_key(event.key):
                    return False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self._dragging = True
                self._drag_origin = event.pos
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self._dragging = False
            elif event.type == pygame.MOUSEMOTION and self._dragging:
                dx = event.pos[0] - self._drag_origin[0]
                dy = event.pos[1] - self._drag_origin[1]
                self.camera.pan_pixels(dx, dy)
                self._drag_origin = event.pos
            elif event.type == pygame.MOUSEWHEEL:
                factor = 1.12 ** event.y
                self.camera.zoom_at(factor, pygame.mouse.get_pos())
        return True

    def _handle_key(self, key: int) -> bool:
        if key in (pygame.K_ESCAPE, pygame.K_q):
            return False
        if key == pygame.K_SPACE:
            self.sim.paused = not self.sim.paused
        elif key == pygame.K_r:
            self.sim.reset()
            self._accumulator = 0.0
        elif key == pygame.K_n:
            self.sim.paused = True
            self._single_step = True
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self._speed_index = min(self._speed_index + 1, len(SPEEDS) - 1)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self._speed_index = max(self._speed_index - 1, 0)
        elif key == pygame.K_TAB:
            self._next_scenario()
        elif key == pygame.K_d:
            self.renderer.show_debug = not self.renderer.show_debug
        elif key == pygame.K_e:
            paths = self.sim.metrics.export_csv(self.export_dir, prefix=self.sim.scenario.name)
            logger.info("Métricas exportadas: %s", ", ".join(str(p) for p in paths))
            print(f"Métricas exportadas en {self.export_dir}/")
        return True

    def _next_scenario(self) -> None:
        self._scenario_index = (self._scenario_index + 1) % len(self._scenario_names)
        name = self._scenario_names[self._scenario_index]
        scenario = build(name)
        config = SimulationConfig(
            fixed_dt=self.config.fixed_dt,
            seed=self.config.seed,
            warmup=self.config.warmup,
            demand=scenario.default_demand,
            probe_route=scenario.default_probe_route,
            probe_time=self.config.probe_time,
            critical_gap=self.config.critical_gap,
        )
        self.config = config
        self.sim = Simulation(scenario, config)
        self.renderer = Renderer(self.sim, self.camera)
        self._accumulator = 0.0
        pygame.display.set_caption(f"Simulador de Tráfico · {name}")
