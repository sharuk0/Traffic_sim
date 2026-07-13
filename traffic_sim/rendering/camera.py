"""Cámara 2D: zoom, desplazamiento y conversión mundo ↔ pantalla.

`to_world` es la inversa EXACTA de `to_screen` (el proyecto original tenía un
`inverse_convert` que llamaba internamente a `convert`, devolviendo la transformación
equivocada, y truncaba a entero perdiendo precisión).

El eje Y del mundo apunta al norte; el de la pantalla, hacia abajo: la cámara invierte
el signo, de modo que el norte se dibuja arriba.
"""

from __future__ import annotations

from dataclasses import dataclass

Point = tuple[float, float]

MIN_ZOOM: float = 0.8
MAX_ZOOM: float = 30.0


@dataclass
class Camera:
    """Transformación afín entre coordenadas del mundo (m) y de pantalla (px)."""

    width: int
    height: int
    zoom: float = 3.2
    center: Point = (0.0, 0.0)
    """Punto del mundo que aparece en el centro de la pantalla."""

    def to_screen(self, point: Point) -> tuple[float, float]:
        x = (point[0] - self.center[0]) * self.zoom + self.width / 2.0
        y = -(point[1] - self.center[1]) * self.zoom + self.height / 2.0
        return (x, y)

    def to_world(self, pixel: Point) -> Point:
        x = (pixel[0] - self.width / 2.0) / self.zoom + self.center[0]
        y = -(pixel[1] - self.height / 2.0) / self.zoom + self.center[1]
        return (x, y)

    def pan_pixels(self, dx: float, dy: float) -> None:
        """Desplaza la cámara según un arrastre en píxeles."""
        self.center = (self.center[0] - dx / self.zoom, self.center[1] + dy / self.zoom)

    def zoom_at(self, factor: float, pixel: Point) -> None:
        """Aplica zoom manteniendo fijo el punto del mundo bajo el cursor."""
        anchor = self.to_world(pixel)
        self.zoom = max(MIN_ZOOM, min(self.zoom * factor, MAX_ZOOM))
        after = self.to_world(pixel)
        self.center = (
            self.center[0] + (anchor[0] - after[0]),
            self.center[1] + (anchor[1] - after[1]),
        )

    def scale(self, metres: float) -> float:
        """Convierte una longitud del mundo a píxeles."""
        return metres * self.zoom

    def resize(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
