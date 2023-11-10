# Simulador de Tráfico en Lima 🚗🚦

## Descripción 📝

Este proyecto es una simulación de tráfico en Lima que busca predecir patrones de congestión vehicular utilizando un modelo basado en autómatas celulares. El simulador servirá como una herramienta para la planificación urbana y la gestión del tráfico, permitiendo evaluar diferentes estrategias para mejorar la movilidad en la ciudad.

## Objetivos :dart:

El proyecto tiene dos objetivos principales:

1. **Generar Casos de Prueba**: Simular condiciones de tráfico variadas para generar una amplia gama de escenarios de prueba.
2. **Implementar Autómatas Celulares**: Utilizar autómatas celulares para modelar el comportamiento del tráfico de manera más realista.

## Requisitos 🛠️

### Para la Generación de Casos de Prueba:

- **Simulación de Diferentes Horas del Día**: La simulación debe reflejar condiciones de hora pico y no pico.
- **Inserción de Vehículo de Prueba**: Un vehículo de prueba se añadirá después de establecer el tráfico inicial para medir su tiempo de viaje.
- **Recolección y Análisis de Datos**: Registrar datos como tiempos de viaje y niveles de congestión para evaluar el rendimiento del sistema.

### Para la Implementación de Autómatas Celulares:

- **Espacio Celular de la Carretera**: Definir la carretera como una cuadrícula de celdas con estados "ocupado" o "libre".
- **Reglas de Transición**: Establecer reglas para el cambio de estado de las celdas basadas en sus vecinos.
- **Movimiento de Vehículos**: Adaptar el movimiento de los vehículos para seguir las reglas de los autómatas celulares.

## Estructura del Proyecto 🏗️

El proyecto está organizado de la siguiente manera:

- `main.py`: Script principal para configurar y ejecutar la simulación.
- `simulation.py`: Módulo que contiene la lógica central de la simulación.
- `vehicle.py`: Clase que define el comportamiento de los vehículos en la simulación.
- `road.py`: Clase que representa las carreteras dentro de la simulación.
- `traffic_signal.py`: Módulo para gestionar los semáforos y su impacto en el tráfico.
- `/geometry`: Directorio que contiene la lógica para modelar geometrías de carreteras.
- `window.py`: Módulo responsable de la visualización gráfica de la simulación.

## Ejecución de la Simulación 🚀

Para ejecutar la simulación, sigue estos pasos:

1. Asegúrate de tener todas las dependencias necesarias instaladas.
2. Ejecuta `main.py` para iniciar la simulación.
3. Observa la simulación y recopila los datos requeridos.

## Contribuciones 🤝

Si deseas contribuir al proyecto, por favor sigue estos pasos:

1. Clona el repositorio y crea una nueva rama para tu característica.
2. Desarrolla y prueba tu trabajo.
3. Envía un pull request con tus cambios.

## Licencia 📄

Este proyecto está licenciado bajo la Licencia MIT - ver el archivo [LICENSE.md](LICENSE.md) para detalles.

---

