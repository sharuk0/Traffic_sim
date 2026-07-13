# Simulador Microscópico de Tráfico Urbano

Simulación microscópica de intersecciones urbanas con **seguimiento vehicular continuo** (modelo tipo IDM), geometrías curvas parametrizadas por longitud de arco, semáforos como máquina de estados y una rotonda con aceptación de brecha. Escrito en Python 3.11+ con `pygame-ce`.

> **Nota de honestidad técnica.** Una versión anterior de este proyecto se describía como un modelo de **autómatas celulares**. No lo era: el código usaba posición, velocidad y aceleración continuas con una lógica de car-following. Esta versión conserva y corrige ese enfoque, y lo documenta por lo que realmente es: un **modelo microscópico continuo de seguimiento vehicular**. No hay ninguna cuadrícula de celdas ni reglas de transición discretas.

---

## 1. Motivación

El tráfico de Lima está dominado por intersecciones saturadas, giros a la izquierda sin protección, rotondas con reglas de prioridad poco respetadas y una mezcla vehicular heterogénea (autos, taxis, combis, buses). Este simulador es un banco de pruebas para razonar cuantitativamente sobre ese tipo de nodos: comparar planes semafóricos, medir el efecto de un giro protegido, o contrastar una intersección señalizada contra una rotonda bajo la misma demanda.

**Lo que este proyecto NO afirma:** no está calibrado contra aforos reales de Lima, no reproduce el comportamiento agresivo específico de la conducción limeña, y sus parámetros son valores razonables de la literatura, no medidos en campo. Es un modelo, no un gemelo digital.

---

## 2. El modelo de simulación

### 2.1 Seguimiento vehicular (IDM adaptado)

Cada vehículo actualiza su aceleración con el **Intelligent Driver Model** (Treiber, Hennecke & Helbing, 2000):

```
a = a_max · [ 1 − (v / v₀)^δ − (s* / s)² ]

s* = s₀ + max(0, v·T + v·Δv / (2·√(a_max · b_conf)))
```

| Símbolo | Significado | Unidad | Valor (auto) |
|---|---|---|---|
| `v₀` | velocidad deseada (mín. entre la del vehículo y el límite del carril) | m/s | 13.9 (50 km/h) |
| `s` | separación libre con el líder | m | — |
| `Δv` | `v − v_líder` (positivo si nos acercamos) | m/s | — |
| `s₀` | separación mínima en parada | m | 2.0 |
| `T` | headway temporal deseado | s | 1.4 |
| `a_max` | aceleración máxima | m/s² | 1.5 |
| `b_conf` | deceleración cómoda | m/s² | 2.0 |
| `b_max` | deceleración máxima física (satura la salida) | m/s² | 6.0 |
| `δ` | exponente de aceleración libre | — | 4 |

**Es una adaptación, no IDM puro.** Las diferencias están declaradas en `traffic_sim/car_following.py`:

1. La salida se **satura** a `[−b_max, a_max]`. El IDM canónico no acota la deceleración y produce frenados infinitos cuando `s → 0`.
2. La separación `s` tiene una cota inferior, lo que elimina la división entre cero.
3. Los **semáforos y el ceda-el-paso** se modelan como un *líder virtual detenido* en la línea de detención. Así el frenado ante un rojo usa la misma dinámica que ante un vehículo: es progresivo y anticipado.
4. Se añade un término de **anticipación del límite de velocidad**: al aproximarse a un carril más lento (una curva), el vehículo decelera con la cinemática exacta `a = (v_lim² − v²)/(2d)` en vez de sufrir un salto de velocidad al cruzar la frontera.

### 2.2 Integración

Euler semiimplícito, con la aceleración interviniendo **una sola vez** por paso:

```
a_n     = IDM(estado_n)
v_{n+1} = clamp(v_n + a_n·Δt, 0, v_max)
s_{n+1} = s_n + v_{n+1}·Δt
```

Con `Δt = 0.05 s` fijo. Garantiza: velocidad nunca negativa, nunca por encima del límite del carril, sin divisiones entre cero, sin aceleraciones infinitas.

### 2.3 Geometría continua

Un carril posee **una** geometría (`Line`, `Polyline`, `Arc` o `CubicBezier`) parametrizada por **longitud de arco**. El vehículo avanza `v·Δt` metros reales sobre la curva; su posición gráfica es `geometry.point_at(s)` y su orientación es la **tangente exacta** `geometry.tangent_at(s)`.

Un giro es **un solo carril**, no una cadena de segmentos rectos. Los giros se ven suaves porque lo son: no hay discretización que saltar.

### 2.4 Transferencia entre vías

El **mismo objeto `Vehicle`** continúa por su ruta (no hay `deepcopy`). Al cruzar el final de un carril, la **distancia sobrante se conserva** y se aplica sobre el siguiente:

```
sobrante = s − longitud(carril)
carril   = siguiente(ruta)
s        = sobrante
```

El bucle admite atravesar varios carriles cortos en un solo paso. El vehículo conserva ID, tiempo de creación, tiempo de espera, distancia, historial de carriles y métricas.

### 2.5 Continuidad entre carriles

El vehículo busca a su líder **aguas abajo de su ruta** (hasta 120 m): si no hay nadie delante en su carril, mira al vehículo más rezagado del carril siguiente. Por eso la restricción de seguimiento existe **antes** de que el vehículo cruce la frontera, y dos vehículos no pueden solaparse en un empalme.

Existe además una red de seguridad numérica (`overlap_fixes`) que corregiría un solapamiento si el modelo fallara. **En condiciones normales nunca se activa**; las pruebas verifican que el contador permanece en cero.

### 2.6 Semáforos

Máquina de estados configurable. Cada fase concede **VERDE** a un conjunto arbitrario de grupos, luego **ÁMBAR**, y termina con un intervalo de **TODO-ROJO** (despeje de la caja). El controlador soporta cualquier número de grupos y accesos, `offset` para coordinación, y duraciones por fase.

Ante un **ámbar**, el vehículo decide con la zona de dilema: si no puede detenerse frenando cómodamente, continúa.

### 2.7 Conflictos — qué se modela y qué no

Esta es la **simplificación más importante del proyecto** y conviene decirla claramente:

- **No hay un modelo general de puntos de conflicto** dentro de la intersección.
- Los conflictos se resuelven por **fases protegidas**: los movimientos que reciben verde simultáneo nunca se cruzan (los giros a la izquierda tienen su propia fase). La intersección es libre de conflictos *por construcción del plan semafórico*.
- En la **rotonda** se usa **aceptación de brecha** (gap acceptance).
- En `right_turn_only` no hace falta semáforo: los giros a la derecha de distintos accesos no se cruzan entre sí.

Un modelo general de zonas de conflicto (con reserva de puntos de cruce) sería la extensión natural, y está listada abajo.

### 2.8 Rotonda

Circulación **antihoraria** (tráfico por la derecha). Por cada acceso en el ángulo θ hay dos nodos sobre el anillo:

```
E_i = θ − 20°   nodo de salida (divergencia)
N_i = θ + 20°   nodo de entrada (convergencia)
```

Recorriendo el anillo en sentido antihorario se pasa **primero por la salida y después por la entrada**, como en la realidad. El anillo son 8 arcos exactos (`Arc`).

Un vehículo entra sólo si el **tiempo de llegada (TTA)** de todo vehículo con prioridad supera la **brecha crítica** (4.5 s por defecto). Una vez tomada la decisión a menos de 6 m de la línea, la maniobra se **compromete** y no se reevalúa (sin esto, la aceptación oscilaría entre pasos y produciría frenados de pánico).

La prioridad del anillo se expresa en la **decisión de entrar**, no en la negativa a frenar: si otro conductor ya invadió el punto de fusión, quien circula frena. Los vehículos no se detienen dentro del anillo salvo que el tráfico lo obligue.

**Aproximaciones declaradas:** brecha crítica homogénea entre conductores, sin modelar la aceleración del vehículo entrante durante la maniobra, prioridad absoluta al anillo. Además, el escenario incluye una **zona de deceleración** de 40 m antes de la línea de ceda-el-paso, porque los conductores reducen la velocidad al aproximarse a una rotonda aunque el anillo esté libre.

### 2.9 Generación de demanda

Cada vehículo generado entra en una **cola de espera por carril de acceso** y se inserta en cuanto hay hueco físico. **La demanda no se pierde** si el acceso está ocupado (el código original descartaba el vehículo y sorteaba otro, lo que sesgaba la distribución de rutas hacia los accesos libres). Sólo se descarta si la cola supera `max_queue`, y eso se contabiliza como `dropped`.

Dos procesos de llegada seleccionables:

- **Poisson** (por defecto): headways exponenciales. Realista para llegadas urbanas no coordinadas; introduce ráfagas y colas.
- **Determinista**: headways constantes. Útil para depurar y comparar planes semafóricos sin ruido estadístico.

Todo el azar proviene de un único `random.Random(seed)`: **misma semilla ⇒ misma ejecución, bit a bit**.

La **hora pico** se elige explícitamente (`--period peak`), multiplicando la demanda por `peak_factor`. **No depende del reloj del sistema** (el código original leía `datetime.now()`, y además descartaba el resultado sin usarlo).

---

## 3. Arquitectura

```
traffic_sim/
    config.py             Dataclasses tipadas: VehicleType, DemandConfig, SignalPlanConfig, SimulationConfig
    geometry.py           Line, Polyline, Arc, CubicBezier → point_at(s), tangent_at(s), length
    network.py            Lane, Route, Network + validación estructural
    car_following.py      IDM adaptado, con clamps y zona de dilema
    vehicle.py            Agente con identidad estable y métricas propias
    signals.py            Máquina de estados semafórica
    generator.py          Cola de espera, Poisson/determinista, semilla
    metrics.py            Métricas por instancia + export CSV + vehículo de prueba
    simulation.py         Bucle de paso fijo, líder inter-carril, transferencia con sobrante
    scenarios/
        base.py           ArmSpec + build_intersection (constructor genérico paramétrico)
        intersections.py  four_way, left_turn_only, right_turn_only, t_junction
        roundabout.py     Rotonda con anillo de arcos y aceptación de brecha
    rendering/
        camera.py         Zoom, paneo, conversión mundo↔pantalla (inversas exactas)
        renderer.py       Dibujo (sin lógica de dominio); red estática cacheada
        window.py         Bucle de tiempo fijo con acumulador e interpolación
scripts/generate_test_cases.py
tests/
main.py
```

### 3.1 Rutas: sin índices numéricos

Una ruta es una secuencia de **IDs legibles**:

```python
Route(id="W0->E", lanes=("W_in_0", "W0_E0_s", "E_out_0"), ...)
```

`Network.validate()` se ejecuta al construir el escenario y **rechaza con un error explícito**:

- carriles inexistentes (`RouteError`),
- pares de carriles no conectados en el grafo (`RouteError: 'a' no conecta con 'b'`),
- discontinuidad geométrica > 0.5 m entre el fin de un carril y el inicio del siguiente (`NetworkError`),
- rutas que no terminan en una salida válida,
- pesos de ruta inválidos.

### 3.2 Escenarios: sin coordenadas duplicadas

Un escenario **declara sus accesos**; la geometría se deriva:

```python
arms = [
    ArmSpec("W", 180.0, in_lanes=2, out_lanes=2, movements={0: {STRAIGHT, RIGHT}, 1: {LEFT}}),
    ArmSpec("N",  90.0, ...),
    ...
]
lanes, routes = build_intersection(arms)
```

Añadir una intersección en T es declarar **tres** accesos en lugar de cuatro. Los movimientos no permitidos simplemente no generan conectores ni rutas: es imposible generar una ruta prohibida.

### 3.3 Simulación y render desacoplados

```python
accumulator += frame_time · speed
while accumulator >= fixed_dt:
    sim.step()
    accumulator -= fixed_dt
alpha = accumulator / fixed_dt
renderer.draw(alpha)     # interpola entre prev_pose y pose
```

La velocidad de la simulación **no depende del framerate**. El movimiento se ve fluido aunque el FPS fluctúe. El número de pasos por cuadro puede ser 0, 1 o varios, con un tope que evita la "espiral de la muerte".

---

## 4. Escenarios

| Comando | Descripción |
|---|---|
| `--scenario four_way` | 4 accesos, 2 carriles/sentido. Carril derecho: recto + derecha. Carril izquierdo: giro a la izquierda **protegido**. Plan de 4 fases (W+E recto, W+E izquierda, N+S recto, N+S izquierda). |
| `--scenario roundabout` | Rotonda de 4 accesos, anillo continuo, prioridad al anillo con aceptación de brecha. |
| `--scenario left_turn_only` | Sólo giros a la izquierda. Dos izquierdas opuestas **sí** se cruzan, así que hay una fase protegida por acceso. |
| `--scenario right_turn_only` | Sólo giros a la derecha. **Sin semáforo**: los movimientos no son conflictivos. Flujo continuo. |
| `--scenario t_junction` | Intersección en T (3 accesos). Demuestra la extensibilidad del constructor. |

---

## 5. Instalación

```bash
git clone <repo> && cd traffic-sim-lima
python3 -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Requiere **Python 3.11+** (se usan `StrEnum` y sintaxis moderna de tipos).

## 6. Ejecución

```bash
python main.py --scenario four_way
python main.py --scenario roundabout --seed 7
python main.py --scenario left_turn_only --period peak --vph 900
python main.py --scenario right_turn_only

# Modo headless (sin ventana), con métricas y export a CSV
python main.py --scenario four_way --headless --duration 900 --export output/

# Llegadas deterministas en vez de Poisson
python main.py --scenario four_way --arrival deterministic
```

Opciones: `--seed`, `--vph`, `--period {off_peak,peak}`, `--arrival {poisson,deterministic}`, `--warmup`, `--probe-route`, `--probe-time`, `--headless`, `--duration`, `--export`, `--verbose`.

## 7. Controles

| Tecla | Acción |
|---|---|
| `ESPACIO` | Pausar / continuar |
| `R` | Reiniciar la simulación |
| `N` | Avanzar un solo paso |
| `+` / `-` | Aumentar / reducir la velocidad (0.25× … 16×) |
| `TAB` | Cambiar de escenario |
| `D` | Mostrar/ocultar información de depuración (IDs de carril, velocidades) |
| `E` | Exportar métricas a CSV |
| `Q` / `ESC` | Salir |
| Rueda del ratón | Zoom (anclado al cursor) |
| Arrastrar | Desplazar la cámara |

## 8. Métricas

Se recolectan **por instancia** (no hay contadores globales) y **excluyen el periodo de calentamiento**:

- Vehículos generados, ingresados, presentes, en cola, descartados, completados
- Throughput (veh/h)
- Tiempo promedio de viaje y de espera
- Velocidad promedio
- Longitud media y máxima de cola
- Número de frenadas fuertes (`a ≤ −3 m/s²`)
- **Desagregación por ruta** (origen → destino)

Export a CSV con `E` en la ventana o `--export DIR`: genera `*_vehicles.csv` (una fila por vehículo), `*_routes.csv` y `*_summary.csv`.

### Vehículo de prueba

Tras el calentamiento se inyecta un **vehículo de prueba** (dibujado en magenta) y se registra su tiempo de ingreso, de salida, tiempo total de viaje, tiempo detenido y ruta utilizada.

```bash
python main.py --scenario four_way --headless --duration 600 --probe-route "W0->E"
```

### Sobre `test_cases.csv` (el archivo heredado)

El `test_cases.csv` del proyecto original **no puede atribuirse a este simulador**, y no se inventa su procedencia:

| Evidencia | Hallazgo |
|---|---|
| Código que lo lea o escriba | No existe en el proyecto original |
| Coordenadas | `start_x ∈ [19, 1982]`, `start_y ∈ [11, 1918]`, con el mundo simulado en `[−312, 312]²` |
| Destino | Siempre `(1000, 0)`, que no es un nodo de la intersección |
| `traffic_density` | Continua en `[51, 297]`, mientras el código sólo podía producir 100 o 200 |
| Columna `hour` | Sin relación con los rangos de hora pico definidos en el código |
| Correlación densidad ↔ tiempo | `r = 0.752` — coherente con datos sintéticos de una fórmula con ruido |

Se conserva en `legacy/test_cases_legacy.csv` como referencia histórica. En su lugar existe un **mecanismo reproducible**:

```bash
python scripts/generate_test_cases.py --out output/test_cases.csv --seeds 10
```

Cada fila queda completamente determinada por `(escenario, periodo, demanda, semilla)`: reejecutar el script con los mismos argumentos produce **exactamente** el mismo CSV.

## 9. Pruebas

```bash
python -m pytest tests/ -v
```

**56 pruebas**, incluyendo los 12 requisitos exigidos:

| # | Requisito | Prueba |
|---|---|---|
| 1 | Velocidad nunca negativa | `test_speed_is_never_negative_when_braking_hard`, `test_speed_never_negative_across_all_scenarios` |
| 2 | Dos vehículos no se superponen | `test_vehicles_never_overlap_in_the_same_lane` (verifica además que `overlap_fixes == 0`) |
| 3 | Ruta desconectada rechazada | `test_disconnected_route_is_rejected`, `test_geometric_discontinuity_is_rejected` |
| 4 | Identidad al cambiar de vía | `test_vehicle_keeps_identity_across_lanes` |
| 5 | Distancia sobrante conservada | `test_leftover_distance_is_preserved_on_lane_change`, `test_leftover_across_multiple_short_lanes` |
| 6 | Luz roja detiene antes de la línea | `test_red_light_stops_the_vehicle_before_the_stop_line` |
| 7 | Luz verde permite continuar | `test_green_light_lets_the_vehicle_through` |
| 8 | Reproducible con la misma semilla | `test_same_seed_gives_identical_runs` (los 5 escenarios) |
| 9 | Prioridad en la rotonda | `test_ring_traffic_has_priority_over_entering_vehicles` |
| 10 | Cada escenario corre minutos sin excepciones | `test_each_scenario_runs_for_minutes_without_exceptions` (10 min simulados c/u) |
| 11 | Todas las rutas terminan en salida válida | `test_all_routes_end_in_a_valid_exit`, `test_turn_only_scenarios_generate_no_forbidden_routes` |
| 12 | Métricas se reinician | `test_metrics_reset_with_a_new_simulation`, `test_reset_clears_state_in_place` |

Más: geometría (parametrización por longitud de arco, continuidad G1 de los conectores), cámara (`to_world` es la inversa exacta de `to_screen`), conservación de la demanda en cola, y un **benchmark** (`tests/test_benchmark.py`) con un umbral deliberadamente laxo, que no exige resultados dependientes del hardware. En una máquina de referencia: ~4 800 pasos/s ≈ **240× tiempo real** con ~26 vehículos en red.

Lint: `ruff check .` (limpio).

## 10. Limitaciones

Dichas sin rodeos:

- **No hay modelo general de conflictos** dentro de la intersección: se depende de fases protegidas y de la aceptación de brecha en la rotonda (§2.7).
- **No hay cambio de carril.** Cada vehículo elige su carril al entrar y lo mantiene hasta su conector.
- **Sin peatones, ciclistas, ni paraderos.** Los buses son sólo vehículos largos y lentos, no hacen paradas.
- **Parámetros no calibrados** contra aforos reales de Lima.
- La brecha crítica de la rotonda es **homogénea**; no hay heterogeneidad de conductores.
- La rotonda registra más frenadas fuertes que los escenarios señalizados: la decisión de ceder es binaria y se reevalúa en cada paso, lo que produce reacciones más decididas que las de un conductor real que anticipa de forma continua.
- Sin coordinación semafórica en red (existe `offset`, pero hay un solo nodo por escenario).

## 11. Posibles extensiones

- Zonas de conflicto explícitas con reserva de puntos de cruce (permitiría giros a la izquierda *permitidos* con precaución, no sólo protegidos).
- Cambio de carril (MOBIL).
- Optimización del plan semafórico: búsqueda de duraciones de fase que minimicen la demora media (el simulador ya devuelve una función objetivo evaluable).
- Cargador de escenarios desde YAML/TOML (la arquitectura ya está preparada: los escenarios son dataclasses puras).
- Heterogeneidad de conductores (brecha crítica, `T` y `a_max` muestreados de una distribución).
- Redes de varios nodos con coordinación semafórica (onda verde).

## 12. Capturas / GIF

```bash
# Ejecutar y grabar (Linux, con ffmpeg)
python main.py --scenario roundabout &
ffmpeg -f x11grab -framerate 30 -video_size 1280x800 -i :0.0 -t 15 demo.mp4
ffmpeg -i demo.mp4 -vf "fps=15,scale=800:-1" -loop 0 docs/roundabout.gif
```

Sugerencia: grabar `four_way` (colas y fases protegidas), `roundabout` (prioridad al anillo) y `left_turn_only` con `D` activado para mostrar los IDs de carril.

## 13. Tecnologías

- **Python 3.11+** — `StrEnum`, dataclasses, type hints en todo el código
- **pygame-ce 2.5+** — visualización
- **pytest** — 56 pruebas
- **ruff** — linting (PEP 8)

Sin NumPy ni SciPy: la distancia euclidiana es `math.hypot`, y las geometrías se resuelven con `math` y `bisect`. Se eliminó SciPy, que el proyecto original importaba entero para calcular una distancia entre dos puntos.

## 14. Relevancia académica

Este proyecto demuestra experiencia en:

- **Modelamiento matemático** — formulación de un sistema dinámico continuo de agentes acoplados (car-following), con parámetros interpretables y comportamiento acotado (velocidad no negativa, aceleración saturada, sin singularidades).
- **Simulación de sistemas dinámicos discretizados** — elección justificada del esquema de integración (Euler semiimplícito), análisis de estabilidad con `dt` pequeño, y separación explícita entre el paso de integración y el de renderizado mediante un acumulador de tiempo fijo.
- **Geometría computacional** — parametrización por longitud de arco de curvas de Bézier y arcos, con tablas de longitud acumulada y búsqueda binaria; continuidad G1 en los empalmes.
- **Diseño algorítmico y optimización computacional** — eliminación de copias profundas en el camino caliente, búsqueda de líder en O(1) amortizado, cacheo de la geometría estática; ~240× tiempo real en un solo hilo.
- **Diseño de experimentos** — ejecuciones deterministas parametrizadas por semilla, periodo de calentamiento, y un vehículo de prueba inyectado tras el régimen transitorio.
- **Análisis de datos** — instrumentación de métricas (throughput, demora, colas, frenadas fuertes) desagregadas por ruta y exportables a CSV para su análisis posterior.

**Este proyecto no utiliza computación cuántica**, ni machine learning, ni ningún componente que no esté justificado por el problema. Es un ejercicio de modelamiento y simulación clásica, hecho con cuidado.

## 15. Licencia

MIT. Ver [LICENSE.md](LICENSE.md).
