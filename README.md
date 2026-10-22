# Algoritmos de búsqueda en laberintos

Comparación experimental de cinco algoritmos de búsqueda en grafos aplicados a laberintos en cuadrícula.

## ¿Qué hace y para qué sirve?

Cada laberinto se modela como un grafo: las celdas libres son nodos y los movimientos arriba/abajo/izquierda/derecha son aristas de coste 1. El programa resuelve cada laberinto con cinco algoritmos, mide su comportamiento y guarda una imagen del camino encontrado.

El objetivo es **entender y comparar** estrategias de búsqueda: cuántos nodos expanden, cuánta memoria usan (listas OPEN/CLOSED) y si el camino que devuelven es el más corto.

| Algoritmo | Tipo | Estructura de OPEN | Óptimo* |
|---|---|---|---|
| `DF` (Profundidad) | No informado | Pila (LIFO) | No |
| `BF` (Amplitud) | No informado | Cola (FIFO) | Sí |
| `IDDF` (Profundización iterativa) | No informado | Pila + límite de profundidad creciente | Sí |
| `Best_First` (Voraz) | Informado, `f(n) = h(n)` | Cola de prioridad | No |
| `A_Star` | Informado, `f(n) = g(n) + h(n)` | Cola de prioridad | Sí |

\* Con costes unitarios. La heurística usada es la **distancia Manhattan**, que es admisible y consistente en una cuadrícula de 4 direcciones, por lo que A* garantiza el camino mínimo.

## Estructura del repositorio

```
.
├── solver.py                  # Clases Node, Maze, algoritmos y MazeVisualizer
├── run_all.py                 # Ejecuta todos los laberintos x todos los algoritmos
├── Laberintos/                # Archivos maze_*.txt de entrada (incluye casos límite)
├── snapshots/                 # Imágenes PNG generadas (una por laberinto y algoritmo)
└── resultados_busqueda.csv    # Métricas generadas
```

## Requisitos

- Python 3.9 o superior
- `numpy`, `pandas`, `matplotlib`

```bash
pip install numpy pandas matplotlib
```

## Uso

1. Coloca los laberintos en `Laberintos/`. Se procesan todos los archivos cuyo nombre empiece por `maze_` y termine en `.txt`.
2. Ejecuta:

```bash
python run_all.py
```

Opciones: `--mazes-dir`, `--snapshots-dir`, `--output` (ver `python run_all.py --help`).

Se generan:

- `resultados_busqueda.csv`: una fila por combinación laberinto/algoritmo.
- `snapshots/<laberinto>_<algoritmo>.png`: el camino encontrado dibujado sobre el laberinto.

### Usar un algoritmo de forma individual

```python
from solver import Maze, AStar, MazeVisualizer

maze = Maze("Laberintos/maze_1.txt")
path, metrics = AStar(maze).solve()
print(metrics)
MazeVisualizer.render(maze, path, title="maze_1 | A*")   # sin save_path abre una ventana
```

## Formato de los laberintos

Archivo de texto: la primera línea contiene `filas columnas` y el resto es la matriz con valores separados por espacios.

| Valor | Significado |
|---|---|
| `3` | Muro |
| `4` | Inicio |
| `5` | Meta |
| cualquier otro (p. ej. `0`) | Celda libre |

- Si el laberinto no tiene inicio o meta, los algoritmos devuelven "sin solución" con 0 nodos expandidos, sin entrar en el bucle de búsqueda.
- Si hay varias celdas `5`, se usa la primera en orden de lectura (fila a fila); las demás se tratan como celdas libres.
- Si la meta existe pero es inalcanzable (encerrada por muros), la búsqueda termina cuando OPEN se vacía y devuelve "sin solución".

## Métricas del CSV

| Columna | Descripción |
|---|---|
| `Resuelto` | Si se encontró un camino |
| `Nodos expandidos` | Número de nodos expandidos (coste computacional) |
| `Profundidad Sol.` | Número de movimientos del camino |
| `Long. Camino` | Número de celdas del camino (= profundidad + 1) |
| `Max OPEN` | Tamaño máximo de la lista OPEN (incluye entradas duplicadas pendientes) |
| `Max CLOSED` | Tamaño máximo del conjunto de nodos ya expandidos |
| `Heurística` | `Manhattan` para Best-First y A*, `N/A` en el resto |

## Resultados (promedio sobre los 10 laberintos resolubles)

| Algoritmo | Nodos expandidos | Long. camino (celdas) | Max OPEN | % caminos mínimos |
|---|---|---|---|---|
| A* | 93.8 | 57.7 | 15.6 | 100% |
| BF | 154.3 | 57.7 | 12.2 | 100% |
| Best-First | 87.2 | 67.1 | 15.7 | 40% |
| DF | 132.8 | 99.9 | 51.0 | 10% |
| IDDF | 9041.6 | 57.7 | 15.5 | 100% |

Los laberintos sin meta o con meta inalcanzable se excluyen del promedio. Detalle por laberinto en `resultados_busqueda.csv`.

## Notas de diseño

- **IDDF** guarda la menor profundidad con la que alcanzó cada nodo y lo re-expande si llega por un camino más corto; con un CLOSED simple devolvería caminos no mínimos.
- **A\*** desempata por menor `h` a igual `f`. Sin ello, en espacios abiertos expande casi todo el mapa (como BF).
- **Max OPEN** cuenta entradas duplicadas pendientes (se descartan al extraerlas), por lo que es un límite superior del tamaño real de la frontera.
- El orden de vecinos (`MOVES`) condiciona el camino que encuentra DF: la pila prueba primero derecha, luego izquierda, abajo y arriba.
