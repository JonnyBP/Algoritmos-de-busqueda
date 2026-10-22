import os
import heapq
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# Códigos de celda de los archivos de laberinto
WALL, START, GOAL = 3, 4, 5

# Movimientos permitidos (4 direcciones): arriba, abajo, izquierda, derecha.
MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# 1. CLASE NODE: representa cada estado/celda en el árbol de búsqueda
class Node:
    def __init__(self, position, parent=None, g=0, h=0, depth=0):
        self.position = position  # Tupla (fila, columna)
        self.parent = parent      # Nodo padre, para reconstruir la ruta
        self.g = g                # Coste real desde el inicio hasta este nodo
        self.h = h                # Estimación heurística hasta la meta
        self.f = g + h            # Evaluación total
        self.depth = depth        # Profundidad en el árbol de búsqueda


# 2. CLASE MAZE: carga y gestiona la matriz del laberinto
class Maze:
    def __init__(self, filepath):
        self.filepath = filepath
        self.filename = os.path.basename(filepath)
        self.grid = None
        self.rows = 0
        self.cols = 0
        self.start_pos = None
        self.goal_pos = None
        self._load()

    def _load(self):
        with open(self.filepath, 'r', encoding='utf-8') as f:
            lines = [line.strip() for line in f if line.strip()]

        grid_data = [[int(v) for v in line.split()] for line in lines[1:]]
        self.grid = np.array(grid_data, dtype=int)
        self.rows, self.cols = self.grid.shape

        # Si hay varios inicios/metas se usa el primero en orden de lectura (fila a fila).
        start_indices = np.argwhere(self.grid == START)
        if len(start_indices) > 0:
            self.start_pos = tuple(start_indices[0])

        goal_indices = np.argwhere(self.grid == GOAL)
        if len(goal_indices) > 0:
            self.goal_pos = tuple(goal_indices[0])

    def get_neighbors(self, pos):
        r, c = pos
        neighbors = []
        for dr, dc in MOVES:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols and self.grid[nr, nc] != WALL:
                neighbors.append((nr, nc))
        return neighbors


# Heurística: distancia Manhattan (admisible y consistente con movimientos de 4 direcciones)
def manhattan_distance(pos1, pos2):
    if pos1 is None or pos2 is None:
        return 0
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])


# 3. CLASE BASE BUSQUEDA
class SearchAlgorithm:
    def __init__(self, maze):
        self.maze = maze
        self.nodes_expanded = 0
        self.max_open_size = 0
        self.max_closed_size = 0

    def _has_endpoints(self):
        return self.maze.start_pos is not None and self.maze.goal_pos is not None

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        path.reverse()
        return path

    def _result(self, goal_node=None):
        """Devuelve (camino, métricas). Sin goal_node = sin solución."""
        metrics = {
            'nodes_expanded': self.nodes_expanded,
            'max_open': self.max_open_size,
            'max_closed': self.max_closed_size,
            'depth': None,
            'path_length': None,
            'solved': goal_node is not None,
        }
        if goal_node is None:
            return None, metrics
        path = self.reconstruct_path(goal_node)
        metrics['depth'] = goal_node.depth
        metrics['path_length'] = len(path)
        return path, metrics

    def _register_expansion(self, closed_size):
        self.nodes_expanded += 1
        self.max_closed_size = max(self.max_closed_size, closed_size)



# 4. ALGORITMOS
class DFS(SearchAlgorithm):
    """Búsqueda en Profundidad - OPEN es una pila (LIFO)."""

    def solve(self):
        if not self._has_endpoints():
            return self._result()

        open_stack = [Node(self.maze.start_pos)]
        closed_set = set()

        while open_stack:
            self.max_open_size = max(self.max_open_size, len(open_stack))
            current = open_stack.pop()
            if current.position in closed_set:
                continue

            closed_set.add(current.position)
            self._register_expansion(len(closed_set))

            if current.position == self.maze.goal_pos:
                return self._result(current)

            for nxt_pos in self.maze.get_neighbors(current.position):
                if nxt_pos not in closed_set:
                    open_stack.append(Node(nxt_pos, parent=current, depth=current.depth + 1))

        return self._result()


class BFS(SearchAlgorithm):
    """Búsqueda en Amplitud - OPEN es una cola (FIFO)"""

    def solve(self):
        if not self._has_endpoints():
            return self._result()

        open_queue = deque([Node(self.maze.start_pos)])
        closed_set = set()

        while open_queue:
            self.max_open_size = max(self.max_open_size, len(open_queue))
            current = open_queue.popleft()
            if current.position in closed_set:
                continue

            closed_set.add(current.position)
            self._register_expansion(len(closed_set))

            if current.position == self.maze.goal_pos:
                return self._result(current)

            for nxt_pos in self.maze.get_neighbors(current.position):
                if nxt_pos not in closed_set:
                    open_queue.append(Node(nxt_pos, parent=current, depth=current.depth + 1))

        return self._result()


class IDDFS(SearchAlgorithm):
    """Profundización Iterativa: DFS con límite de profundidad creciente."""

    def solve(self):
        if not self._has_endpoints():
            return self._result()

        limit = 0
        while True:
            goal_node, cutoff = self._dls(limit)
            if goal_node is not None:
                return self._result(goal_node)
            if not cutoff:  # espacio agotado: la meta no es alcanzable
                return self._result()
            limit += 1

    def _dls(self, limit):
        """Búsqueda en profundidad limitada. Devuelve (nodo_meta | None, hubo_corte)."""
        open_stack = [Node(self.maze.start_pos)]
        best_depth = {}  # posición -> menor profundidad con la que se expandió
        cutoff = False

        while open_stack:
            self.max_open_size = max(self.max_open_size, len(open_stack))
            current = open_stack.pop()

            if best_depth.get(current.position, float('inf')) <= current.depth:
                continue

            best_depth[current.position] = current.depth
            self._register_expansion(len(best_depth))

            if current.position == self.maze.goal_pos:
                return current, cutoff

            for nxt_pos in self.maze.get_neighbors(current.position):
                new_depth = current.depth + 1
                if best_depth.get(nxt_pos, float('inf')) <= new_depth:
                    continue
                if new_depth > limit:
                    cutoff = True  # hay ramas que el límite no deja explorar
                else:
                    open_stack.append(Node(nxt_pos, parent=current, depth=new_depth))

        return None, cutoff


class BestFirst(SearchAlgorithm):
    """Búsqueda Primero el Mejor (voraz) - OPEN ordenada por f(n) = h(n)."""

    def solve(self):
        if not self._has_endpoints():
            return self._result()

        goal = self.maze.goal_pos
        start_node = Node(self.maze.start_pos, h=manhattan_distance(self.maze.start_pos, goal))
        start_node.f = start_node.h

        counter = 0  # desempate estable: a igual h, sale antes el que entró primero
        open_pq = [(start_node.f, counter, start_node)]
        closed_set = set()

        while open_pq:
            self.max_open_size = max(self.max_open_size, len(open_pq))
            _, _, current = heapq.heappop(open_pq)
            if current.position in closed_set:
                continue

            closed_set.add(current.position)
            self._register_expansion(len(closed_set))

            if current.position == goal:
                return self._result(current)

            for nxt_pos in self.maze.get_neighbors(current.position):
                if nxt_pos not in closed_set:
                    h_val = manhattan_distance(nxt_pos, goal)
                    nxt_node = Node(nxt_pos, parent=current, h=h_val, depth=current.depth + 1)
                    nxt_node.f = h_val
                    counter += 1
                    heapq.heappush(open_pq, (nxt_node.f, counter, nxt_node))

        return self._result()


class AStar(SearchAlgorithm):
    """A* - OPEN ordenada por f(n) = g(n) + h(n)"""

    def solve(self):
        if not self._has_endpoints():
            return self._result()

        goal = self.maze.goal_pos
        start_node = Node(self.maze.start_pos, g=0, h=manhattan_distance(self.maze.start_pos, goal))

        counter = 0
        open_pq = [(start_node.f, start_node.h, counter, start_node)]
        closed_set = set()

        while open_pq:
            self.max_open_size = max(self.max_open_size, len(open_pq))
            _, _, _, current = heapq.heappop(open_pq)
            if current.position in closed_set:
                continue

            closed_set.add(current.position)
            self._register_expansion(len(closed_set))

            if current.position == goal:
                return self._result(current)

            for nxt_pos in self.maze.get_neighbors(current.position):
                if nxt_pos not in closed_set:
                    nxt_node = Node(nxt_pos, parent=current, g=current.g + 1,
                                    h=manhattan_distance(nxt_pos, goal), depth=current.depth + 1)
                    counter += 1
                    heapq.heappush(open_pq, (nxt_node.f, nxt_node.h, counter, nxt_node))

        return self._result()


# 5. VISUALIZADOR
class MazeVisualizer:
    # Valores de la rejilla de dibujo
    FREE, WALL_, PATH, START_, GOAL_ = 0, 1, 2, 3, 4
    COLORS = ['#FFFFFF', '#2C3E50', '#2ECC71', '#F1C40F', '#E74C3C']  # libre, muro, ruta, inicio, meta

    @staticmethod
    def render(maze, path=None, title="", save_path=None):
        V = MazeVisualizer
        vis_grid = np.where(maze.grid == WALL, V.WALL_, V.FREE)

        if path:
            for r, c in path:
                vis_grid[r, c] = V.PATH
        if maze.start_pos:
            vis_grid[maze.start_pos] = V.START_
        if maze.goal_pos:
            vis_grid[maze.goal_pos] = V.GOAL_

        plt.figure(figsize=(7, 7))
        plt.imshow(vis_grid, cmap=ListedColormap(V.COLORS), vmin=0, vmax=len(V.COLORS) - 1)
        plt.title(title, fontsize=12, fontweight='bold', pad=10)
        plt.xticks([])
        plt.yticks([])
        plt.tight_layout()

        if save_path:
            folder = os.path.dirname(save_path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close()
        else:
            plt.show()