import os
import math
import heapq
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# 1. CLASE NODE: Representa cada estado/celda en el árbol de búsqueda
class Node:
    def __init__(self, position, parent=None, g=0, h=0, depth=0):
        self.position = position  # Tupla (fila, columna)
        self.parent = parent      # Referencia al nodo padre para reconstruir la ruta
        self.g = g                # Coste real desde el inicio hasta este nodo
        self.h = h                # Estimación heurística hasta la meta
        self.f = g + h            # Evaluación total
        self.depth = depth        # Profundidad en el árbol de búsqueda

    def __lt__(self, other):
        return self.f < other.f


# 2. CLASE MAZE: Carga y gestiona la matriz del laberinto
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
        
        # Lectura de la cabecera: filas columnas
        header = lines[0].split()
        
        grid_data = []
        for line in lines[1:]:
            row_vals = [int(v) for v in line.split()]
            if row_vals:
                grid_data.append(row_vals)
        
        self.grid = np.array(grid_data, dtype=int)
        self.rows, self.cols = self.grid.shape
        
        # Localizar Inicio (4) y Meta (5)
        start_indices = np.argwhere(self.grid == 4)
        if len(start_indices) > 0:
            self.start_pos = tuple(start_indices[0])
            
        goal_indices = np.argwhere(self.grid == 5)
        if len(goal_indices) > 0:
            self.goal_pos = tuple(goal_indices[0])

    def get_neighbors(self, pos):
        r, c = pos
        # Movimientos permitidos: Arriba, Abajo, Izquierda, Derecha
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.grid[nr, nc] != 3:  # 3 representa un muro
                    neighbors.append((nr, nc))
        return neighbors

# Función Heurística (4 direcciones - manhattan)
def manhattan_distance(pos1, pos2):
    if pos1 is None or pos2 is None:
        return 0
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])







# 3. CLASE BASE SEARCH ALGORITHM
class SearchAlgorithm:
    def __init__(self, maze):
        self.maze = maze
        self.nodes_expanded = 0
        self.max_open_size = 0
        self.max_closed_size = 0

    def reconstruct_path(self, node):
        path = []
        curr = node
        while curr:
            path.append(curr.position)
            curr = curr.parent
        path.reverse()
        return path


# 4. IMPLEMENTACIÓN DE CADA ALGORITMO
class DFS(SearchAlgorithm):
    """Búsqueda en Profundidad (Depth-First Search) - Uso de Pila (LIFO)"""
    
    def solve(self):
        if not self.maze.start_pos or not self.maze.goal_pos:
            return None, {'nodes_expanded': 0, 'max_open': 0, 'max_closed': 0, 'depth': None, 'path_length': None, 'solved': False}
        
        start_node = Node(self.maze.start_pos)
        open_stack = [start_node]
        closed_set = set()
        
        while open_stack:
            self.max_open_size = max(self.max_open_size, len(open_stack))
            current = open_stack.pop()
            
            if current.position in closed_set:
                continue
            
            closed_set.add(current.position)
            self.nodes_expanded += 1
            self.max_closed_size = max(self.max_closed_size, len(closed_set))
            
            if current.position == self.maze.goal_pos:
                path = self.reconstruct_path(current)
                return path, {
                    'nodes_expanded': self.nodes_expanded,
                    'max_open': self.max_open_size,
                    'max_closed': self.max_closed_size,
                    'depth': current.depth,
                    'path_length': len(path),
                    'solved': True
                }
            
            for nxt_pos in self.maze.get_neighbors(current.position):
                if nxt_pos not in closed_set:
                    nxt_node = Node(nxt_pos, parent=current, depth=current.depth + 1)
                    open_stack.append(nxt_node)
                    
        return None, {'nodes_expanded': self.nodes_expanded, 'max_open': self.max_open_size, 'max_closed': self.max_closed_size, 'depth': None, 'path_length': None, 'solved': False}


class BFS(SearchAlgorithm):
    """Búsqueda en Amplitud (Breadth-First Search) - Uso de Cola (FIFO)"""
    def solve(self):
        if not self.maze.start_pos or not self.maze.goal_pos:
            return None, {'nodes_expanded': 0, 'max_open': 0, 'max_closed': 0, 'depth': None, 'path_length': None, 'solved': False}
        
        start_node = Node(self.maze.start_pos)
        open_queue = deque([start_node])
        closed_set = set()
        
        while open_queue:
            self.max_open_size = max(self.max_open_size, len(open_queue))
            current = open_queue.popleft()
            
            if current.position in closed_set:
                continue
            
            closed_set.add(current.position)
            self.nodes_expanded += 1
            self.max_closed_size = max(self.max_closed_size, len(closed_set))
            
            if current.position == self.maze.goal_pos:
                path = self.reconstruct_path(current)
                return path, {
                    'nodes_expanded': self.nodes_expanded,
                    'max_open': self.max_open_size,
                    'max_closed': self.max_closed_size,
                    'depth': current.depth,
                    'path_length': len(path),
                    'solved': True
                }
            
            for nxt_pos in self.maze.get_neighbors(current.position):
                if nxt_pos not in closed_set:
                    nxt_node = Node(nxt_pos, parent=current, depth=current.depth + 1)
                    open_queue.append(nxt_node)
                    
        return None, {'nodes_expanded': self.nodes_expanded, 'max_open': self.max_open_size, 'max_closed': self.max_closed_size, 'depth': None, 'path_length': None, 'solved': False}


class IDDFS(SearchAlgorithm):
    """Profundización Iterativa (Iterative Deepening Depth-First Search)"""
    def solve(self, max_depth_limit=500):
        if not self.maze.start_pos or not self.maze.goal_pos:
            return None, {'nodes_expanded': 0, 'max_open': 0, 'max_closed': 0, 'depth': None, 'path_length': None, 'solved': False}
        
        for limit in range(max_depth_limit):
            path, found = self._dls(self.maze.start_pos, limit)
            if found:
                return path, {
                    'nodes_expanded': self.nodes_expanded,
                    'max_open': self.max_open_size,
                    'max_closed': self.max_closed_size,
                    'depth': len(path) - 1,
                    'path_length': len(path),
                    'solved': True
                }
        return None, {'nodes_expanded': self.nodes_expanded, 'max_open': self.max_open_size, 'max_closed': self.max_closed_size, 'depth': None, 'path_length': None, 'solved': False}

    def _dls(self, start_pos, limit):
        start_node = Node(start_pos)
        open_stack = [start_node]
        closed_set = set()
        
        while open_stack:
            self.max_open_size = max(self.max_open_size, len(open_stack))
            current = open_stack.pop()
            
            if current.position in closed_set:
                continue
            
            closed_set.add(current.position)
            self.nodes_expanded += 1
            self.max_closed_size = max(self.max_closed_size, len(closed_set))
            
            if current.position == self.maze.goal_pos:
                return self.reconstruct_path(current), True
            
            if current.depth < limit:
                for nxt_pos in self.maze.get_neighbors(current.position):
                    if nxt_pos not in closed_set:
                        nxt_node = Node(nxt_pos, parent=current, depth=current.depth + 1)
                        open_stack.append(nxt_node)
                        
        return None, False


class BestFirst(SearchAlgorithm):
    """Búsqueda Primero el Mejor (Greedy) - Ordena OPEN por f(n) = h(n)"""
    def solve(self):
        if not self.maze.start_pos or not self.maze.goal_pos:
            return None, {'nodes_expanded': 0, 'max_open': 0, 'max_closed': 0, 'depth': None, 'path_length': None, 'solved': False}
        
        start_node = Node(self.maze.start_pos, h=manhattan_distance(self.maze.start_pos, self.maze.goal_pos))
        start_node.f = start_node.h
        
        counter = 0
        open_pq = [(start_node.f, counter, start_node)]
        closed_set = set()
        
        while open_pq:
            self.max_open_size = max(self.max_open_size, len(open_pq))
            _, _, current = heapq.heappop(open_pq)
            
            if current.position in closed_set:
                continue
            
            closed_set.add(current.position)
            self.nodes_expanded += 1
            self.max_closed_size = max(self.max_closed_size, len(closed_set))
            
            if current.position == self.maze.goal_pos:
                path = self.reconstruct_path(current)
                return path, {
                    'nodes_expanded': self.nodes_expanded,
                    'max_open': self.max_open_size,
                    'max_closed': self.max_closed_size,
                    'depth': current.depth,
                    'path_length': len(path),
                    'solved': True
                }
            
            for nxt_pos in self.maze.get_neighbors(current.position):
                if nxt_pos not in closed_set:
                    h_val = manhattan_distance(nxt_pos, self.maze.goal_pos)
                    nxt_node = Node(nxt_pos, parent=current, h=h_val, depth=current.depth + 1)
                    nxt_node.f = h_val
                    counter += 1
                    heapq.heappush(open_pq, (nxt_node.f, counter, nxt_node))
                    
        return None, {'nodes_expanded': self.nodes_expanded, 'max_open': self.max_open_size, 'max_closed': self.max_closed_size, 'depth': None, 'path_length': None, 'solved': False}


class AStar(SearchAlgorithm):
    """Búsqueda A* - Ordena OPEN por f(n) = g(n) + h(n)"""
    def solve(self):
        if not self.maze.start_pos or not self.maze.goal_pos:
            return None, {'nodes_expanded': 0, 'max_open': 0, 'max_closed': 0, 'depth': None, 'path_length': None, 'solved': False}
        
        h_start = manhattan_distance(self.maze.start_pos, self.maze.goal_pos)
        start_node = Node(self.maze.start_pos, g=0, h=h_start)
        
        counter = 0
        open_pq = [(start_node.f, counter, start_node)]
        closed_set = set()
        
        while open_pq:
            self.max_open_size = max(self.max_open_size, len(open_pq))
            _, _, current = heapq.heappop(open_pq)
            
            if current.position in closed_set:
                continue
            
            closed_set.add(current.position)
            self.nodes_expanded += 1
            self.max_closed_size = max(self.max_closed_size, len(closed_set))
            
            if current.position == self.maze.goal_pos:
                path = self.reconstruct_path(current)
                return path, {
                    'nodes_expanded': self.nodes_expanded,
                    'max_open': self.max_open_size,
                    'max_closed': self.max_closed_size,
                    'depth': current.depth,
                    'path_length': len(path),
                    'solved': True
                }
            
            for nxt_pos in self.maze.get_neighbors(current.position):
                if nxt_pos not in closed_set:
                    g_val = current.g + 1
                    h_val = manhattan_distance(nxt_pos, self.maze.goal_pos)
                    nxt_node = Node(nxt_pos, parent=current, g=g_val, h=h_val, depth=current.depth + 1)
                    counter += 1
                    heapq.heappush(open_pq, (nxt_node.f, counter, nxt_node))
                    
        return None, {'nodes_expanded': self.nodes_expanded, 'max_open': self.max_open_size, 'max_closed': self.max_closed_size, 'depth': None, 'path_length': None, 'solved': False}



# 5. VISUALIZADOR DE LABERINTOS
class MazeVisualizer:
    @staticmethod
    def render(maze, path=None, title="", save_path=None):
        vis_grid = np.zeros((maze.rows, maze.cols), dtype=int)
        for r in range(maze.rows):
            for c in range(maze.cols):
                if maze.grid[r, c] == 3:
                    vis_grid[r, c] = 1  # Muro
                else:
                    vis_grid[r, c] = 0  # Pasillo libre
                    
        if path:
            for r, c in path:
                vis_grid[r, c] = 2  # Ruta encontrada
                
        if maze.start_pos:
            vis_grid[maze.start_pos] = 3  # Inicio
            
        if maze.goal_pos:
            vis_grid[maze.goal_pos] = 4  # Meta
            
        # Mapa de colores personalizado: Blanco (Pasillo), Azul/Gris (Muro), Verde (Ruta), Amarillo (Inicio), Rojo (Meta)
        cmap = ListedColormap(['#FFFFFF', '#2C3E50', '#2ECC71', '#F1C40F', '#E74C3C'])
        
        plt.figure(figsize=(7, 7))
        plt.imshow(vis_grid, cmap=cmap, vmin=0, vmax=4)
        plt.title(title, fontsize=12, fontweight='bold', pad=10)
        plt.xticks([])
        plt.yticks([])
        
        plt.tight_layout()
        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close()
        else:
            plt.show()