import os
import pandas as pd
from solver import Maze, DFS, BFS, IDDFS, BestFirst, AStar, MazeVisualizer

def run_experiments(mazes_dir="./Laberintos", snapshots_dir="./snapshots", output_csv="resultados_busqueda.csv"):
    os.makedirs(snapshots_dir, exist_ok=True)
    
    # Buscar todos los archivos .txt de laberintos
    maze_files = sorted([f for f in os.listdir(mazes_dir) if f.startswith("maze_") and f.endswith(".txt")])
    
    algorithms = [
        ("DF", DFS),
        ("BF", BFS),
        ("IDDF", IDDFS),
        ("Best_First", BestFirst),
        ("A_Star", AStar)
    ]
    
    results = []
    
    print(f"Iniciando ejecuciones para {len(maze_files)} laberintos y 5 algoritmos...\n")
    
    for maze_file in maze_files:
        maze_path = os.path.join(mazes_dir, maze_file)
        maze = Maze(maze_path)
        
        for alg_name, alg_class in algorithms:
            solver = alg_class(maze)
            path, metrics = solver.solve()
            
            snapshot_filename = f"{os.path.splitext(maze_file)[0]}_{alg_name}.png"
            snapshot_path = os.path.join(snapshots_dir, snapshot_filename)
            
            title_str = f"{maze_file} | {alg_name}\n"
            if metrics['solved']:
                title_str += f"Pasos: {metrics['nodes_expanded']} | Longitud: {metrics['path_length']} | Max OPEN: {metrics['max_open']}"
            else:
                title_str += f"Sin Solución | Pasos: {metrics['nodes_expanded']}"
                
            MazeVisualizer.render(maze, path, title=title_str, save_path=snapshot_path)
            
            heuristica_str = "Manhattan" if alg_name in ["Best_First", "A_Star"] else "N/A"
            
            results.append({
                'Laberinto': maze_file,
                'Algoritmo': alg_name,
                'Resuelto': 'Sí' if metrics['solved'] else 'No',
                'Pasos (Nodos Exp.)': metrics['nodes_expanded'],
                'Profundidad Sol.': metrics['depth'],
                'Long. Camino': metrics['path_length'],
                'Max OPEN': metrics['max_open'],
                'Max CLOSED': metrics['max_closed'],
                'Heurística': heuristica_str,
                'Snapshot': snapshot_filename
            })
            
    # Guardar los datos en Excel/CSV
    df = pd.DataFrame(results)
    df.to_csv(output_csv, index=False, encoding='utf-8-sig')
    
    print("=" * 80)
    print(" EJECUCIÓN COMPLETADA EXITOSAMENTE")
    print("=" * 80)
    print(f"1. Archivo de métricas CSV generado: {output_csv}")
    print(f"2. {len(results)} capturas (Snapshots) guardadas en la carpeta: {snapshots_dir}/\n")
    
    # Imprimir resumen de tabla
    print(df[['Laberinto', 'Algoritmo', 'Resuelto', 'Pasos (Nodos Exp.)', 'Long. Camino', 'Max OPEN', 'Max CLOSED']].to_string())

if __name__ == "__main__":
    run_experiments()