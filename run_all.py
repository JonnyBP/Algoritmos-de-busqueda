import argparse
import os
import pandas as pd
from solver import Maze, DFS, BFS, IDDFS, BestFirst, AStar, MazeVisualizer

# (nombre en el CSV, clase, heurística usada)
ALGORITHMS = [
    ("DF", DFS, "N/A"),
    ("BF", BFS, "N/A"),
    ("IDDF", IDDFS, "N/A"),
    ("Best_First", BestFirst, "Manhattan"),
    ("A_Star", AStar, "Manhattan"),
]


def build_title(maze_file, alg_name, metrics):
    title = f"{maze_file} | {alg_name}\n"
    if metrics['solved']:
        return title + (f"Nodos exp.: {metrics['nodes_expanded']} | Longitud: {metrics['path_length']} "
                        f"| Max OPEN: {metrics['max_open']}")
    return title + f"Sin solución | Nodos exp.: {metrics['nodes_expanded']}"


def run_experiments(mazes_dir="./Laberintos", snapshots_dir="./snapshots", output_csv="resultados_busqueda.csv"):
    if not os.path.isdir(mazes_dir):
        raise FileNotFoundError(f"No existe la carpeta de laberintos: {mazes_dir}")
    os.makedirs(snapshots_dir, exist_ok=True)

    maze_files = sorted(f for f in os.listdir(mazes_dir) if f.startswith("maze_") and f.endswith(".txt"))
    if not maze_files:
        raise FileNotFoundError(f"No hay archivos maze_*.txt en {mazes_dir}")

    print(f"Iniciando ejecuciones para {len(maze_files)} laberintos y {len(ALGORITHMS)} algoritmos...\n")

    results = []
    for maze_file in maze_files:
        maze = Maze(os.path.join(mazes_dir, maze_file))

        for alg_name, alg_class, heuristic in ALGORITHMS:
            path, metrics = alg_class(maze).solve()

            snapshot_filename = f"{os.path.splitext(maze_file)[0]}_{alg_name}.png"
            MazeVisualizer.render(maze, path, title=build_title(maze_file, alg_name, metrics),
                                    save_path=os.path.join(snapshots_dir, snapshot_filename))

            results.append({
                'Laberinto': maze_file,
                'Algoritmo': alg_name,
                'Resuelto': 'Sí' if metrics['solved'] else 'No',
                'Nodos expandidos': metrics['nodes_expanded'],
                'Profundidad Sol.': metrics['depth'],   # nº de movimientos
                'Long. Camino': metrics['path_length'],  # nº de celdas
                'Max OPEN': metrics['max_open'],
                'Max CLOSED': metrics['max_closed'],
                'Heurística': heuristic,
                'Snapshot': snapshot_filename,
            })

    df = pd.DataFrame(results)
    df = df.astype({'Profundidad Sol.': 'Int64', 'Long. Camino': 'Int64'})
    df.to_csv(output_csv, index=False, encoding='utf-8-sig')

    print("=" * 80)
    print(" EJECUCIÓN COMPLETADA")
    print("=" * 80)
    print(f"1. Métricas CSV: {output_csv}")
    print(f"2. {len(results)} capturas guardadas en: {snapshots_dir}/\n")
    print(df[['Laberinto', 'Algoritmo', 'Resuelto', 'Nodos expandidos',
                'Long. Camino', 'Max OPEN', 'Max CLOSED']].to_string())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mazes-dir", default="./Laberintos", help="carpeta con los maze_*.txt")
    parser.add_argument("--snapshots-dir", default="./snapshots", help="carpeta de salida de las imágenes")
    parser.add_argument("--output", default="resultados_busqueda.csv", help="archivo CSV de métricas")
    args = parser.parse_args()
    run_experiments(args.mazes_dir, args.snapshots_dir, args.output)