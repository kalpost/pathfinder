# Multi-Strategy Pathfinding Visualizer & Maze Engine

A lightweight, zero-dependency C++17 pathfinding suite and procedural maze generator. This project benchmarks informed versus uninformed graph traversal algorithms across discrete 2D grid topologies.

---

## Features

* **Procedural Map Synthesis:** Generates perfect mazes (spanning trees without loops or isolated clusters) via randomized depth-first search (recursive backtracker).
* **Algorithmic Traversal Implementations:**
  * **A* Search:** Heuristic-driven search utilizing Manhattan distance ($L_1$ norm) to minimize evaluated state spaces:
    $$f(n) = g(n) + h(n)$$
  * **Dijkstra's Algorithm:** Uniform-cost graph exploration utilizing a min-priority queue (`std::priority_queue`).
  * **Breadth-First Search (BFS):** Uninformed queue-based expansion guaranteeing the shortest hop-count path on unweighted planar graphs.
* **Microsecond Profiling:** Built-in execution timing via `std::chrono::high_resolution_clock` to benchmark node exploration metrics and runtimes directly in the terminal.

---

## Algorithmic Comparison

| Metric | Breadth-First Search (BFS) | Dijkstra's Algorithm | A* Search (Manhattan) |
| :--- | :--- | :--- | :--- |
| **Optimality Guarantee** | Yes (unweighted) | Yes | Yes (admissible heuristic) |
| **Frontier Structure** | FIFO Queue (`std::queue`) | Min-Heap (`std::priority_queue`) | Min-Heap (`std::priority_queue`) |
| **Explored State Density** | Exhaustive planar expansion | Radial cost expansion | Directional heuristic pruning |
| **Heuristic Overhead** | None | None | $\mathcal{O}(1)$ evaluation per neighbor |

---

## Getting Started

### Prerequisites

* C++17 compatible compiler (`clang++` $\ge$ 10.0, `g++` $\ge$ 9.0, or MSVC)
* CMake 3.16+

### Build Instructions

```bash
# Clone the repository
git clone [https://github.com/kalpost/pathfinder.git](https://github.com/kalpost/pathfinder.git)
cd pathfinding-visualizer

# Generate build configuration
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile
cmake --build build

# Execute
./build/pathfinder