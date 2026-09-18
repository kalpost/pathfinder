#include "pathfinder.hpp"
#include <iostream>

int main() {
    // Generate a 15x35 random maze
    auto procedural_maze = GridSolver::generate_maze(15, 35);

    GridSolver solver(procedural_maze);

    std::cout << "================ PROCEDURAL MAZE ================" << '\n';
    for (const auto& row : procedural_maze) {
        std::cout << row << '\n';
    }
    std::cout << '\n';

    // Solve with A*
    PathResult res_astar = solver.solve(Algorithm::AStar);
    std::cout << "================ A* SEARCH RESOLUTION ================" << '\n';
    for (const auto& row : res_astar.rendered_grid) {
        std::cout << row << '\n';
    }
    std::cout << "Status: " << (res_astar.found ? "Solved" : "Failed")
              << " | Length: " << res_astar.path_length 
              << " | Visited: " << res_astar.nodes_visited 
              << " | Runtime: " << res_astar.duration.count() << " us\n\n";

    // Solve with BFS
    PathResult res_bfs = solver.solve(Algorithm::BFS);
    std::cout << "================ BFS SEARCH STATS ================" << '\n';
    std::cout << "Status: " << (res_bfs.found ? "Solved" : "Failed")
              << " | Length: " << res_bfs.path_length 
              << " | Visited: " << res_bfs.nodes_visited 
              << " | Runtime: " << res_bfs.duration.count() << " us\n";

    return 0;
}