#pragma once

#include <vector>
#include <string>
#include <utility>
#include <chrono>

enum class Algorithm {
    BFS,
    Dijkstra,
    AStar
};

struct Node {
    int r, c;
    int g_cost;
    int h_cost;
    int f_cost() const { return g_cost + h_cost; }

    bool operator>(const Node& other) const {
        return f_cost() > other.f_cost();
    }
};

struct PathResult {
    bool found;
    int path_length;
    int nodes_visited;
    std::chrono::microseconds duration;
    std::vector<std::string> rendered_grid;
};

class GridSolver {
public:
    GridSolver() = default;
    GridSolver(const std::vector<std::string>& ascii_map);
    
    // Generates an odd-dimensioned maze with randomized DFS
    static std::vector<std::string> generate_maze(int rows, int cols, unsigned int seed = 0);

    void set_grid(const std::vector<std::string>& ascii_map);
    PathResult solve(Algorithm algo);

private:
    std::vector<std::string> grid_;
    int rows_ = 0;
    int cols_ = 0;
    std::pair<int, int> start_ = {0, 0};
    std::pair<int, int> goal_ = {0, 0};

    int manhattan(int r, int c) const;
    bool in_bounds(int r, int c) const;
    bool is_walkable(int r, int c) const;
};