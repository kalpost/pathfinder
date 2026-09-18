#include "pathfinder.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <random>
#include <stack>

GridSolver::GridSolver(const std::vector<std::string>& ascii_map) {
    set_grid(ascii_map);
}

void GridSolver::set_grid(const std::vector<std::string>& ascii_map) {
    grid_ = ascii_map;
    rows_ = static_cast<int>(ascii_map.size());
    cols_ = rows_ > 0 ? static_cast<int>(ascii_map[0].size()) : 0;
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            if (grid_[r][c] == 'S') start_ = {r, c};
            if (grid_[r][c] == 'E') goal_ = {r, c};
        }
    }
}

std::vector<std::string> GridSolver::generate_maze(int rows, int cols, unsigned int seed) {
    // Grid dimensions must be odd to allow walls between passages
    if (rows % 2 == 0) rows++;
    if (cols % 2 == 0) cols++;

    std::vector<std::string> maze(rows, std::string(cols, '#'));
    std::mt19937 rng(seed == 0 ? std::random_device{}() : seed);

    std::stack<std::pair<int, int>> st;
    maze[1][1] = '.';
    st.push({1, 1});

    const int dr[] = {-2, 2, 0, 0};
    const int dc[] = {0, 0, -2, 2};

    while (!st.empty()) {
        auto [r, c] = st.top();

        std::vector<int> neighbors;
        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            if (nr > 0 && nr < rows - 1 && nc > 0 && nc < cols - 1 && maze[nr][nc] == '#') {
                neighbors.push_back(i);
            }
        }

        if (!neighbors.empty()) {
            std::shuffle(neighbors.begin(), neighbors.end(), rng);
            int dir = neighbors[0];
            int nr = r + dr[dir];
            int nc = c + dc[dir];

            // Carve intermediate wall and target cell
            maze[r + dr[dir] / 2][c + dc[dir] / 2] = '.';
            maze[nr][nc] = '.';

            st.push({nr, nc});
        } else {
            st.pop();
        }
    }

    maze[1][1] = 'S';
    maze[rows - 2][cols - 2] = 'E';

    return maze;
}

int GridSolver::manhattan(int r, int c) const {
    return std::abs(r - goal_.first) + std::abs(c - goal_.second);
}

bool GridSolver::in_bounds(int r, int c) const {
    return r >= 0 && r < rows_ && c >= 0 && c < cols_;
}

bool GridSolver::is_walkable(int r, int c) const {
    return in_bounds(r, c) && grid_[r][c] != '#';
}

PathResult GridSolver::solve(Algorithm algo) {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::string> display = grid_;
    std::vector<std::vector<bool>> visited(rows_, std::vector<bool>(cols_, false));
    std::vector<std::vector<std::pair<int, int>>> parent(rows_, std::vector<std::pair<int, int>>(cols_, {-1, -1}));
    std::vector<std::vector<int>> g_dist(rows_, std::vector<int>(cols_, 1e9));

    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::queue<std::pair<int, int>> q;

    int nodes_explored = 0;
    bool reached = false;

    if (algo == Algorithm::BFS) {
        q.push(start_);
        visited[start_.first][start_.second] = true;
    } else {
        int h = (algo == Algorithm::AStar) ? manhattan(start_.first, start_.second) : 0;
        pq.push({start_.first, start_.second, 0, h});
        g_dist[start_.first][start_.second] = 0;
    }

    while ((algo == Algorithm::BFS && !q.empty()) || (algo != Algorithm::BFS && !pq.empty())) {
        int r, c;
        if (algo == Algorithm::BFS) {
            auto curr = q.front(); q.pop();
            r = curr.first; c = curr.second;
        } else {
            auto curr = pq.top(); pq.pop();
            r = curr.r; c = curr.c;
            if (visited[r][c]) continue;
            visited[r][c] = true;
        }

        nodes_explored++;

        if (r == goal_.first && c == goal_.second) {
            reached = true;
            break;
        }

        if (grid_[r][c] != 'S') display[r][c] = '~';

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];

            if (!is_walkable(nr, nc)) continue;

            if (algo == Algorithm::BFS) {
                if (!visited[nr][nc]) {
                    visited[nr][nc] = true;
                    parent[nr][nc] = {r, c};
                    q.push({nr, nc});
                }
            } else {
                int new_g = g_dist[r][c] + 1;
                if (new_g < g_dist[nr][nc]) {
                    g_dist[nr][nc] = new_g;
                    parent[nr][nc] = {r, c};
                    int h = (algo == Algorithm::AStar) ? manhattan(nr, nc) : 0;
                    pq.push({nr, nc, new_g, h});
                }
            }
        }
    }

    int path_len = 0;
    if (reached) {
        auto curr = parent[goal_.first][goal_.second];
        while (curr != std::make_pair(-1, -1) && curr != start_) {
            display[curr.first][curr.second] = '*';
            curr = parent[curr.first][curr.second];
            path_len++;
        }
        path_len++;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    return {reached, path_len, nodes_explored, elapsed, display};
}