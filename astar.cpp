#include "astar.hpp"
#include "visualizator.hpp"
#include "map.hpp"
#include <vector>

static bool points_out_of_range(Point *points) {
    for (int r = 0; r < ROBOTS_COUNT; r++) {
        if (point_is_out_of_range(points[r]))
            return true;
    }
    return false;
}

void AStarSystem::open_points() {
    for (int i = -1; i < 9; i += 2)
        for (int j = -1; j < 9; j += 2)
            for (int k = -1; k < 9; k += 2) {
                if (i == -1 && j == -1 && k == -1)
                    continue;
                int x0 = position->robot_positions[0].x + (i > 0 ? (i / 3 - 1) : 0),
                    y0 = position->robot_positions[0].y + (i > 0 ? (i % 3 - 1) : 0),
                    x1 = position->robot_positions[1].x + (j > 0 ? (j / 3 - 1) : 0),
                    y1 = position->robot_positions[1].y + (j > 0 ? (j % 3 - 1) : 0),
                    x2 = position->robot_positions[2].x + (k > 0 ? (k / 3 - 1) : 0),
                    y2 = position->robot_positions[2].y + (k > 0 ? (k % 3 - 1) : 0);
                if (x0 < 0 || y0 < 0 || x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 ||
                    x0 >= MAP_SIZE_X || y0 >= MAP_SIZE_Y || x1 >= MAP_SIZE_X || y1 >= MAP_SIZE_Y || x2 >= MAP_SIZE_X || y2 >= MAP_SIZE_Y)
                    continue;
                Point new_robot_positions[ROBOTS_COUNT]{ map[x0][y0], map[x1][y1], map[x2][y2] };
                MultiAStarNode node = MultiAStarNode(*this->position, new_robot_positions, goal);

                if (new_robot_positions[0] == new_robot_positions[1] ||
                        new_robot_positions[1] == new_robot_positions[2] ||
                        new_robot_positions[2] == new_robot_positions[0] ||
                        new_robot_positions[0] == position->robot_positions[1] ||
                        new_robot_positions[0] == position->robot_positions[2] ||
                        new_robot_positions[1] == position->robot_positions[0] ||
                        new_robot_positions[1] == position->robot_positions[2] ||
                        new_robot_positions[2] == position->robot_positions[0] ||
                        new_robot_positions[2] == position->robot_positions[1] ||
                        points_out_of_range(new_robot_positions))
                    closedPoints.insert(node).second;
                if (!closedPoints.count(node))
                    openedPoints.insert(node);
            }
    for (auto& i : openedPoints) {
        for (const MultiAStarNode* node = &i; node; node = node->parent) {

        }
    }
    graph_to_dot(openedPoints);
}

MultiAStarNode AStarSystem::select_direction() {
    const MultiAStarNode res = *openedPoints.begin();
    result = new MultiAStarNode(res);
    closedPoints.insert(*result);
    openedPoints.erase(*result);
    return *result;
}

std::vector<MultiAStarNode> create_path_from_node(MultiAStarNode *finish_node, int (*makespan_it)[ROBOTS_COUNT]) {
    std::vector<MultiAStarNode> path;
    for (MultiAStarNode* node = finish_node; node; node = node->parent) {
        for (int i = 0; i < ROBOTS_COUNT; i++) {
            if (node->robot_positions[i] != finish_node->robot_positions[i])
                (*makespan_it)[i]++;
        }
        path.push_back(*node);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void AStarSystem::move() {
    open_points();
    MultiAStarNode next_node = select_direction();
    this->position = new MultiAStarNode(next_node);
    finish = true;
    for (int i = 0; i < ROBOTS_COUNT; i++) {
        finish &= this->position->robot_positions[i] == goal.robot_positions[i];
        if (this->position->robot_positions[i] == goal.robot_positions[i]) {
            if (!robots[i]->finish) {
                robots[i]->finalize();
            }
        }
    }
    if (finish) {
        trajectory = create_path_from_node(&next_node, &this->makespan_it);
        finalize();
    }
}

void AStarSystem::finalize() {
    delete[] robots;
}
