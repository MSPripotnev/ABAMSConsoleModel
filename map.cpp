#include "map.hpp"

#include <iostream>

#define MAX_OBSTACLE_POTENTIAL 2

bool point_is_out_of_range(Point p) {
    return p.x < 0 || p.y < 0 ||
        p.x >= MAP_SIZE_X || p.y >= MAP_SIZE_Y ||
        p.blocked;
}

int distance_to_obstacle(Point** map, Point p) {
    int min = 999;
    for (int i = 0; i < MAX_OBSTACLE_POTENTIAL; i++)
        for (int j = 1; j < 9; j += 2) {
            //выбор направления
            Point pos = Point(p.x + (i / 3 - 1), p.y + (i % 3 - 1));
            if (pos.x < 0 || pos.y < 0 || pos.x >= MAP_SIZE_X || pos.y >= MAP_SIZE_Y || pos.blocked)
                continue;
            if (pos.blocked && distance_to(pos, p) < min)
                min = distance_to(pos, p);
        }
    return min > MAX_OBSTACLE_POTENTIAL ? 0 : min;
}

void print_map(Point** map, Robot* robots, Point* goals) {
    for (int i = -1; i < MAP_SIZE_X + 1; i++) {
        for (int j = -1; j < MAP_SIZE_Y + 1; j++) {
            if (i < 0 || i >= MAP_SIZE_X || j < 0 || j >= MAP_SIZE_Y || map[i][j].blocked)
                std::cout << " O ";
            else if (!distance_to_near(get_robots_positions(robots, ROBOTS_COUNT), &map[i][j])) {
                long long r = std::distance(robots, std::find(robots, robots + ROBOTS_COUNT, map[i][j]));
                std::cout << " R" << r + 1;
            }
            else if (!distance_to_near(goals, &map[i][j])) {
                long long r = std::distance(goals, std::find(goals, goals + ROBOTS_COUNT, map[i][j]));
                std::cout << " X" << r + 1;
            }
            else
                std::cout << "   ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
