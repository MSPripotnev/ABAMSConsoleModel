#include "potential-field.hpp"
#include "map.hpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

PotentialPoint** PotentialPoint::get_nullpot_map(Point** pot_map) {
    PotentialPoint** PotentialMap = new PotentialPoint * [MAP_SIZE_X];
    for (int i = 0; i < MAP_SIZE_X; i++) {
        PotentialMap[i] = new PotentialPoint[MAP_SIZE_Y];
        for (int j = 0; j < MAP_SIZE_Y; j++)
            PotentialMap[i][j] = PotentialPoint(pot_map[i][j], 0);
    }
    return PotentialMap;
}

void PotentialRobot::escape_dead_end() {
    if (trajectory.size() > 5 && trajectory.back() == *(trajectory.end() - 3) && 
        *(trajectory.end() - 3) == *(trajectory.end() - 5) && !collision_check(*other_robots, *(trajectory.end()-1))) {
        Point p = trajectory[trajectory.size() - 1];
        potential_map[p.x][p.y].blocked = true;
    }
}

Point PotentialRobot::select_direction() {
    int max_potential = 0;
    Point next_position;
    for (int i = 1; i < 9; i += 2) {
        //выбор направления
        Point pos = Point(position->x + (i / 3 - 1), position->y + (i % 3 - 1));
        if (pos.x < 0 || pos.y < 0 || pos.x >= MAP_SIZE_X || pos.y >= MAP_SIZE_Y || pos.blocked)
            continue;
        PotentialPoint p = potential_map[pos.x][pos.y];
        if (max_potential < p.potential - ROBOT_POTENTIAL) {
            max_potential = p.potential - ROBOT_POTENTIAL;
            next_position = pos;
        }
    }
    //if (next_position.blocked)
    //    throw "Cannot find trajectory!";
    return next_position;
}

void PotentialRobot::move() {
    this->position->blocked = false;
    Point next = select_direction();
    if (collision_check(*other_robots, next))
        return;
    *this->position = next;
    trajectory.push_back(*this->position);
    escape_dead_end();

    if (!(finish = *position == goal))
        potential_map = build_potential_map(potential_map, other_robots, goal);
    else finalize();
}
void PotentialRobot::finalize() {
    Robot::finalize();
    makespan_ms = Robot::makespan_ms;
    delete[] potential_map[0];
    delete[] potential_map;
    delete[] other_robots;
}

PotentialPoint** PotentialRobot::build_potential_map(PotentialPoint** Map, Robot** other_robots, Point goal) {
    PotentialPoint** PotentialMap = new PotentialPoint * [MAP_SIZE_X];
    // построение карты потенциалов
    for (int i = 0; i < MAP_SIZE_X; i++) {
        PotentialMap[i] = new PotentialPoint[MAP_SIZE_Y];
        for (int j = 0; j < MAP_SIZE_Y; j++) {
            int potential = 0;
            potential += std::max(GOAL_POTENTIAL - distance_to(goal, Map[i][j]), 0);
            for (int k = 0; k < ROBOTS_COUNT - 1; k++) {
                if (!other_robots[k]->finish)
                    potential += std::min(ROBOT_POTENTIAL + distance_to(Map[i][j], *(other_robots[k]->position)), 0);
                else Map[other_robots[k]->position->x][other_robots[k]->position->y].blocked = true;
            }

            potential -= distance_to_obstacle((Point**)Map, Map[i][j]);
            if (Map[i][j].blocked)
                potential = OBSTACLE_POTENTIAL;
            PotentialMap[i][j] = PotentialPoint(Map[i][j], potential);
        }
    }
    return PotentialMap;
}

void PotentialRobot::print_potential_map() {
    for (int i = 0; i < MAP_SIZE_X; i++) {
        for (int j = 0; j < MAP_SIZE_Y; j++) {
            std::cout << (this->potential_map[i][j].potential < 0 ? " -" : " +") << abs(this->potential_map[i][j].potential);
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
