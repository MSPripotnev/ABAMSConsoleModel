#pragma once

#include "potential-field.hpp"

#include "map.hpp"
#include "robot.hpp"
#include <vector>

constexpr auto GOAL_POTENTIAL = 20;
constexpr auto ROBOT_POTENTIAL = -4;
constexpr auto OBSTACLE_POTENTIAL = -50;

struct PotentialPoint : Point {
    int potential = 0;
    PotentialPoint(const Point& p, int _potential) : Point(p) {
        potential = _potential;
    }
    PotentialPoint() {
        x = y = 0;
        blocked = false;
        potential = 0;
    }
    static PotentialPoint** get_nullpot_map(Point** pot_map);
};

class PotentialRobot : Robot {
public:
    PotentialPoint** potential_map;
    int makespan_ms = 0;
    bool finish = false;
    PotentialRobot() {
        other_robots = nullptr;
        potential_map = nullptr;
        this->finish = false;
    }
    PotentialRobot(const PotentialRobot &r) : Robot(r) {
        this->finish = false;
        other_robots = r.other_robots;
        potential_map = r.potential_map;
    }
    PotentialRobot(Robot& r, Point **map, Robot *_robots) : Robot(r) {
        other_robots = new Robot*[ROBOTS_COUNT - 1];
        for (int i = 0, k = 0; i < ROBOTS_COUNT; i++, k++) {
            if (_robots[i] != (Robot)*this)
                other_robots[k] = &_robots[i];
            else k--;
        }
        potential_map = build_potential_map(PotentialPoint::get_nullpot_map(map), other_robots, goal);
    }
    void print_potential_map();
    void move() override;
private:
    Robot **other_robots;
    void escape_dead_end();
    Point select_direction();
    PotentialPoint** build_potential_map(PotentialPoint** Map, Robot** robots, Point goal);
    void finalize() override;
};
