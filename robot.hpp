#pragma once
#include "point.hpp"
#include <vector>

#define ROBOTS_COUNT 3

class Robot {
public:
    Point *position;
    Point goal;
    bool finish = false;
    std::vector<Point> trajectory;
    operator Point() const { return *position; }
    Robot() {
        position = nullptr;
    }
    Robot(Point *_position, Point _goal) {
        position = _position;
        goal = _goal;
    }
    Robot(const Robot& r) {
        this->position = r.position;
        goal = r.goal;
        finish = r.finish;
        trajectory = r.trajectory;
    }
    friend bool operator ==(Robot r1, Robot r2) {
        return r1.position == r2.position && r1.goal == r2.goal;
    }
    friend bool operator !=(Robot r1, Robot r2) {
        return !(r1 == r2);
    }
protected:
    bool collision_check(const Robot* other_robots, Point next_position);
};
Point* get_robots_positions(Robot* robots, size_t count);
