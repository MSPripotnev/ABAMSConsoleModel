#pragma once
#include "point.hpp"
#include <vector>
#include <chrono>

#define ROBOTS_COUNT 3

class Robot {
public:
    Point *position;
    Point goal;
    int makespan_ms = 0;
    bool finish = false;
    std::vector<Point> trajectory;
    operator Point() const { return *position; }
    Robot() {
        position = nullptr;
    }
    Robot(Point *_position, Point _goal) {
        position = _position;
        goal = _goal;
        start = std::chrono::steady_clock::now();
    }
    Robot(const Robot& r) : Robot(r.position, r.goal) {
        finish = r.finish;
        trajectory = r.trajectory;
        start = std::chrono::steady_clock::now();
    }
    friend bool operator ==(Robot r1, Robot r2) {
        return r1.position == r2.position && r1.goal == r2.goal;
    }
    friend bool operator !=(Robot r1, Robot r2) {
        return !(r1 == r2);
    }
    virtual void move();
    virtual void finalize();
protected:
    bool collision_check(const Robot* other_robots, Point next_position);

private:
    std::chrono::steady_clock::time_point start;
};
Point* get_robots_positions(Robot* robots, size_t count);
