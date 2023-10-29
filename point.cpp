
#include <cmath>
#include "point.hpp"
#include "robot.hpp"

int distance_to(Point p1, Point p2) {
    return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}

int distance_to_near(Point* goals, Point* p) {
    int min = 99999;
    for (int i = 0; i < ROBOTS_COUNT; i++)
        min = min > distance_to(goals[i], *p) ? distance_to(goals[i], *p) : min;
    return min;
}
