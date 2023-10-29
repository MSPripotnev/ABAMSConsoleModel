#include "robot.hpp"

Point* get_robots_positions(Robot* robots, size_t count) {
    Point* result = new Point[count];
    for (int i = 0; i < count; i++)
        result[i] = robots[i];
    return result;
}

bool Robot::collision_check(const Robot* other_robots, Point next_position) {
    for (int i = 0; i < ROBOTS_COUNT - 1; i++)
        if (next_position == other_robots[i])
            return true;
    return false;
}
