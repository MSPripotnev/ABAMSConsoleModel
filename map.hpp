
#pragma once
#include "point.hpp"
#include "robot.hpp"

#define MAP_SIZE_X 10
#define MAP_SIZE_Y 10

int distance_to_obstacle(Point **map, Point p);
void print_map(Point** map, Robot* robots, Point* goals);
