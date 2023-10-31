#include "input.hpp"

#include <iostream>
#include <fstream>

void input(int* method, std::istream& stream) {
    if (&stream == &std::cin)
        std::cout << "Select method:\n1. A* for system graph\n2. Potential fields";
    stream >> *method;
}
void input(Point* goals, std::istream &stream) {
    if (&stream == &std::cin)
        std::cout << "Enter goals coordinates in format '%d %d': " << std::endl;
    for (int i = 0; i < ROBOTS_COUNT; i++) {
        if (&stream == &std::cin)
            std::cout << i + 1 << ". ";
        stream >> (goals[i]).x >> (goals[i]).y;
    }
}
void input(Robot* robots, std::istream &stream) {
    if (&stream == &std::cin)
        std::cout << "Enter robots coordinates in format '%d %d': " << std::endl;
    for (int i = 0; i < ROBOTS_COUNT; i++) {
        if (&stream == &std::cin)
            std::cout << i + 1 << ". ";
        robots[i].position = new Point(0, 0);
        stream >> (robots[i]).position->x >> (robots[i]).position->y;
    }
}
void input(Point** Map, std::istream &stream) {
    for (int i = 0; i < MAP_SIZE_X; i++) {
        Map[i] = new Point[MAP_SIZE_Y];
        for (int j = 0; j < MAP_SIZE_Y; j++)
            Map[i][j] = *(new Point(i, j));
    }
    if (&stream == &std::cin)
        std::cout << "Enter obstacle (blocked points) coordinates in format '%d %d'. End when entered '-1 -1': " << std::endl;
    for (int i = 0; 0 == 0; i++) {
        int x, y;
        stream >> x >> y;
        if (x < 0 || y < 0)
            break;
        Map[x][y].blocked = true;
    }
}
