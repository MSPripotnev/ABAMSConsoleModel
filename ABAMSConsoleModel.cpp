/*
* Бригада:
*   - Волков Виктор
*   - Панфилов Александр
*   - Припотнев Михаил
*/
#include <fstream>
#include <iostream>
#include <tuple>

#include "input.hpp"
#include "map.hpp"
#include "windows.h"
#include "potential-field.hpp"

#define PRINT_MAP 1
#define PRINT_POTENTIAL_MAP 0
#define ROBOT_POTENTIAL_MAP_INDEX 1
#define REFRESH_TIME 200

std::tuple<int, int> potential_field(Point **Map, Robot* robots, Point *goals) {
    int makespan_it = 0, flowtime_it = 0;
    PotentialRobot* pot_robots = new PotentialRobot[ROBOTS_COUNT];
    for (int i = 0; i < ROBOTS_COUNT; i++)
        pot_robots[i] = PotentialRobot(robots[i], Map, robots);
    while (true) {
        flowtime_it++;
        bool all_finish = true;
        for (int i = 0; i < ROBOTS_COUNT; i++) {
#if PRINT_POTENTIAL_MAP
            if (i == ROBOT_POTENTIAL_MAP_INDEX)
                pot_robots[i].print_potential_map();
#endif
            if (!pot_robots[i].finish) {
                makespan_it++;
                try {
                    pot_robots[i].move();
                }
                catch (...) {
                    std::cout << "Cannot find trajectory for robot#" << i;
                    return std::forward_as_tuple(-1, -1);
                }
            }
#if PRINT_POTENTIAL_MAP > 1
            if (i == ROBOT_POTENTIAL_MAP_INDEX)
                pot_robots[i].print_potential_map();
#endif
            Sleep(REFRESH_TIME);
            all_finish &= pot_robots[i].finish;
        }
#if PRINT_MAP
        print_map(Map, robots, goals);
#endif
        if (all_finish)
            break;
    }
    return std::forward_as_tuple(makespan_it, flowtime_it);
}

int main() {
    Point **map = new Point*[MAP_SIZE_X],
          *goals = new Point[ROBOTS_COUNT];
    Robot *robots = new Robot[ROBOTS_COUNT];
    std::ifstream stream("input.txt");
    input(goals, stream);
    input(robots, stream);
    for (int i = 0; i < ROBOTS_COUNT; i++)
        robots[i].goal = goals[i];
    input(map, stream);
    stream.close();
    print_map(map, robots, goals);

    int makespan_it, flowtime_it = 0;
    std::tie(makespan_it, flowtime_it) = potential_field(map, robots, goals);
    std::cout << "PotentialFields:\nMakespan = " << makespan_it << "\nFlowtime = " << flowtime_it;

    delete[] map[0];
    delete[] map;
    delete [] goals;
    delete [] robots;
}

