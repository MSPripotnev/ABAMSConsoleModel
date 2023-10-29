/*
* Бригада:
*   - Волков Виктор
*   - Панфилов Александр
*   - Припотнев Михаил
*/
#include <fstream>
#include <iostream>
#include <tuple>
#include <chrono>

#include "input.hpp"
#include "map.hpp"
#include "windows.h"
#include "potential-field.hpp"

#define PRINT_MAP 1
#define PRINT_POTENTIAL_MAP 0
#define ROBOT_POTENTIAL_MAP_INDEX 1
#define REFRESH_TIME 1

std::tuple<int, int, int> potential_field(Point **Map, Robot* robots, Point *goals) {
    int makespan_ms = 0, makespan_it = 0, flowtime_it = 0;
    PotentialRobot* pot_robots = new PotentialRobot[ROBOTS_COUNT];
    for (int i = 0; i < ROBOTS_COUNT; i++)
        pot_robots[i] = PotentialRobot(robots[i], Map, robots);
    while (true) {
        bool all_finish = true;
        for (int i = 0; i < ROBOTS_COUNT; i++) {
            flowtime_it++;
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
                    return std::forward_as_tuple(-1, -1, -1);
                }
            }
            else if (makespan_ms < pot_robots[i].makespan_ms)
                makespan_ms += pot_robots[i].makespan_ms;
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
    return std::forward_as_tuple(makespan_it, flowtime_it, makespan_ms);
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

    int makespan_it, flowtime_it = 0, makespan_ms = 0;
    auto start = std::chrono::steady_clock::now();
    std::tie(makespan_it, flowtime_it, makespan_ms) = potential_field(map, robots, goals);
    auto potential_time = std::chrono::steady_clock::now();
    std::cout << "PotentialFields:" << std::endl <<
        "Makespan = " << makespan_it << " it" << std::endl <<
        makespan_ms << " ms" << std::endl <<
        "Flowtime = " << flowtime_it << " it" << std::endl <<
        std::chrono::duration_cast<std::chrono::milliseconds>(potential_time - start).count() << " ms";

    delete[] map[0];
    delete[] map;
    delete [] goals;
    delete [] robots;
}

