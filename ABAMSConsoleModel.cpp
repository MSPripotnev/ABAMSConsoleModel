/*
* Бригада:
*   - Волков Виктор
*   - Панфилов Александр
*   - Припотнев Михаил
*/
#include <chrono>
#include <fstream>
#include <iostream>
#include <tuple>

#include "astar.hpp"
#include "input.hpp"
#include "map.hpp"
#include "potential-field.hpp"
#include "windows.h"

#define PRINT_MAP 0
#define PRINT_POTENTIAL_MAP 0
#define ROBOT_POTENTIAL_MAP_INDEX 1
#define REFRESH_TIME 400

void output(const char* type, int makespan_it, int makespan_ms, int flowtime_it, int flowtime) {
    std::cout << type << ":" << std::endl <<
        "- Makespan = " << makespan_it << " it\n\t\t" <<
        makespan_ms << " ms" << std::endl <<
        "- Flowtime = " << flowtime_it << " it\n\t\t" <<
        flowtime << " ms" << std::endl;
}

std::tuple<int, int, int> potential_field(Point **map, Robot* robots, Point *goals) {
    int makespan_ms = 0, makespan_it = 0, flowtime_it = 0;
    PotentialRobot* pot_robots = new PotentialRobot[ROBOTS_COUNT];
    for (int i = 0; i < ROBOTS_COUNT; i++)
        pot_robots[i] = PotentialRobot(robots[i], map, robots);
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
        system("cls");
        print_map(map, robots, goals);
#endif
        if (all_finish)
            break;
    }
    return std::forward_as_tuple(makespan_it, flowtime_it, makespan_ms);
}

std::tuple<int, int, int> astar_system(Point** map, Robot* robots, Point* goals) {
    AStarSystem ASystem = AStarSystem(&robots);
    int makespan_ms = 0, makespan_it = 0, flowtime_it = 0;
    while (!ASystem.finish) {
        flowtime_it++;
        ASystem.move();
    }
    for (auto &node : ASystem.trajectory) {
        for (int i = 0; i < ROBOTS_COUNT; i++)
            robots[i].position = &node.robot_positions[i];
#if PRINT_MAP
        print_map(map, robots, goals);
        Sleep(REFRESH_TIME);
#endif
    }
    //for (int i = 0; i < ROBOTS_COUNT; i++)
    //    makespan_ms += ASystem.robots[i]->makespan_ms;
    return std::forward_as_tuple(ASystem.makespan_it, flowtime_it, 0);
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

    int makespan_it, flowtime_it = 0, makespan_ms = 0, flowtime = 0;
    auto start = std::chrono::steady_clock::now();
    //std::tie(makespan_it, flowtime_it, makespan_ms) = potential_field(map, robots, goals);
    flowtime = (std::chrono::steady_clock::now() - start).count();
    //output("PotentialFields", makespan_it, makespan_ms, flowtime_it, flowtime);

    start = std::chrono::steady_clock::now();
    std::tie(makespan_it, flowtime_it, makespan_ms) = astar_system(map, robots, goals);
    flowtime = (std::chrono::steady_clock::now() - start).count();
    output("AStarSystem", makespan_it, makespan_ms, flowtime_it, flowtime);

    delete[] map[0];
    delete[] map;
    delete [] goals;
    delete [] robots;
}

