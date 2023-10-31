/*
* Бригада:
*   - Волков Виктор
*   - Припотнев Михаил
*   - Радван Ахмед
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

#define PRINT_MAP 0 // 0 - disable, 1 - A*, 2 - PotentialFields, 3 - Both
#define PRINT_POTENTIAL_MAP 0 // 0 - disable potential map drawing, 1 - draw before move, 2 - draw after, 3 - both
#define ROBOT_POTENTIAL_MAP_INDEX 1 // index of robot to draw his map for
#define REFRESH_TIME 1 // sleep timer (in ms) before images changing

void output(const char* type, int *makespan_it, int *makespan_ms, int flowtime_it, int flowtime) {
    std::cout << type << ":" << std::endl <<
        "- Makespan = (";
    for (int i = 0; i < ROBOTS_COUNT; i++) {
        if (i < ROBOTS_COUNT - 1) {
            std::cout << makespan_it[i] << " it / ";
            if (type != "AStarSystem")
                std::cout << makespan_ms[i] << " us , ";
        }
        else {
            std::cout << makespan_it[i] << " it / ";
            if (type != "AStarSystem")
                std::cout << makespan_ms[i] << " us , ";
            else std::cout << ")\n";
        }
    }
    std::cout << "\n- Flowtime = " << flowtime_it << " it, " << flowtime << " ms" << std::endl;
}

std::tuple<int *, int, int *> potential_field(Point** map, Robot* robots, Point* goals) {
    int flowtime_it = 0;
    int makespan_it[ROBOTS_COUNT] = { 0, }, makespan_ms[ROBOTS_COUNT] = { 0, };
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
                makespan_it[i]++;
                try {
                    pot_robots[i].move();
                    if (flowtime_it > 1000)
                        throw "";
                }
                catch (...) {
                    std::cout << "Cannot find trajectory for robot#" << i;
                    int empty[ROBOTS_COUNT] = { -1, };
                    return std::forward_as_tuple(empty, -1, empty);
                }
            }
            else
                makespan_ms[i] = pot_robots[i].makespan_ms;
#if PRINT_POTENTIAL_MAP > 1
            if (i == ROBOT_POTENTIAL_MAP_INDEX)
                pot_robots[i].print_potential_map();
            Sleep(REFRESH_TIME);
#endif
            all_finish &= pot_robots[i].finish;
        }
#if PRINT_MAP == 2 || PRINT_MAP == 3
        print_map(map, robots, goals);
        Sleep(REFRESH_TIME);
#endif
        if (all_finish)
            break;
    }
    int* makespan_it_copy = new int[ROBOTS_COUNT], *makespan_ms_copy = new int[ROBOTS_COUNT];
    for (int i = 0; i < ROBOTS_COUNT; i++) {
        makespan_it_copy[i] = makespan_it[i];
        makespan_ms_copy[i] = makespan_ms[i];
    }

    return std::make_tuple(makespan_it_copy, flowtime_it, makespan_ms_copy);
}

std::tuple<int *, int> astar_system(Point** map, Robot* robots, Point* goals) {
    AStarSystem ASystem = AStarSystem(map, &robots);
    int flowtime_it = -1;
    while (!ASystem.finish) {
        ASystem.move();
    }
    for (auto &node : ASystem.trajectory) {
        flowtime_it++;
        for (int i = 0; i < ROBOTS_COUNT; i++)
            robots[i].position = &node.robot_positions[i];
#if PRINT_MAP == 1 || PRINT_MAP == 3
        print_map(map, robots, goals);
        Sleep(REFRESH_TIME);
#endif
    }
    int *makespan_it = new int[ROBOTS_COUNT], *makespan_ms = new int[ROBOTS_COUNT];
    for (int i = 0; i < ROBOTS_COUNT; i++) {
        makespan_it[i] = ASystem.makespan_it[i];
    }
    return std::make_tuple(makespan_it, flowtime_it);
}

int main() {
    Point **map = new Point*[MAP_SIZE_X],
          *goals = new Point[ROBOTS_COUNT];
    Robot *robots = new Robot[ROBOTS_COUNT];
    int method;
    std::ifstream stream("input.txt");
    input(&method, stream);
    input(goals, stream);
    input(robots, stream);
    for (int i = 0; i < ROBOTS_COUNT; i++)
        robots[i].goal = goals[i];
    input(map, stream);
    stream.close();
    print_map(map, robots, goals);

    int* makespan_it, flowtime_it = 0, *makespan_ms = nullptr, flowtime = 0, calctime_ms = 0;
    auto start = std::chrono::steady_clock::now();
    if (method == 1) {
        std::tie(makespan_it, flowtime_it) = astar_system(map, robots, goals);
        flowtime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        output("AStarSystem", makespan_it, makespan_ms, flowtime_it, flowtime);
    } else if (method == 2) {
        std::tie(makespan_it, flowtime_it, makespan_ms) = potential_field(map, robots, goals);
        flowtime = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
        output("PotentialFields", makespan_it, makespan_ms, flowtime_it, flowtime);
    } else {
        std::cout << "Not implemented method!";
    }

    delete[] map[0];
    delete[] map;
    delete[] goals;
    delete[] robots;
}
