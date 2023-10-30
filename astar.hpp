#pragma once
#include "robot.hpp"
#include "point.hpp"
#include "map.hpp"
#include <set>

struct MultiAStarNode {
	bool blocked = false;
	Point robot_positions[ROBOTS_COUNT];
	MultiAStarNode* parent = nullptr;
	int distance = 0, heuristic = 0;
	MultiAStarNode() {}
	MultiAStarNode(Robot **robots) {
		for (int i = 0; i < ROBOTS_COUNT; i++) {
			robot_positions[i] = *(*robots)[i].position;
			heuristic += distance_to(*(*robots)[i].position, (*robots)[i].goal);
		}
	}
	MultiAStarNode(const MultiAStarNode &other) {
		blocked = other.blocked;
		parent = other.parent;
		for (int i = 0; i < ROBOTS_COUNT; i++)
			robot_positions[i] = other.robot_positions[i];
		distance = other.distance;
		heuristic = other.heuristic;
	}
	MultiAStarNode(MultiAStarNode &node, Point positions[], MultiAStarNode goal) {
		parent = &node;
		distance = parent->distance;
		for (int i = 0; i < ROBOTS_COUNT; i++) {
			robot_positions[i] = positions[i];
			distance += distance_to(parent->robot_positions[i], robot_positions[i]);
			heuristic += distance_to(robot_positions[i], goal.robot_positions[i]);
		}
	}
	friend bool operator ==(MultiAStarNode node1, MultiAStarNode node2) {
		bool res = true;
		for (int i = 0; i < ROBOTS_COUNT; i++)
			res &= node1.robot_positions[i] == node2.robot_positions[i];
		return res;
	}
	friend bool operator !=(MultiAStarNode node1, MultiAStarNode node2) {
		return !(node1 == node2);
	}
	friend bool operator <(MultiAStarNode node1, MultiAStarNode node2) {
		// if !(node1 < node2) && !!(node 1 < node2) => node1==node2
		// if (node1 < node2) && !(node1 < node2) => invalid comparator
		if (node1.distance + node1.heuristic == node2.distance + node2.heuristic) {
			if (node1.heuristic != node2.heuristic)
				return node1.heuristic < node2.heuristic;
			if (node1.distance != node2.distance)
				return node1.distance < node2.distance;
			for (int i = 0; i < ROBOTS_COUNT; i++) {
				if (node1.robot_positions[i].x != node2.robot_positions[i].x)
					return node1.robot_positions[i].x < node2.robot_positions[i].x;
				if (node1.robot_positions[i].y != node2.robot_positions[i].y)
					return node1.robot_positions[i].y < node2.robot_positions[i].y;
			}
			return false;
		}
		return (node1.distance + node1.heuristic) < (node2.distance + node2.heuristic);
	}
	friend bool operator <=(MultiAStarNode node1, MultiAStarNode node2) {
		return node1.distance + node1.heuristic <= node2.distance + node2.heuristic;
	}
};

class AStarSystem : Robot {
public:
	int makespan_it[ROBOTS_COUNT] = { 0, };
    bool finish = false;
	MultiAStarNode* position;
	MultiAStarNode goal;
	Point** map;
	Robot** robots;
	std::vector<MultiAStarNode> trajectory;
    void move() override;
	AStarSystem() {
		position = nullptr;
		map = new Point * [MAP_SIZE_X];
		robots = new Robot * [ROBOTS_COUNT];
	}
	AStarSystem(Point **_map, Robot **_robots) : AStarSystem() {
		for (int i = 0; i < MAP_SIZE_X; i++)
			map[i] = _map[i];
		for (int i = 0; i < ROBOTS_COUNT; i++) {
			robots[i] = new Robot((*_robots)[i]);
			goal.robot_positions[i] = (*_robots)[i].goal;
		}
		position = new MultiAStarNode(_robots);
	}
private:
	std::set<MultiAStarNode> openedPoints, closedPoints;
	std::chrono::steady_clock::time_point start;
	MultiAStarNode *result = nullptr;
	void open_points();
	MultiAStarNode select_direction();
	void finalize();
};