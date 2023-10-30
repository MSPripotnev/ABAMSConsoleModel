#pragma once

struct Point {
    int x = 0, y = 0;
    bool blocked = false;
    Point() {}
    Point(int _x, int _y) {
        x = _x;
        y = _y;
        blocked = false;
    }
    Point(int _x, int _y, bool _blocked) {
        x = _x;
        y = _y;
        blocked = _blocked;
    }
    Point(const Point& point) {
        x = point.x;
        y = point.y;
        blocked = point.blocked;
    }
    friend bool operator==(Point p1, Point p2) {
        return p1.x == p2.x && p1.y == p2.y;
    }
    friend bool operator!=(Point p1, Point p2) {
        return ! (p1 == p2);
    }
    friend Point operator-(Point p1, Point p2) {
        return *(new Point(p1.x - p2.x, p1.y - p2.y));
    }
};

bool point_is_out_of_range(Point p);
int distance_to(Point p1, Point p2);
int distance_to_near(Point* goals, Point* p);
