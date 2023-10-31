#pragma once
#include "map.hpp"
#include <fstream>

void input(int* method, std::istream& stream);
void input(Point* goals, std::istream& stream);
void input(Robot* robots, std::istream& stream);
void input(Point** Map, std::istream& stream);
