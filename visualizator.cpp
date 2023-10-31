#define _CRT_SECURE_NO_WARNINGS
#include "visualizator.hpp"

#include <string>
#include <iostream>
#include <fstream>
///////////////////////////////////////////////
/*Использовал инструмент Graphiz для графа, он использует свой язык в файле graph.dot и строит по нему график
и сохраняет график как картинку graph.dot.png в проекте. Библиотека офстрим для работы с файлами*/
////////////////////////////////////////////////////////
struct points
{
    int position[ROBOTS_COUNT * 2];
    points(MultiAStarNode parent) {
        for (int i = 0; i < ROBOTS_COUNT * 2; i++)
            position[i] = i % 2 ? parent.robot_positions[i % ROBOTS_COUNT].x : parent.robot_positions[i % ROBOTS_COUNT].y;
    }
};

void graph_to_dot(std::set<MultiAStarNode> openedPoints) {
    std::vector<points> parents;
    std::vector<points> children;
    for (auto& i : openedPoints) {
        for (const MultiAStarNode* iparent = &i; iparent != nullptr; iparent = iparent->parent) {
            parents.push_back(*iparent);
            children.push_back(*iparent);
        }
        parents.push_back(i);
        children.push_back(i);
    }

    std::string graphStr = "digraph {"; // начало файла .дот по которому будет строится граф
    for (int i = 0; i < parents.size(); i++) // цикл до конца массива детей/родителей
    {
        char buffer[1024];
        graphStr.push_back('"');
        for (int j = 0; j < ROBOTS_COUNT * 2; j++) {
            graphStr.append(_itoa(parents[i].position[j], buffer, 10));
            if (j % 2) {
                graphStr.push_back(';');
            }
            else {
                graphStr.append(" / ");
            }
        }
        graphStr.append("\"->\"");
        for (int j = 0; j < ROBOTS_COUNT * 2; j++) {
            graphStr.append(_itoa(parents[i].position[j], buffer, 10));
            if (j % 2) {
                graphStr.push_back(';'); 
            }
            else {
                graphStr.append(" / ");
            }
            graphStr.append(_itoa(parents[i].position[j], buffer, 10));
        }
        graphStr.append("\"; "); // пробел между связями
    }
    graphStr.push_back('}'); // скобка закрывающая граф

    std::ofstream fout; // объект для работы с файлами
    fout.open("graph.dot"); // создал/открыл файл .дот
    if (!fout.is_open()) // проверка открылся ли файл
        std::cout << "Ошибка: файл не найден" << std::endl;
    else
        fout << graphStr; // если открылся записываю граф в файл .дот

    fout.close(); // закрываю поток работы с файлом

    system("dot -Tpng -O graph.dot"); // создаю файл png с рисунком нашего графа
}