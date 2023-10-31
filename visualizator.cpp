#define _CRT_SECURE_NO_WARNINGS
#include "visualizator.hpp"

#include <string>
#include <iostream>
#include <fstream>
///////////////////////////////////////////////
/*����������� ���������� Graphiz ��� �����, �� ���������� ���� ���� � ����� graph.dot � ������ �� ���� ������
� ��������� ������ ��� �������� graph.dot.png � �������. ���������� ������� ��� ������ � �������*/
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

    std::string graphStr = "digraph {"; // ������ ����� .��� �� �������� ����� �������� ����
    for (int i = 0; i < parents.size(); i++) // ���� �� ����� ������� �����/���������
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
        graphStr.append("\"; "); // ������ ����� �������
    }
    graphStr.push_back('}'); // ������ ����������� ����

    std::ofstream fout; // ������ ��� ������ � �������
    fout.open("graph.dot"); // ������/������ ���� .���
    if (!fout.is_open()) // �������� �������� �� ����
        std::cout << "������: ���� �� ������" << std::endl;
    else
        fout << graphStr; // ���� �������� ��������� ���� � ���� .���

    fout.close(); // �������� ����� ������ � ������

    system("dot -Tpng -O graph.dot"); // ������ ���� png � �������� ������ �����
}