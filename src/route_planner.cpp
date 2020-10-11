#include "route_planner.h"
#include <iostream>
#include <algorithm>

using std::reverse;
using std::sort;
using std::cout;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto &neighbor: current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }

}

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b)
{
    float f1 = a->g_value + a->h_value;
    float f2= b->g_value + b->h_value;
    return f1 > f2;

}


RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), Compare);
    
    RouteModel::Node* ptr = open_list.back();
    open_list.pop_back();
    return ptr; 
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Add initial node:
    path_found.push_back(*current_node);

    // Create a pointer to hold node addresses:
    RouteModel::Node* temp_node = current_node;

    while(temp_node->parent != nullptr)
    {
        RouteModel::Node* parent = temp_node->parent;
        distance += temp_node->distance(*parent);
        path_found.push_back(*parent);
        temp_node = parent;
    }

    // Reverse the vector:
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->h_value = CalculateHValue(start_node);
    start_node->g_value=0;
    start_node -> visited = true;
    start_node -> parent = nullptr;
  
    open_list.push_back(start_node);

    while(open_list.size() >0)
    {
        current_node = NextNode();

        // Goal found:
        if(current_node->x == end_node->x && current_node->y == end_node->y)
        {
            m_Model.path=  ConstructFinalPath(current_node);
            break;
        }

        AddNeighbors(current_node);

    }

}