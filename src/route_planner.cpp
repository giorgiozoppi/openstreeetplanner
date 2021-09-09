#include "route_planner.h"
#include <algorithm>
#include <deque>
#include <vector>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node: current_node->neighbors) {
        if (!node->visited) {
            node->parent = current_node;
            node->h_value = CalculateHValue(node);
            node->g_value = current_node->g_value + current_node->distance(*node);    
            open_list.push_back(node);
            node->visited = true;   
        }
    }
 
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(begin(open_list), end(open_list), [](auto n1, auto n2) {
        auto first= n1->g_value +  n1->h_value;
        auto second = n2->g_value + n2->h_value;
        return first < second;
    });   
    auto n = open_list[0];
    open_list.erase(open_list.begin());
    return n;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    // note that vector here is the wrong abstraction.
    
    distance = 0.0f;
    std::deque<RouteModel::Node> path_found;
    start_node->parent = nullptr;
    for (auto current = current_node; current != nullptr;) {
        RouteModel::Node path_node{*current};
        if (current->parent != nullptr) {
            distance += current->distance(*(current->parent));
        }
        path_found.emplace_front(path_node);
        current = current->parent;
    }
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.     
    return std::vector<RouteModel::Node>(begin(path_found), end(path_found));
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    start_node->h_value = CalculateHValue(end_node);
    start_node->visited = true;
    open_list.push_back(start_node);
    while(open_list.size() != 0) {
        current_node = NextNode();
        if (current_node == end_node){
            m_Model.path = ConstructFinalPath(end_node);    
            return;
        }
        AddNeighbors(current_node);
    }
}
 