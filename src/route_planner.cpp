#include "route_planner.h"
#include <algorithm>

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
    auto h = node->distance(*end_node);
    return h;

}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for(auto neighboring_node : current_node->neighbors) {
        neighboring_node->parent = current_node;
        neighboring_node->h_value = CalculateHValue(neighboring_node);
        neighboring_node->g_value = current_node->g_value + current_node->distance(*neighboring_node);

        open_list.push_back(neighboring_node);
        neighboring_node->visited = true;
    }

}

RouteModel::Node *RoutePlanner::NextNode() {
  	std::sort(open_list.begin(), open_list.end(), [](const auto &a, const auto &b) {return ((a->g_value + a->h_value) > (b->g_value + b->h_value)); } );

    int last_in_list = open_list.size() - 1;
    auto lowest_node = open_list[last_in_list];

    open_list.pop_back();
    return lowest_node;

}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    while(current_node->parent != nullptr) {
        distance += current_node->distance(*current_node->parent);

        path_found.insert(path_found.begin(), *current_node); // because we want to pass the pointer to the corrent node

        current_node = current_node->parent;
    }
  	path_found.insert(path_found.begin(), *current_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.push_back(start_node);

    while(open_list.size() != 0) {
        current_node = NextNode();

        if(current_node->distance(*end_node) != 0) {
            AddNeighbors(current_node);
        } else {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
    }
}
