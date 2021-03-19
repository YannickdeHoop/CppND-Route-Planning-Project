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
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    if (!neighbor->visited)
    {
      neighbor->h_value = CalculateHValue(neighbor);
      neighbor->g_value = neighbor->distance(*current_node) + current_node->g_value;
      neighbor->visited = true;
      neighbor->parent = current_node;
      open_list.push_back(neighbor);
    }
  }
}

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
  float f1 = a->g_value + a->h_value;
  float f2 = b->g_value + b->h_value;
  return f1 > f2;
}

void NodeSort(std::vector<RouteModel::Node*> *v) {
  std::sort(v->begin(), v->end(), Compare);
}

RouteModel::Node *RoutePlanner::NextNode() {
  NodeSort(&open_list);
  RouteModel::Node* node = open_list.back();
  open_list.pop_back();

  return node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found = {*end_node};

    while(current_node->distance(*start_node) != 0)
    {
      RouteModel::Node* parent_node = current_node->parent;
      distance += current_node->distance(*parent_node);
      path_found.insert(path_found.begin(), *parent_node);
      current_node = parent_node;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    start_node->visited = true;
    open_list.push_back(start_node);

    while(!open_list.empty())
    {
      AddNeighbors(current_node);
      current_node = NextNode();

      if(current_node->distance(*end_node) == 0)
      {
         m_Model.path = ConstructFinalPath(current_node);
      }
    }

    // TODO: Implement your solution here.

}
