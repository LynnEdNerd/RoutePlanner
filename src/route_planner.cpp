#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node =  &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return (node->distance( *(this->end_node) ));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    RouteModel::Node *current_neighbor;

    for (RouteModel::Node *current_neighbor : current_node->neighbors) {
        // If the node has not been previously visited, update the g, h, parent values and put it on the open list.
        if (!current_neighbor->visited) {
            current_neighbor->g_value = current_node->g_value + current_node->distance(*current_neighbor);
            current_neighbor->h_value = this->CalculateHValue (current_neighbor);
            current_neighbor->parent = current_node;
            current_neighbor->visited = true;
            this->open_list.push_back(current_neighbor);
  
        }
    }
}

// This is a helper function that is used with the standard library "sort" method.  
// The return is true if g value + h value of node1 > g value + h value of node2,
// otherwise falst
//
bool NodeCompare( RouteModel::Node *Node1, RouteModel::Node *Node2 ) {
    bool return_value = false;

    if ((Node1->g_value + Node1->h_value) > (Node2->g_value + Node2->h_value)) {
        return_value = true;
    }

    return(return_value);
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *ReturnValue;

    // Sort the list so that the node with the smallest g value + h value is at the back of the list.
    std::sort(open_list.begin( ), open_list.end(), NodeCompare);
    ReturnValue = open_list.back();
    open_list.pop_back();
    return(ReturnValue);
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
 
    // Since the target or end_node has been found, traverse the parent nodes back to start_node.  Accumulating the
    // nnodes in the path_found variable.
    do  {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    } while (current_node != start_node);

    // Push the start_node to the back of the list
    path_found.push_back(*current_node);

    // The path_found vector has the target node as its first node and the starting node as its
    // final node.  This is the reverse order from what is desired.  Thus the reverse.
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Take care of the boundary conditons with start_node (i.e., visited = true and g_value = 0).
    // Then seed the open list with start_node.
    start_node->visited = true;
    start_node->g_value = 0;
    open_list.push_back(start_node);

    while (open_list.size() > 0 ) {
        current_node = NextNode();
        if (current_node == end_node) {
            // The target has been found!  Now construct the path from start to end.
            m_Model.path = ConstructFinalPath( current_node);
            break;
        } 

        AddNeighbors(current_node);
    }
}
