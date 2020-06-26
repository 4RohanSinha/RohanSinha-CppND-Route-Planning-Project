#include "route_planner.h"
#include "route_model.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y); //start node points to the closest node to start_x and start_y
    end_node = &m_Model.FindClosestNode(end_x, end_y); //end node points to the closest node to end_x and end_y

}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node); //get distance from node to the end node 
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	//populate current_node.neighbors vector with neighbors
	current_node->FindNeighbors();

	//iterate through all neighbors
	for (auto i: current_node->neighbors) {
		//add to open list only if it has not been visited
		if (!i->visited) {
			i->parent = current_node; //the parent of the neighbor node is the current_node
			i->h_value = CalculateHValue(i); //assign h
			//assign g
			i->g_value = current_node->g_value + current_node->distance(*i);
			i->visited = true; //mark the neighbor node as visited
			open_list.push_back(i); //add the neighbor node to the open list
		}
	}
}

//compare two RouteModel::Node objects for sorting in NextNode()
//the objects are compared by the sums of their g and h values
bool compareNodes(const RouteModel::Node* a, const RouteModel::Node* b) {
	//will return a boolean comparing the two objects' g and h values
	return (a->g_value + a->h_value) > (b->g_value + b->h_value);
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
	
	//sort the nodes in the open list by using the compareNodes() function above
	//the open list is sorted in descending order so that the lowest sum can be removed with pop_back()
	std::sort(open_list.begin(), open_list.end(), &compareNodes);
	
	//create a pointer to the last node in the open list - since it is in descending order, the last element will have the lowest sum
	RouteModel::Node* leastNode = open_list.back();
	
	//pop that node from the open list
	open_list.pop_back();

	//return this node
	return leastNode;
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

    // TODO: Implement your solution here.
 
    // the parent of the start node is null so continue looping until the loop reaches the parent of the start node, which is null
    while (current_node != nullptr) {

	    //include if statement to check if current node's parent is nullptr, just like the start node
	    //this statement makes sure that the parent is not null before adding the distance between the node and the parent
	    //for example, the start node's parent is null, so the distance from that node to its parent should not be added
	    //do not break out of the loop here because the start node still needs to be added to the path found vector
	    if (current_node->parent != nullptr) {

		//add the distance between the current node and the parent
	    	distance += current_node->distance(*(current_node->parent));
	    }

	    //add the current node to the path found vector
	    path_found.push_back(*current_node);

	    //go to the parent of the current node
	    current_node = current_node->parent;
    }

    //reverse the path found vector
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    //return the path found vector
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    //the current node for the while loop
    RouteModel::Node *current_node = start_node;

    // TODO: Implement your solution here.
    current_node->visited = true; //mark the start node as visited

    open_list.push_back(current_node); //add the start node to the open list
    
    while (open_list.size() > 0) { //ensure that there are elements in the open list for NextNode()

	    current_node = NextNode(); //get the node with the lowest g and h sum in the open list

	    if (current_node->distance(*end_node) == 0.0) { //if the algorithm has found the end node (the distance between the current node and end node is zero)

		    //construct the final path
		    m_Model.path = ConstructFinalPath(current_node);

		    return; //return (essentially breaking out of the loop)

	    }

	    //if the end node has not been reached, add neighbors to the open list
	    AddNeighbors(current_node);
    }
}
