#include <queue>
#include <stack>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "planning.h"

#include <queue>
#include <vector>


void printPath(std::vector<int>& path, Graph& g) {

    if (path.size() < 1)
    {
        std::cout << "No path found :(\n";
        return;
    }

    std::cout << "Path: ";
    for (int i = 0; i < path.size() - 1; i++)
    {
        std::cout << g.data[path[i]] << " -> ";
    }
    std::cout <<  g.data[path.back()] << "\n";
};

std::vector<int> tracePath(int n, Graph& g) {
    std::vector<int> path;
    int curr = n;
    do {
        path.push_back(curr);
        curr = getParent(curr, g);
    } while (curr != -1);

    // Since we built the path backwards, we need to reverse it.
    std::reverse(path.begin(), path.end());
    return path;
};

std::vector<int> getNeighbors(int n, Graph& g)
{
    return g.edges[n];
}

std::vector<float> getEdgeCosts(int n, Graph& g)
{
    return g.edge_costs[n];
}

int getParent(int n, Graph& g)
{
    // *** Task: Implement this function *** //
    if (n>=0 && n< g.nodes.size()) {
        return g.nodes[n].parent;
    }
    else { 
        return -1;
    }

    // *** End student code *** //
}

void initGraph(Graph& g)
{
    g.nodes.clear();

    // Iterate over each node in the graph's data
    for (size_t i = 0; i < g.data.size(); ++i) {
        // Create a new node with default values
        Node n;
        n.city = g.data[i];  // Assign the node's name
        n.visited = false;   // Mark the node as unvisited
        n.parent = -1;       // Set the parent to -1 (no parent)

        // Add the node to the graph's nodes vector
        g.nodes.push_back(n);
    }
}

std::vector<int> bfs(int start, int goal, Graph& g) {
    std::queue<int> frontier;  
    std::vector<int> path;     

    initGraph(g);

    frontier.push(start);
    g.nodes[start].visited = true;

    while (!frontier.empty()) {
        int current = frontier.front(); 
        frontier.pop();                

        if (current == goal) {
            int node = goal;
            while (node != -1) {      
                path.push_back(node);
                node = g.nodes[node].parent;
            }
            std::reverse(path.begin(), path.end()); 
            return path;
        }

        for (int neighbor : g.edges[current]) {
            if (!g.nodes[neighbor].visited) {      
                g.nodes[neighbor].visited = true;  
                g.nodes[neighbor].parent = current; 
                frontier.push(neighbor);          
            }
        }
    } 
    // *** End student code *** //

    return path;
}


std::vector<int> dfs(int start, int goal, Graph& g)
{
    initGraph(g);
    std::vector<int> path;

    std::stack<int> visit_stack;

    // *** Task: Implement this function if completing the advanced extension *** //

    // *** End student code *** //

    return path;
}
