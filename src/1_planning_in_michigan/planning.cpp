#include <queue>
#include <stack>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "planning.h"


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
    return -1;

    // *** End student code *** //
}

void initGraph(Graph& g)
{
    g.nodes.clear();
    for (int i = 0; i < g.data.size(); i++)
    {
        Node n;
        n.city = g.data[i];
        g.nodes.push_back(n);
    }
}

std::vector<int> bfs(int start, int goal, Graph& g)
{
    initGraph(g);
    std::vector<int> path;

    std::queue<int> visit_queue;

    // *** Task: Implement this function *** //
   
    g.nodes[start].visited = true;
    visit_queue.push(start);

    while (!visit_queue.empty()) {
        int curr = visit_queue.front();
        visit_queue.pop();

        if (curr == goal) {
            int at = goal;
            while (at != -1) {
                path.push_back(at); 
                at = g.nodes[at].parent;
            }
            std::reverse(path.begin(), path.end()); 
            return path;
        }

        for (int neighbor : g.edges[curr]) {
            if (!g.nodes[neighbor].visited) {
                g.nodes[neighbor].visited = true; 
                g.nodes[neighbor].parent = curr; 
                visit_queue.push(neighbor); 
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
