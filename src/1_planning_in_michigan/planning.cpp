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
    return g.parent[n];
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
    visit_queue.push(start);
    g.visited[start] = true;
    while(!visit_queue.empty()) {
        int curr = visit_queue.front();
        visit_queue.pop();

        if (curr == goal) {
            for (int t = goal; t != -1; t = g.parent[t]) {
                path.push_back(t);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (int neighbor : g.edges[curr]) {
            if (!g.visited[neighbor]) {
                g.visited[neighbor] = true;
                g.parent[neighbor] = curr;
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
    g.visited[start] = true;
    while (!visit_stack.empty()) {
            int curr = visit_stack.top();
            visit_stack.pop();

            if (curr == goal) {
                // Trace the path from goal to start
                for (int t = goal; t != -1; t = g.parent[t]) {
                    path.push_back(t);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int neighbor : g.edges[curr]) {
                if (!g.visited[neighbor]) {
                    g.visited[neighbor] = true;
                    g.parent[neighbor] = curr;
                    visit_stack.push(neighbor);
                }
            }
        }

    // *** End student code *** //

    return path;
}
