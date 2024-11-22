#include <iostream>
#include <cmath>
#include <queue>
#include <functional>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/graph_search.h>
#include <queue>
#include <vector>

/**
 * General graph search instructions:
 *
 * First, define the correct data type to keep track of your visited cells
 * and add the start cell to it. If you need to initialize any properties
 * of the start cell, do that too.
 *
 * Next, implement the graph search function. Save the result in the path
 * variable defined for you.
 *
 * To visualize which cells are visited in the navigation webapp, save each
 * visited cell in the vector in the graph struct as follows:
 *      graph.visited_cells.push_back(c);
 * where c is a Cell struct corresponding to the visited cell you want to
 * visualize.
 *
 * The tracePath() function will return a path (which you should assign to
 * the path variable above) given the goal index, if you have kept track
 * of the parent of each node correctly and have implemented the
 * getParent() function. If you do not find a path, return an empty path
 * vector.
*/

std::vector<Cell> depthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    // *** Task: Implement this function if completing the advanced extensions *** //

    // *** End student code *** //

    return path;
}

std::vector<Cell> breadthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; 

    initGraph(graph); 

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    

    std::queue<int> q; 
    q.push(start_idx);

    graph.cell_nodes[start_idx].visited = true;

    while (!q.empty()) {
        int current_idx = q.front();
        graph.cell_nodes[current_idx].visited = true;
        q.pop();

        if (current_idx == goal_idx) {
            path = tracePath(current_idx, graph);
            return path; 
            break;
        }

        Cell current_cell = idxToCell(current_idx, graph);
        std::vector<int> neighbor_idx = findNeighbors(current_idx, graph);
        for (int x = 0; x<neighbor_idx.size(); x++) {
            if (!graph.cell_nodes[neighbor_idx[x]].visited) { //consider if the cell is occupied and consider if the cell is already in the visit queue
                graph.cell_nodes[neighbor_idx[x]].visited = true; //mark that its been added to the queue
                q.push(neighbor_idx[x]);
                
                Cell neighbor_cell = idxToCell(neighbor_idx[x], graph);
                float dx = neighbor_cell.i - goal.i;
                float dy = neighbor_cell.j - goal.j;
                float distance_to_goal = std::sqrt(dx * dx + dy * dy); // only update distance and parent if the distance would be smaller

                graph.cell_nodes[neighbor_idx[x]].distance = distance_to_goal;
                graph.cell_nodes[neighbor_idx[x]].parent = current_idx;


                
            }
        }
    }
    // *** End student code *** //

   return path;
}

std::vector<Cell> iterativeDeepeningSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    // *** Task: Implement this function if completing the advanced extensions *** //

    // *** End student code *** //

    return path;
}

std::vector<Cell> aStarSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    // *** Task: Implement this function if completing the advanced extensions *** //

    // *** End student code *** //

    return path;
}
