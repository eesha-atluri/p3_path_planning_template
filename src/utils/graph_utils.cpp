#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>
#include <vector>
#include <cmath>
#include <limits>


bool isLoaded(const GridGraph& graph)
{
    bool correct_size = graph.cell_odds.size() == graph.width * graph.height;
    bool positive_size = graph.width > 0 && graph.height > 0;
    bool positive_m_per_cell = graph.meters_per_cell > 0;
    return correct_size && positive_size && positive_m_per_cell;
}


bool loadFromFile(const std::string& file_path, GridGraph& graph)
{
    std::ifstream in(file_path);
    if (!in.is_open())
    {
        std::cerr << "ERROR: loadFromFile: Failed to load from " << file_path << std::endl;
        return false;
    }

    // Read header
    in >> graph.origin_x >> graph.origin_y >> graph.width >> graph.height >> graph.meters_per_cell;

    // Check sanity of values
    if (graph.width < 0 || graph.height < 0 || graph.meters_per_cell < 0.0f)
    {
        return false;
    }

    graph.collision_radius = ROBOT_RADIUS + graph.meters_per_cell;

    // Reset odds vector.
    int num_cells = graph.width * graph.height;
    graph.cell_odds.clear();
    graph.cell_odds.resize(num_cells);

    graph.obstacle_distances = std::vector<float>(graph.width * graph.height, 0);

    // Read in each cell value
    int odds;  // read in as an int so it doesn't convert the number to the corresponding ASCII code
    for (int idx = 0; idx < num_cells; ++idx)
    {
        in >> odds;
        graph.cell_odds[idx] = odds;
    }

    // Reset the nodes in the graph.
    initGraph(graph);

    return true;
};

std::string mapAsString(GridGraph& graph)
{
    std::ostringstream oss;
    // Header data.
    oss << graph.origin_x << " " << graph.origin_y << " ";
    oss << graph.width << " " << graph.height << " " << graph.meters_per_cell << " ";

    // Cell data.
    for (int j = 0; j < graph.height; j++)
    {
        for (int i = 0; i < graph.width; i++)
        {
            oss << +graph.cell_odds[cellToIdx(i, j, graph)] << " ";
        }
    }

    return oss.str();
}

void initGraph(GridGraph& graph)
{
    // *** Task: Initialize any variables YOU added to the GridGraph *** //
    graph.visited_cells.clear();
    graph.cell_nodes.clear();
    
    int num_cells = graph.width * graph.height;

    graph.cell_nodes.resize(num_cells);

    for (int idx = 0; idx < num_cells; ++idx) {
        CellNode& node = graph.cell_nodes[idx];
        node.x = idx % graph.width;          
        node.y = idx / graph.width;    
        node.parent = -1;                     
        node.occupancy = graph.cell_odds[idx]; 
        node.visited = false;                 
        node.distance = std::numeric_limits<float>::infinity(); // Infinite distance


   

    // *** End student code *** //
}
}


int cellToIdx(int i, int j, const GridGraph& graph)
{
     return i * graph.width + j;
}


Cell idxToCell(int idx, const GridGraph& graph)
{
    Cell c;
    c.i = idx % graph.width;
    c.j = idx / graph.width;
    return c;
}


Cell posToCell(float x, float y, const GridGraph& graph)
{
    int i = static_cast<int>(floor((x - graph.origin_x) / graph.meters_per_cell));
    int j = static_cast<int>(floor((y - graph.origin_y) / graph.meters_per_cell));

    Cell c;
    c.i = i;
    c.j = j;
    return c;
}


std::vector<float> cellToPos(int i, int j, const GridGraph& graph)
{
    float x = (i + 0.5) * graph.meters_per_cell + graph.origin_x;
    float y = (j + 0.5) * graph.meters_per_cell + graph.origin_y;

    return std::vector<float>({x, y});
}


bool isCellInBounds(int i, int j, const GridGraph& graph)
{
   return (i >= 0 && i < graph.height && j >= 0 && j < graph.width);
}


bool isIdxOccupied(int idx, const GridGraph& graph)
{
    return graph.cell_odds[idx] >= graph.threshold;
}


bool isCellOccupied(int i, int j, const GridGraph& graph)
{
    return isIdxOccupied(cellToIdx(i, j, graph), graph);
}


std::vector<int> findNeighbors(int idx, const GridGraph& graph)
{

    std::vector<int> neighbors;

    Cell current = idxToCell(idx, graph);

    // std::cout << current.i << "  " << current.j << std::endl;

    for (int dj = current.j-1; dj <= current.j + 1; dj++) {
        for (int di = current.i-1; di <= current.i+1; di++) {
            if (di!=current.i || dj!=current.j) {
            // int ni = current.i + di; 
            // int nj = current.j + dj;

            //  std::cout << isCellInBounds(ni, nj, graph) << std::endl;

            // std::cout << isIdxOccupied(cellToIdx(di, dj, graph), graph) << std::endl;
            
            // std::cout << di << "  " << dj << "  " << graph.height << "   " << graph.width << std::endl;

                if (isCellInBounds(di, dj, graph)) {
                    int neighborIdx = cellToIdx(di, dj, graph);
                    neighbors.push_back(neighborIdx);
                    // neighbors.push_back(cellToIdx(ni, nj, graph));

                }
            }
        }
    }
  
    return neighbors;
   
    /**
     * NOTE: Be sure the neighbors are returned in the following order:
     * 
     * -1 in i direction, -1 in j direction
     * -1 in i direction, +0 in j direction
     * -1 in i direction, +1 in j direction
     * +0 in i direction, -1 in j direction
     * +0 in i direction, +0 in j direction
     * +0 in i direction, +1 in j direction
     * +1 in i direction, -1 in j direction 
     * +1 in i direction, +0 in j direction
     * +1 in i direction, +1 in j direction
     * 
     * This will be necessary to pass the test cases.
     */

   

    // *** End student code *** //
}  


bool checkCollisionFast(int idx, const GridGraph& graph)
{
    return graph.obstacle_distances[idx] * graph.meters_per_cell <= graph.collision_radius;
}


bool checkCollision(int idx, const GridGraph& graph)
{
    // Check if this cell is in collision.
    if (isIdxOccupied(idx, graph))
    {
        return true;
    }

    // Use arc length of approximately 1 cell width.
    //   arc_length = theta * radius
    double dtheta = graph.meters_per_cell / graph.collision_radius;
    double theta = 0;
    auto c = idxToCell(idx, graph);
    auto state = cellToPos(c.i, c.j, graph);
    while (theta < 2 * PI)
    {
        double x = state[0] + graph.collision_radius * cos(theta);
        double y = state[1] + graph.collision_radius * sin(theta);
        Cell c = posToCell(x, y, graph);

        if (!isCellInBounds(c.i, c.j, graph))
        {
            return true;
        }

        if (isCellOccupied(c.i, c.j, graph))
        {
            return true;
        }

        theta += dtheta;
    }

    return false;
}


int getParent(int idx, const GridGraph& graph)
{
    // *** Task: Implement this function *** //
    return graph.cell_nodes[idx].parent;
    

    // *** End student code *** //
}


float getScore(int idx, const GridGraph& graph)
{
    // *** Task: Implement this function if completing the A* extension *** //
    
    return 0.0;

    // *** End student code *** //
}


int findLowestScore(const std::vector<int>& node_list, const GridGraph& graph)
{
    int min_idx = 0;

    // Find the cell with the lowest score.
    for (int i = 1; i < node_list.size(); ++i)
    {
        if (getScore(node_list[i], graph) < getScore(node_list[min_idx], graph))
        {
            min_idx = i;
        }
    }

    // Return index of the lowest score cell
    return min_idx;
}


std::vector<Cell> tracePath(int goal, const GridGraph& graph)
{
    std::vector<Cell> path;
    int current = goal;
    do
    {
        path.push_back(idxToCell(current, graph));
        current = getParent(current, graph);
    } while (current >= 0);  // A cell with no parent has parent -1.

    // Since we built the path backwards, we need to reverse it.
    std::reverse(path.begin(), path.end());
    return path;
}
