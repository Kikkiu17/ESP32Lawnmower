#include "AStar.hpp"
#include <cmath>
#include <vector>
#include <iostream>

/**
 * Example of use of AStar class implementint A* pathfind algorithm.
 *
 * This example creates custom graph adapter by inheriting type of Graph::GraphAdapter and implementing it's API.
 */


/* Typedefs */
typedef psramvec<psramvec<int>> GraphType;
typedef std::pair<unsigned int, unsigned int> PositionType; // Position type. In example below its pair of indices (x, y).
typedef double CostType; // Cost type. Represents type of heuristic and non-heuristic functions cost type.

typedef Graph::AStar<GraphType, PositionType, CostType> MyAStar; // Shortcut so you don't have to write template arguents all the time


/**
 * Custom graph adapter - inherit's interface of Graph::GraphAdapter
 */
class MyGraphAdapter : public Graph::GraphAdapter<GraphType, PositionType, CostType> {
    public:
        MyGraphAdapter(GraphType& graph) : GraphAdapter(graph) {

        }

        bool isAvailable(const PositionType& position) const {
            bool isOnMap;
            bool isAvailable = false;

            // check if positions is 'on' our map
            isOnMap = (position.first < _graph.size() && position.second < _graph[0].size());

            // check if it's value is zero
            if(isOnMap)
                isAvailable = (_graph[position.second][position.first] == 0);

            return isAvailable;
        }

        psramvec<NodeAdapterType> getNeighboursOf(const NodeAdapterType& node) const {
            psramvec<MyAStar::NodeAdapterType> neighbours;

            static const psramvec<std::pair<int, int>> coordDiffs = {
                {  0, -1 }, {  0,  1 }, {  1,  0 }, { -1,  0 }
            };

            for(const std::pair<int, int>& coordDiff : coordDiffs) {
                MyAStar::NodeAdapterType neighbour(std::make_pair(node.position.first + coordDiff.first, node.position.second + coordDiff.second));
                neighbours.push_back(neighbour);
            }

            return neighbours;
        }

        CostType getHeuristicCostLeft(const NodeAdapterType& currentNode, const NodeAdapterType& goal) const {
            MyAStar::CostType xCost, yCost;

            xCost = (goal.position.first - currentNode.position.first);
            yCost = (goal.position.second - currentNode.position.second);

            return std::sqrt(xCost*xCost + yCost*yCost);
        }

};

psramvec<Point> PathFinder::getPath(GraphType &input_map, int32_t width, int32_t height, Point* start, Point* end)
{
    typedef std::pair<unsigned int, unsigned int> PositionType; // Position type. In example below its pair of indices (x, y).
    typedef double CostType; // Cost type. Represents type of heuristic and non-heuristic functions cost type.

    typedef Graph::AStar<GraphType, PositionType, CostType> MyAStar; // Shortcut so you don't have to write template arguents all the time

    /**
     * Map
     * Creation and initialization map.
     */
    GraphType map = input_map;

    Serial.printf("\n\n");
    for(unsigned int i = 0; i < map.size(); ++i) {
        for(unsigned int j = 0; j < map[i].size(); ++j) {
            printf("%d ", map[i][j]);
        }
        std::cout << std::endl;
    }

    std::cout << "\nPROCESSING\n" << std::endl;

    /**
     * Code
     * Creation and initialization of AStar instance, mapAdapter for AStar and 'start' and 'goal' nodes.
     */
    MyAStar astar; // Instance of AStar
    MyGraphAdapter graphAdapter(map); // Adapter for our map to use by AStar instance. This type uses inheritance to provide needed functionality.
    MyAStar::NodeAdapterType nodeStart(std::make_pair(start->x, start->y)); // Adapter for start node for AStar instance
    MyAStar::NodeAdapterType nodeGoal(std::make_pair(end->x, end->y)); // Adapter for goal node for ASTar istance


    /**
     * Querying for shortest path
     */
    MyAStar::PathType path = astar.getPath(graphAdapter, nodeStart, nodeGoal);

    // Marks path on our map
    for(PositionType& pos : path) {
        map[pos.second][pos.first] = '.';
    }

    // Prints out map
    map[nodeStart.position.second][nodeStart.position.first] = 'S';
    map[nodeGoal.position.second][nodeGoal.position.first] = 'E';

    for(unsigned int i = 0; i < map.size(); ++i) {
        for(unsigned int j = 0; j < map[i].size(); ++j) {
            if (map[i][j] == 0)
                map[i][j] = '0';
            else if (map[i][j] == 1)
                map[i][j] = '1';
            printf("%c ", map[i][j]);
        }
        std::cout << std::endl;
    }
}
