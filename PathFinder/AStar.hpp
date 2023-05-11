#ifndef GRAPH_ASTAR_HPP_INCLUDED
#define GRAPH_ASTAR_HPP_INCLUDED

#include <map>
#include <set>
#include <list>
#include <vector>

#include "NodeAdapter.hpp"
#include "GraphAdapter.hpp"

typedef psramvec<psramvec<int>> GraphType;

class Point
{
public:
    int16_t x, y, id = 0;
    Point() {}
    Point(int16_t inx, int16_t iny) { x = inx; y = iny; }
    Point(int16_t inx, int16_t iny, int16_t inid) { x = inx; y = iny; id = inid; }
};

class PathFinder
{
public:
    psramvec<Point> getPath(GraphType &input_map, int32_t width, int32_t height, Point* start, Point* end);
};

namespace Graph {

    template<typename _MapType, typename _PositionType, typename _CostType = int>
    class AStar {
        public:
            typedef _MapType                        MapType;
            typedef _PositionType                   PositionType;
            typedef _CostType                       CostType;
            typedef std::list<PositionType>         PathType;

            typedef NodeAdapter<PositionType, CostType> NodeAdapterType;
            typedef GraphAdapter<MapType, PositionType, CostType> GraphAdapterType;


        public:
            AStar() {

            };

            PathType getPath(const GraphAdapterType& graphAdapter, const NodeAdapterType& start, const NodeAdapterType& goal) const {
                PathType resultPath;

                struct NodePositionComparator {
                    bool operator() (const NodeAdapterType& lhs, const NodeAdapterType& rhs) {
                        return lhs.position < rhs.position;
                    }
                };

                std::set<NodeAdapterType> open = { start };
                std::set<NodeAdapterType, NodePositionComparator> closed;
                std::map<NodeAdapterType, NodeAdapterType, NodePositionComparator> came_from;
                typename std::set<NodeAdapterType>::iterator current;

                while(open.empty() == false) {
                    current = open.begin();

                    if(current->position == goal.position) {
                        // Goal found, recreating path from 'goal' node to 'start' node
                        auto pathCurrent = came_from.find(*current);
                        if(pathCurrent != came_from.end()) {
                            while(pathCurrent->second.position != start.position) {
                                resultPath.push_front(pathCurrent->second.position);
                                pathCurrent = came_from.find(pathCurrent->second);
                            }
                        }

                        return resultPath;
                    }

                    closed.insert(*current);

                    /*psramvec<NodeAdapterType> neighbours = graphAdapter.getNeighboursOf(*current);
                    for(NodeAdapterType& neighbour : neighbours) {
                        if(graphAdapter.isAvailable(neighbour.position)) {
                            typename std::set<NodeAdapterType>::iterator cIter, oIter;

                            cIter = closed.find(neighbour);
                            if(cIter != closed.end()) {
                                continue;
                            }

                            CostType hypotheticalNeighbourCostG = current->g() + 1;

                            oIter = open.find(neighbour);
                            if(oIter == open.end() || hypotheticalNeighbourCostG < oIter->g()) {
                                if(oIter != open.end()) {
                                    open.erase(oIter);
                                }

                                auto cameFromIter = came_from.find(neighbour);
                                if(cameFromIter != came_from.end())
                                    came_from.erase(cameFromIter);
                                came_from.emplace(std::pair<NodeAdapterType, NodeAdapterType>(neighbour, *current));

                                neighbour.g(hypotheticalNeighbourCostG);
                                neighbour.h(graphAdapter.getHeuristicCostLeft(neighbour, goal));
                                open.insert(neighbour);
                            }
                        }
                    }*/

                    open.erase(current);
                }

                // If algorithm comes here, no path was found, and return value of this method will be empty vector

                return resultPath;
            }
    };

}

#endif
