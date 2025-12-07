#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "graph.h"

#include <string>
#include <vector>

struct PathResult
{
    std::vector<std::string> path;
    double cost{0.0};
    bool reachable{false};
    bool negativeCycle{false};
};

PathResult dijkstra(const Graph &graph, const std::string &source, const std::string &destination);
PathResult bellmanFord(const Graph &graph, const std::string &source, const std::string &destination);
PathResult greedyWalk(const Graph &graph, const std::string &source, const std::string &destination);

#endif // ALGORITHMS_H
