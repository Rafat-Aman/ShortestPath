#include "algorithms.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace
{
    constexpr double kInf = std::numeric_limits<double>::infinity();

    std::vector<std::string> reconstructPath(const std::unordered_map<std::string, std::string> &prev,
                                             const std::string &source,
                                             const std::string &destination)
    {
        std::vector<std::string> path;
        std::string current = destination;
        while (true)
        {
            path.push_back(current);
            if (current == source)
            {
                break;
            }
            const auto it = prev.find(current);
            if (it == prev.end())
            {
                return {};
            }
            current = it->second;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
} // namespace

PathResult dijkstra(const Graph &graph, const std::string &source, const std::string &destination)
{
    PathResult result;
    if (!graph.contains(source) || !graph.contains(destination))
    {
        return result;
    }

    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, std::string> prev;
    using NodeState = std::pair<double, std::string>;
    std::priority_queue<NodeState, std::vector<NodeState>, std::greater<NodeState>> pq;

    dist[source] = 0.0;
    pq.emplace(0.0, source);

    while (!pq.empty())
    {
        const NodeState state = pq.top();
        pq.pop();
        const double currentDist = state.first;
        const std::string node = state.second;

        const auto distIt = dist.find(node);
        if (distIt == dist.end() || currentDist > distIt->second)
        {
            continue;
        }

        if (node == destination)
        {
            break;
        }

        for (const auto &edge : graph.neighbors(node))
        {
            if (edge.weight < 0.0)
            {
                // Dijkstra assumes non-negative weights; skip any unexpected negatives.
                continue;
            }

            const double nextDist = currentDist + edge.weight;
            const auto destIt = dist.find(edge.to);
            if (destIt == dist.end() || nextDist < destIt->second)
            {
                dist[edge.to] = nextDist;
                prev[edge.to] = node;
                pq.emplace(nextDist, edge.to);
            }
        }
    }

    const auto finalIt = dist.find(destination);
    if (finalIt == dist.end())
    {
        return result;
    }

    result.path = reconstructPath(prev, source, destination);
    result.cost = finalIt->second;
    result.reachable = !result.path.empty();
    return result;
}

PathResult bellmanFord(const Graph &graph, const std::string &source, const std::string &destination)
{
    PathResult result;
    const auto nodes = graph.getAllNodes();
    if (nodes.empty() || !graph.contains(source) || !graph.contains(destination))
    {
        return result;
    }

    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, std::string> prev;
    for (const auto &node : nodes)
    {
        dist[node] = kInf;
    }
    dist[source] = 0.0;

    for (std::size_t i = 1; i < nodes.size(); ++i)
    {
        bool updated = false;
        for (const auto &u : nodes)
        {
            const double currentDist = dist[u];
            if (currentDist == kInf)
            {
                continue;
            }
            for (const auto &edge : graph.neighbors(u))
            {
                const double candidate = currentDist + edge.weight;
                if (candidate < dist[edge.to])
                {
                    dist[edge.to] = candidate;
                    prev[edge.to] = u;
                    updated = true;
                }
            }
        }
        if (!updated)
        {
            break;
        }
    }

    for (const auto &u : nodes)
    {
        const double currentDist = dist[u];
        if (currentDist == kInf)
        {
            continue;
        }
        for (const auto &edge : graph.neighbors(u))
        {
            if (currentDist + edge.weight < dist[edge.to])
            {
                result.negativeCycle = true;
                return result;
            }
        }
    }

    const double finalCost = dist[destination];
    if (finalCost == kInf)
    {
        return result;
    }

    result.path = reconstructPath(prev, source, destination);
    result.cost = finalCost;
    result.reachable = !result.path.empty();
    return result;
}

PathResult greedyWalk(const Graph &graph, const std::string &source, const std::string &destination)
{
    PathResult result;
    if (!graph.contains(source) || !graph.contains(destination))
    {
        return result;
    }

    std::unordered_set<std::string> visited;
    std::vector<std::string> path;
    std::string current = source;
    double cost = 0.0;

    path.push_back(current);
    visited.insert(current);

    while (current != destination)
    {
        const auto &edges = graph.neighbors(current);
        Edge bestEdge{};
        bool found = false;
        for (const auto &edge : edges)
        {
            if (visited.count(edge.to) != 0)
            {
                continue;
            }
            if (!found || edge.weight < bestEdge.weight)
            {
                bestEdge = edge;
                found = true;
            }
        }

        if (!found)
        {
            result.path = path;
            result.cost = cost;
            result.reachable = false;
            return result;
        }

        current = bestEdge.to;
        cost += bestEdge.weight;
        path.push_back(current);

        if (visited.count(current) != 0)
        {
            result.path = path;
            result.cost = cost;
            result.reachable = false;
            return result;
        }

        visited.insert(current);
    }

    result.path = path;
    result.cost = cost;
    result.reachable = true;
    return result;
}
