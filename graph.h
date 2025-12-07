#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Edge types supported by the city graph.
enum class EdgeType
{
    Bus,
    Metro,
    Road,
    Walking,
    Unknown
};

struct Edge
{
    std::string to;
    double weight{};
    EdgeType type{EdgeType::Unknown};
};

class Graph
{
public:
    Graph();

    bool loadFromFile(const std::string &filename);
    void addEdge(const std::string &from, const std::string &to, double weight, EdgeType type);

    const std::vector<Edge> &neighbors(const std::string &node) const;
    std::vector<std::string> getAllNodes() const;
    bool contains(const std::string &node) const;
    bool hasNegativeEdge() const;
    std::size_t edgeCount() const;

private:
    std::unordered_map<std::string, std::vector<Edge>> adjacency;
    std::unordered_set<std::string> nodes;
    bool negativeEdgePresent{false};

    EdgeType parseType(const std::string &raw) const;
};

#endif // GRAPH_H
