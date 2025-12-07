#include "graph.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>

namespace
{
    const std::vector<Edge> kEmptyEdges{};

    std::string toLower(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return value;
    }
} // namespace

Graph::Graph() = default;

EdgeType Graph::parseType(const std::string &raw) const
{
    const std::string lowered = toLower(raw);
    if (lowered == "bus")
    {
        return EdgeType::Bus;
    }
    if (lowered == "metro")
    {
        return EdgeType::Metro;
    }
    if (lowered == "road")
    {
        return EdgeType::Road;
    }
    if (lowered == "walking" || lowered == "walk")
    {
        return EdgeType::Walking;
    }
    return EdgeType::Unknown;
}

bool Graph::loadFromFile(const std::string &filename)
{
    std::ifstream input(filename);
    if (!input.is_open())
    {
        std::cerr << "Failed to open graph file: " << filename << "\n";
        return false;
    }

    adjacency.clear();
    nodes.clear();
    negativeEdgePresent = false;

    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line))
    {
        ++lineNumber;
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::istringstream iss(line);
        std::string from, to, typeToken;
        double weight = 0.0;
        if (!(iss >> from >> to >> weight >> typeToken))
        {
            std::cerr << "Skipping malformed line " << lineNumber << ": " << line << "\n";
            continue;
        }

        EdgeType edgeType = parseType(typeToken);
        if (edgeType == EdgeType::Walking)
        {
            // Walking edges are unit weight regardless of input.
            weight = 1.0;
        }
        addEdge(from, to, weight, edgeType);
    }

    return !adjacency.empty();
}

void Graph::addEdge(const std::string &from, const std::string &to, double weight, EdgeType type)
{
    adjacency[from].push_back(Edge{to, weight, type});
    nodes.insert(from);
    nodes.insert(to);
    if (weight < 0.0)
    {
        negativeEdgePresent = true;
    }
}

const std::vector<Edge> &Graph::neighbors(const std::string &node) const
{
    const auto it = adjacency.find(node);
    if (it == adjacency.end())
    {
        return kEmptyEdges;
    }
    return it->second;
}

std::vector<std::string> Graph::getAllNodes() const
{
    return std::vector<std::string>(nodes.begin(), nodes.end());
}

bool Graph::contains(const std::string &node) const
{
    return nodes.find(node) != nodes.end();
}

bool Graph::hasNegativeEdge() const
{
    return negativeEdgePresent;
}

std::size_t Graph::edgeCount() const
{
    std::size_t total = 0;
    for (const auto &entry : adjacency)
    {
        total += entry.second.size();
    }
    return total;
}
