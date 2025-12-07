#include "algorithms.h"
#include "graph.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>

namespace
{
    void printResult(const std::string &label, const PathResult &result)
    {
        std::cout << label << " result:\n";
        if (result.negativeCycle)
        {
            std::cout << "  Negative cycle detected. No reliable shortest path.\n\n";
            return;
        }
        if (!result.reachable)
        {
            std::cout << "  Destination unreachable with this method.\n\n";
            return;
        }

        std::cout << "  Path: ";
        for (std::size_t i = 0; i < result.path.size(); ++i)
        {
            std::cout << result.path[i];
            if (i + 1 < result.path.size())
            {
                std::cout << " -> ";
            }
        }
        std::cout << "\n";
        std::cout << "  Total cost: " << result.cost << "\n\n";
    }

    void printTiming(const std::string &name, bool ran, double milliseconds, const std::string &note = "")
    {
        std::cout << std::left << std::setw(16) << name << ": ";
        if (!ran)
        {
            std::cout << "N/A (skipped)";
        }
        else
        {
            std::cout << milliseconds << " ms";
        }
        if (!note.empty())
        {
            std::cout << " - " << note;
        }
        std::cout << "\n";
    }
} // namespace

int main()
{
    std::cout << "Enter city map file path (default citymap.txt): ";
    std::string filePath;
    std::getline(std::cin, filePath);
    if (filePath.empty())
    {
        filePath = "citymap.txt";
    }

    Graph graph;
    if (!graph.loadFromFile(filePath))
    {
        std::cerr << "Failed to load graph. Check the file path and format.\n";
        return 1;
    }

    std::cout << "Loaded graph with " << graph.getAllNodes().size() << " nodes and " << graph.edgeCount() << " edges.\n";
    std::cout << "Contains negative edges: " << (graph.hasNegativeEdge() ? "Yes" : "No") << "\n\n";

    std::string source;
    std::string destination;
    std::cout << "Enter source node: ";
    std::getline(std::cin, source);
    std::cout << "Enter destination node: ";
    std::getline(std::cin, destination);

    if (!graph.contains(source) || !graph.contains(destination))
    {
        std::cerr << "Both source and destination must exist in the graph.\n";
        return 1;
    }

    const bool needBellmanFord = graph.hasNegativeEdge();
    const bool canUseDijkstra = !needBellmanFord;

    PathResult dijkstraResult;
    PathResult bellmanResult;
    PathResult greedyResult;
    bool ranDijkstra = false;
    bool ranBellman = false;
    double dijkstraMs = 0.0;
    double bellmanMs = 0.0;
    double greedyMs = 0.0;

    auto greedyStart = std::chrono::steady_clock::now();
    greedyResult = greedyWalk(graph, source, destination);
    auto greedyEnd = std::chrono::steady_clock::now();
    greedyMs = std::chrono::duration<double, std::milli>(greedyEnd - greedyStart).count();

    if (canUseDijkstra)
    {
        auto start = std::chrono::steady_clock::now();
        dijkstraResult = dijkstra(graph, source, destination);
        auto end = std::chrono::steady_clock::now();
        dijkstraMs = std::chrono::duration<double, std::milli>(end - start).count();
        ranDijkstra = true;
    }

    if (needBellmanFord)
    {
        auto start = std::chrono::steady_clock::now();
        bellmanResult = bellmanFord(graph, source, destination);
        auto end = std::chrono::steady_clock::now();
        bellmanMs = std::chrono::duration<double, std::milli>(end - start).count();
        ranBellman = true;
    }

    if (ranDijkstra)
    {
        printResult("Dijkstra", dijkstraResult);
    }
    else
    {
        std::cout << "Dijkstra result:\n  Skipped because graph contains negative edges.\n\n";
    }

    if (ranBellman)
    {
        printResult("Bellman-Ford", bellmanResult);
    }
    else
    {
        std::cout << "Bellman-Ford result:\n  Skipped because graph has no negative edges.\n\n";
    }

    printResult("Greedy walk", greedyResult);

    std::cout << "Timing comparison (milliseconds):\n";
    printTiming("Dijkstra", ranDijkstra, dijkstraMs, canUseDijkstra ? "" : "not run");
    printTiming("Bellman-Ford", ranBellman, bellmanMs, needBellmanFord ? "" : "not needed");
    printTiming("Greedy", true, greedyMs);

    if (bellmanResult.negativeCycle)
    {
        std::cout << "\nNote: Negative cycle detected. Shortest path is undefined.\n";
    }

    return 0;
}
