#include "common.h"
#include <functional>

// Implementation of KMB 2-approximation algorithm
// Step 1: Construct a complete graph over terminals
// Step 2: Find a minimum spanning tree of this complete graph
// Step 3: Replace edges of the MST with shortest paths in the original graph

SteinerTree kmb_algorithm(const Graph &graph)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    SteinerTree result;

    // If there's only one terminal, return empty tree
    if (graph.terminals.size() <= 1)
    {
        auto end_time = std::chrono::high_resolution_clock::now();
        result.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        return result;
    }

    // Step 1: Compute shortest paths between all terminal vertices
    // We use Floyd-Warshall algorithm to find shortest paths
    auto paths = graph.shortestPaths();

    // Create a complete graph over terminal vertices
    std::vector<std::tuple<int, int, int, int, std::vector<Edge>>> terminal_edges;

    for (auto it1 = graph.terminals.begin(); it1 != graph.terminals.end(); ++it1)
    {
        for (auto it2 = std::next(it1); it2 != graph.terminals.end(); ++it2)
        {
            int u = *it1;
            int v = *it2;

            // Calculate the weight of blocked edges in the path
            int blocked_weight = 0;
            for (const Edge &edge : paths[u][v].second)
            {
                if (edge.is_blocked)
                {
                    blocked_weight += edge.weight;
                }
            }

            // Calculate the weight of the path (only unblocked edges)
            int path_weight = 0;
            for (const Edge &edge : paths[u][v].second)
            {
                if (!edge.is_blocked)
                {
                    path_weight += edge.weight;
                }
            }

            terminal_edges.push_back({blocked_weight, path_weight, u, v, paths[u][v].second});
        }
    }

    // Sort edges first by weight of blocked edges, then by unblocked weight
    std::sort(terminal_edges.begin(), terminal_edges.end());

    // Step 2: Find MST of the complete graph using Kruskal's algorithm
    std::vector<std::vector<Edge>> mst_paths;
    std::vector<int> parent(graph.V);
    for (int i = 0; i < graph.V; i++)
    {
        parent[i] = i;
    }

    // Find function for Union-Find
    std::function<int(int)> find = [&](int x)
    {
        if (parent[x] != x)
        {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    };

    // Union function for Union-Find
    auto unite = [&](int x, int y)
    {
        parent[find(x)] = find(y);
    };

    // Apply Kruskal's algorithm
    for (const auto &[blocked_count, path_weight, u, v, path] : terminal_edges)
    {
        if (find(u) != find(v))
        {
            unite(u, v);
            mst_paths.push_back(path);
        }
    }

    // Step 3: Replace edges with shortest paths and add to result
    std::set<std::pair<int, int>> included_edges;

    for (const auto &path : mst_paths)
    {
        for (const Edge &edge : path)
        {
            // Create an undirected edge representation
            std::pair<int, int> undirected_edge = std::minmax(edge.u, edge.v);

            // Add the edge if not already included
            if (included_edges.find(undirected_edge) == included_edges.end())
            {
                included_edges.insert(undirected_edge);
                result.addEdge(edge);
            }
        }
    }

    // Calculate metrics
    result.calculate_metrics();

    auto end_time = std::chrono::high_resolution_clock::now();
    result.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    return result;
}