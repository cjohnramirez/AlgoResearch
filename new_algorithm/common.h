#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <limits>
#include <chrono>
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

// Structure to represent an edge
struct Edge
{
    int u, v;        // vertices (endpoints)
    int weight;      // weight of the edge
    bool is_blocked; // whether the edge is blocked

    Edge(int u, int v, int weight, bool is_blocked)
        : u(u), v(v), weight(weight), is_blocked(is_blocked) {}

    // For sorting and comparison
    bool operator<(const Edge &other) const
    {
        if (is_blocked != other.is_blocked)
        {
            return !is_blocked; // Non-blocked edges come first
        }
        return weight < other.weight; // Then by weight
    }

    bool operator==(const Edge &other) const
    {
        // Check if this edge connects the same vertices as the other edge
        return ((u == other.u && v == other.v) || (u == other.v && v == other.u));
    }
};

// Structure to represent the Steiner tree solution
struct SteinerTree
{
    std::vector<Edge> edges;         // Edges in the solution
    int total_weight;                // Sum of weights of unblocked edges
    std::vector<Edge> blocked_edges; // Blocked edges used in the solution
    double runtime;                  // Runtime in milliseconds

    SteinerTree() : total_weight(0), runtime(0) {}

    void addEdge(const Edge &edge)
    {
        edges.push_back(edge);
        if (!edge.is_blocked)
        {
            total_weight += edge.weight;
        }
        else
        {
            blocked_edges.push_back(edge);
        }
    }

    void calculate_metrics()
    {
        // Clear existing metrics
        total_weight = 0;
        blocked_edges.clear();

        // Recalculate
        for (const Edge &edge : edges)
        {
            if (edge.is_blocked)
            {
                blocked_edges.push_back(edge);
            }
            else
            {
                total_weight += edge.weight;
            }
        }
    }

    int blockedWeight() const
    {
        int weight = 0;
        for (const Edge &edge : blocked_edges)
        {
            weight += edge.weight;
        }
        return weight;
    }

    void print_results() const
    {
        std::cout << "Steiner Tree Results:" << std::endl;
        std::cout << "Edges in Steiner tree:" << std::endl;
        
        for (const Edge &edge : edges)
        {
            std::cout << edge.u << " " << edge.v << " " << edge.weight
                      << (edge.is_blocked ? " 1" : " 0") << std::endl;
        }
        std::cout << "Number of unblocked edges: " << edges.size() << std::endl;
        std::cout << "Total weight of unblocked edges: " << total_weight << std::endl;
        std::cout << "Number of blocked edges: " << blocked_edges.size() << std::endl;
        std::cout << "Total weight of blocked edges: " << blockedWeight() << std::endl;

        std::cout << "Runtime: " << runtime << " ms" << std::endl;
        std::cout << std::endl;
    }
};

// Graph representation
class Graph
{
public:
    int V;                              // Number of vertices
    std::vector<std::vector<Edge>> adj; // Adjacency list
    std::vector<Edge> edge_list;        // List of all edges
    std::set<int> terminals;            // Set of terminal vertices

    Graph(int V) : V(V), adj(V) {}

    void addEdge(int u, int v, int weight, bool is_blocked)
    {
        Edge edge(u, v, weight, is_blocked);
        adj[u].push_back(edge);
        adj[v].push_back(Edge(v, u, weight, is_blocked)); // Add the reverse edge as well

        // Add to edge list, avoid duplicates
        bool exists = false;
        for (const Edge &e : edge_list)
        {
            if ((e.u == u && e.v == v) || (e.u == v && e.v == u))
            {
                exists = true;
                break;
            }
        }
        if (!exists)
        {
            edge_list.push_back(edge);
        }
    }

    void addTerminal(int t)
    {
        terminals.insert(t);
    }

    int totalBlockedWeight(const std::vector<Edge> &edges) const
    {
        int weight = 0;
        for (const Edge &edge : edges)
        {
            if (edge.is_blocked)
            {
                weight += edge.weight;
            }
        }
        return weight;
    }

    // Compute shortest paths between all pairs of vertices
    std::vector<std::vector<std::pair<int, std::vector<Edge>>>> shortestPaths() const
    {
        std::vector<std::vector<std::pair<int, std::vector<Edge>>>> paths(V, std::vector<std::pair<int, std::vector<Edge>>>(V, {std::numeric_limits<int>::max(), {}}));

        // Initialize with direct edges
        for (int u = 0; u < V; ++u)
        {
            paths[u][u] = {0, {}}; // Distance to self is 0
            for (const Edge &edge : adj[u])
            {
                int v = edge.v;
                // We now use the actual edge weight in the calculation rather than a heavy penalty
                paths[u][v] = {edge.weight, {edge}};
            }
        }

        // Modified Floyd-Warshall algorithm that prioritizes minimizing blocked edge weight
        for (int k = 0; k < V; ++k)
        {
            for (int i = 0; i < V; ++i)
            {
                for (int j = 0; j < V; ++j)
                {
                    if (paths[i][k].first != std::numeric_limits<int>::max() &&
                        paths[k][j].first != std::numeric_limits<int>::max())
                    {

                        // Calculate blocked weight for current best path i->j
                        int current_blocked_weight = 0;
                        for (const Edge &e : paths[i][j].second)
                        {
                            if (e.is_blocked)
                                current_blocked_weight += e.weight;
                        }

                        // Calculate blocked weight for potential new path i->k->j
                        int new_blocked_weight = 0;
                        for (const Edge &e : paths[i][k].second)
                        {
                            if (e.is_blocked)
                                new_blocked_weight += e.weight;
                        }
                        for (const Edge &e : paths[k][j].second)
                        {
                            if (e.is_blocked)
                                new_blocked_weight += e.weight;
                        }

                        // Replace if new path has lower blocked weight, or equal blocked weight but lower total weight
                        bool replace = false;
                        if (paths[i][j].second.empty() ||
                            new_blocked_weight < current_blocked_weight ||
                            (new_blocked_weight == current_blocked_weight &&
                             paths[i][k].first + paths[k][j].first < paths[i][j].first))
                        {
                            replace = true;
                        }

                        if (replace)
                        {
                            // Combine the paths
                            std::vector<Edge> new_path = paths[i][k].second;
                            new_path.insert(new_path.end(), paths[k][j].second.begin(), paths[k][j].second.end());

                            paths[i][j] = {paths[i][k].first + paths[k][j].first, new_path};
                        }
                    }
                }
            }
        }

        return paths;
    }

    // Dijkstra's algorithm for shortest path with preference for non-blocked edges
    std::vector<Edge> shortestPath(int start, int end) const
    {
        std::vector<int> dist(V, std::numeric_limits<int>::max());
        std::vector<int> blocked_count(V, std::numeric_limits<int>::max());
        std::vector<int> parent(V, -1);
        std::vector<Edge> parent_edge(V, Edge(-1, -1, -1, false));
        std::priority_queue<std::tuple<int, int, int>, std::vector<std::tuple<int, int, int>>, std::greater<>> pq;

        dist[start] = 0;
        blocked_count[start] = 0;
        pq.push({0, 0, start}); // {blocked_count, distance, vertex}

        while (!pq.empty())
        {
            auto [curr_blocked, curr_dist, u] = pq.top();
            pq.pop();

            if (u == end)
                break;

            if (curr_blocked > blocked_count[u] || (curr_blocked == blocked_count[u] && curr_dist > dist[u]))
            {
                continue;
            }

            for (const Edge &edge : adj[u])
            {
                int v = edge.v;
                int new_blocked = curr_blocked + (edge.is_blocked ? 1 : 0);
                int new_dist = curr_dist + edge.weight;

                if (new_blocked < blocked_count[v] ||
                    (new_blocked == blocked_count[v] && new_dist < dist[v]))
                {
                    blocked_count[v] = new_blocked;
                    dist[v] = new_dist;
                    parent[v] = u;
                    parent_edge[v] = edge;
                    pq.push({new_blocked, new_dist, v});
                }
            }
        }

        if (parent[end] == -1)
        {
            return {}; // No path found
        }

        // Reconstruct the path
        std::vector<Edge> path;
        for (int v = end; v != start; v = parent[v])
        {
            path.push_back(parent_edge[v]);
        }
        std::reverse(path.begin(), path.end());

        return path;
    }

    // Check if a set of edges forms a valid Steiner tree
    bool isValidSteinerTree(const std::vector<Edge> &tree_edges) const
    {
        if (tree_edges.empty() && terminals.size() > 1)
        {
            return false;
        }

        // Create a graph from the tree edges
        std::vector<std::vector<int>> tree(V);
        for (const Edge &edge : tree_edges)
        {
            tree[edge.u].push_back(edge.v);
            tree[edge.v].push_back(edge.u);
        }

        // Check connectivity using BFS
        std::vector<bool> visited(V, false);
        std::queue<int> q;

        // Start from the first terminal
        int start = *terminals.begin();
        q.push(start);
        visited[start] = true;

        while (!q.empty())
        {
            int u = q.front();
            q.pop();

            for (int v : tree[u])
            {
                if (!visited[v])
                {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }

        // Check if all terminals are connected
        for (int t : terminals)
        {
            if (!visited[t])
            {
                return false;
            }
        }

        return true;
    }

    // Count blocked edges in a set of edges
    int countBlockedEdges(const std::vector<Edge> &edges) const
    {
        int count = 0;
        for (const Edge &edge : edges)
        {
            if (edge.is_blocked)
            {
                count++;
            }
        }
        return count;
    }

    // Compute the total weight of unblocked edges
    int totalUnblockedWeight(const std::vector<Edge> &edges) const
    {
        int weight = 0;
        for (const Edge &edge : edges)
        {
            if (!edge.is_blocked)
            {
                weight += edge.weight;
            }
        }
        return weight;
    }

    // Parse input from a file
    static Graph parseInput(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            exit(1);
        }

        int num_vertices, num_edges, num_terminals;
        file >> num_vertices >> num_edges >> num_terminals;

        Graph graph(num_vertices);

        // Read terminals
        for (int i = 0; i < num_terminals; ++i)
        {
            int terminal;
            file >> terminal;
            graph.addTerminal(terminal);
        }

        // Read edges
        for (int i = 0; i < num_edges; ++i)
        {
            int u, v, weight, is_blocked;
            file >> u >> v >> weight >> is_blocked;
            graph.addEdge(u, v, weight, is_blocked == 1);
        }

        return graph;
    }
};

// Function declarations for KMB approximation algorithm
SteinerTree kmb_algorithm(const Graph &graph);

// Function declarations for exact algorithm
SteinerTree exact_algorithm(const Graph &graph);

#endif // COMMON_H