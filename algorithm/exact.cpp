#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <set>
#include <iomanip>
#include <ctime>
#include <thread>
#include <chrono>
#include <limits> // Required for std::numeric_limits

const long long INF_EXACT = std::numeric_limits<long long>::max(); // Use long long for INF

struct EdgeExact
{
    int u, v;
    int weight;
    int id;
    bool is_blocked;

    // Sort primarily by weight, then id for tie-breaking
    bool operator<(const EdgeExact &other) const
    {
        if (weight != other.weight)
        {
            return weight < other.weight;
        }
        return id < other.id;
    }

    bool operator==(const EdgeExact &other) const
    {
        return id == other.id;
    }
};

struct DSUExact
{
    std::vector<int> parent;
    int num_components;

    DSUExact(int n)
    {
        parent.resize(n);
        for (int i = 0; i < n; ++i)
        {
            parent[i] = i;
        }
        num_components = n;
    }

    int find(int i)
    {
        if (parent[i] == i)
            return i;
        return parent[i] = find(parent[i]);
    }

    void unite(int i, int j)
    {
        int root_i = find(i);
        int root_j = find(j);
        if (root_i != root_j)
        {
            parent[root_i] = root_j;
            num_components--;
        }
    }
};

std::pair<long long, std::vector<EdgeExact>> kruskal_exact(
    std::vector<EdgeExact> edges,
    const std::vector<int> &active_vertices,
    const std::set<int>& active_vertices_set) // Pass set for efficient lookup
{
    // If no active vertices, cost is 0, no edges
    if (active_vertices.empty())
    {
        return {0, {}};
    }

    // If only one active vertex, cost is 0, no edges needed
    if (active_vertices.size() == 1) {
         return {0, {}};
    }

    std::sort(edges.begin(), edges.end());

    std::map<int, int> vertex_to_dsu_idx;
    int dsu_idx_counter = 0;
    for (int v_orig : active_vertices)
    {
        vertex_to_dsu_idx[v_orig] = dsu_idx_counter++;
    }

    // DSU size should be based on the number of active vertices
    DSUExact dsu(dsu_idx_counter);
    long long mst_cost = 0; // Use long long for cost
    std::vector<EdgeExact> mst_edges;

    for (const auto &edge : edges)
    {
        // Only consider edges between active vertices
        if (active_vertices_set.find(edge.u) == active_vertices_set.end() ||
            active_vertices_set.find(edge.v) == active_vertices_set.end())
        {
            continue;
        }

        int u_dsu = vertex_to_dsu_idx[edge.u];
        int v_dsu = vertex_to_dsu_idx[edge.v];

        if (dsu.find(u_dsu) != dsu.find(v_dsu))
        {
            dsu.unite(u_dsu, v_dsu);
            mst_cost += edge.weight;
            mst_edges.push_back(edge);
            // Optimization: if we have connected all active vertices (num_components == 1)
            if (dsu.num_components == 1) break;
        }
    }

    // Check if all active vertices are connected
    // The MST must connect all vertices in active_vertices for a valid Steiner tree candidate
    bool all_active_connected = (dsu.num_components <= 1); // Should be exactly 1 if connected

    if (all_active_connected) {
         return {mst_cost, mst_edges};
    } else {
        // If not all active vertices are connected, this subset is not valid for Steiner tree
        return {INF_EXACT, {}};
    }
}


int main()
{
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    auto t1 = std::chrono::high_resolution_clock::now();

    int num_vertices;
    int num_edges_input;
    std::cin >> num_vertices >> num_edges_input;

    if (num_vertices <= 0 || num_edges_input < 0)
    {
        std::cerr << "Invalid number of vertices or edges." << std::endl;
        return 1;
    }

    std::vector<EdgeExact> all_edges_original;
    all_edges_original.reserve(num_edges_input);
    for (int i = 0; i < num_edges_input; ++i)
    {
        int u, v, w, blocked_flag;
        std::cin >> u >> v >> w >> blocked_flag;

        if (u < 0 || u >= num_vertices || v < 0 || v >= num_vertices)
        {
             continue;
        }

        if (u == v)
        {
            continue;
        }

        all_edges_original.push_back({u, v, w, i, (blocked_flag == 1)});
    }

    int num_terminals_input;
    std::cin >> num_terminals_input;
    std::vector<int> terminals_input(num_terminals_input);
    std::set<int> terminal_set;
    for (int i = 0; i < num_terminals_input; ++i)
    {
        std::cin >> terminals_input[i];
        if (terminals_input[i] < 0 || terminals_input[i] >= num_vertices) {
             continue;
        }
        terminal_set.insert(terminals_input[i]);
    }

    std::vector<int> terminals(terminal_set.begin(), terminal_set.end());
    int num_terminals = terminals.size();


    // Base case: 0 or 1 terminal
    if (num_terminals <= 1)
    {
        std::cout << "--- Exact Algorithm Result (Brute-Force Subset Enumeration) ---" << std::endl;
        std::cout << "Total Cost: 0" << std::endl;
        std::cout << "Edges in Steiner Tree (0 edges):" << std::endl;

        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Exact took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                  << " milliseconds\n";
        return 0;
    }

    std::vector<int> non_terminals;
    non_terminals.reserve(num_vertices - num_terminals);
    for (int i = 0; i < num_vertices; ++i)
    {
        if (terminal_set.find(i) == terminal_set.end())
        {
            non_terminals.push_back(i);
        }
    }

    long long min_steiner_tree_cost = INF_EXACT;
    std::vector<EdgeExact> best_steiner_tree_edges;

    int num_non_terminals = non_terminals.size();

    // Brute force over all subsets of non-terminals
    for (int i = 0; i < (1 << num_non_terminals); ++i)
    {
        std::vector<int> current_active_nodes = terminals;
        std::set<int> active_nodes_set(terminals.begin(), terminals.end());

        // Add selected non-terminals to active nodes
        for (int j = 0; j < num_non_terminals; ++j)
        {
            if ((i >> j) & 1)
            {
                current_active_nodes.push_back(non_terminals[j]);
                active_nodes_set.insert(non_terminals[j]);
            }
        }

        std::vector<EdgeExact> current_graph_edges;
        current_graph_edges.reserve(all_edges_original.size());

        // Build the subgraph for the current set of active nodes, INCLUDE ONLY NON-BLOCKED EDGES
        for (const auto &original_edge : all_edges_original)
        {
            if (!original_edge.is_blocked && active_nodes_set.count(original_edge.u) && active_nodes_set.count(original_edge.v))
            {
                current_graph_edges.push_back(original_edge);
            }
        }

        // If there are active nodes, find MST
        if (!current_active_nodes.empty())
        {
            std::pair<long long, std::vector<EdgeExact>> current_mst_result =
                kruskal_exact(current_graph_edges, current_active_nodes, active_nodes_set);

            // kruskal_exact returns INF_EXACT cost if active vertices are not connected
            if (current_mst_result.first != INF_EXACT)
            {
                // This MST connects all nodes in current_active_nodes.
                // We need to ensure all ORIGINAL terminals are in current_active_nodes.
                // This is guaranteed by the subset enumeration logic (terminals are always included).
                // The kruskal_exact function now verifies if all nodes in active_vertices are connected.
                // So, if current_mst_result.first != INF_EXACT, all terminals are connected in this subgraph.

                if (current_mst_result.first < min_steiner_tree_cost)
                {
                    min_steiner_tree_cost = current_mst_result.first;
                    best_steiner_tree_edges = current_mst_result.second;
                }
            }
        }
    }

    std::cout << "--- Exact Algorithm Result (Brute-Force Subset Enumeration) ---" << std::endl;
    if (min_steiner_tree_cost >= INF_EXACT)
    {
        std::cout << "Could not connect all terminals using unblocked paths." << std::endl;
        std::cout << "Total Cost: INF" << std::endl;
        std::cout << "Edges in Steiner Tree (0 edges):" << std::endl;
    }
    else
    {
        std::cout << "Total Cost: " << min_steiner_tree_cost << std::endl;
        std::cout << "Edges in Steiner Tree (" << best_steiner_tree_edges.size() << " edges):" << std::endl;
        for (const auto &edge : best_steiner_tree_edges)
        {
            // Output format consistent with input (0 for not blocked, 1 for blocked)
            std::cout << edge.u << " " << edge.v << " " << edge.weight << " " << (edge.is_blocked ? 1 : 0) << std::endl;
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Exact took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds\n";

    return 0;
}