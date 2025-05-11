#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <set>
#include <iomanip>
#include <queue>
#include <limits>
#include <thread>
#include <chrono>
#include <cmath>

const long long INF = std::numeric_limits<long long>::max();

struct Edge
{
    int u, v;
    long long weight;
    int id;
    bool is_blocked; // Original blocked status

    // Sort primarily by weight, then id for tie-breaking
    bool operator<(const Edge &other) const
    {
        if (weight != other.weight)
        {
            return weight < other.weight;
        }
        return id < other.id;
    }

    bool operator==(const Edge &other) const
    {
        return id == other.id;
    }
};

struct SuperEdge
{
    int u_idx, v_idx; // Indices in the terminals vector
    long long weight;

    bool operator<(const SuperEdge &other) const
    {
        if (weight != other.weight)
        {
            return weight < other.weight;
        }
        if (u_idx != other.u_idx)
        {
            return u_idx < other.u_idx;
        }
        return v_idx < other.v_idx;
    }
};

struct DSU
{
    std::vector<int> parent;
    int num_components; // Added num_components to DSU

    DSU(int n)
    {
        parent.resize(n);
        for (int i = 0; i < n; ++i)
        {
            parent[i] = i;
        }
        num_components = n; // Initialize component count
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
            num_components--; // Decrement component count on union
        }
    }
};


// Dijkstra using edge index and accessing weight from original edges
std::pair<std::vector<long long>, std::vector<int>> dijkstra_with_edges(
    int start_node,
    int num_vertices,
    const std::vector<Edge> &all_edges_original,
    const std::vector<std::vector<std::pair<int, int>>> &adj_edge_indices) // adj stores pairs of {neighbor, original_edge_index}
{
    std::vector<long long> dist(num_vertices, INF);
    std::vector<int> pred_edge_idx(num_vertices, -1);

    dist[start_node] = 0;
    std::priority_queue<std::pair<long long, int>, std::vector<std::pair<long long, int>>, std::greater<std::pair<long long, int>>> pq;
    pq.push({0, start_node});

    while (!pq.empty())
    {
        long long d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u])
        {
            continue;
        }

        for (const auto &edge_pair : adj_edge_indices[u])
        {
            int v = edge_pair.first;
            int edge_idx = edge_pair.second; // Original edge index

            if (edge_idx < 0 || static_cast<size_t>(edge_idx) >= all_edges_original.size()) {
                continue;
            }

            long long weight = all_edges_original[edge_idx].weight;

            if (dist[u] != INF && weight != INF)
            {
                if (dist[u] > INF - weight) // Check for overflow
                {
                    continue;
                }
                if (dist[u] + weight < dist[v])
                {
                    dist[v] = dist[u] + weight;
                    pred_edge_idx[v] = edge_idx;
                    pq.push({dist[v], v});
                }
            }
        }
    }
    return {dist, pred_edge_idx};
}

std::vector<int> reconstruct_path(
    int start_node,
    int end_node,
    const std::vector<int> &pred_edge_idx,
    const std::vector<Edge> &all_edges_original,
    int num_vertices)
{
    std::vector<int> path_indices;
    int curr = end_node;

    // Check for trivial case or no path found by Dijkstra
    if (start_node == end_node) return {};
    if (curr < 0 || curr >= num_vertices || pred_edge_idx.empty() || pred_edge_idx.size() != static_cast<size_t>(num_vertices) || pred_edge_idx[curr] == -1) {
         return {}; // Invalid end_node, pred_edge_idx size, or no path exists
    }


    size_t safeguard_counter = 0;
    const size_t max_path_len = num_vertices;

    while (curr != start_node && pred_edge_idx[curr] != -1 && safeguard_counter <= max_path_len)
    {
        int edge_on_path_idx = pred_edge_idx[curr];

        if (edge_on_path_idx < 0 || static_cast<size_t>(edge_on_path_idx) >= all_edges_original.size())
        {
            // This indicates an issue with Dijkstra's pred_edge_idx population
            std::cerr << "Error in reconstruct_path: Invalid edge index " << edge_on_path_idx << " in pred_edge_idx[" << curr << "]." << std::endl;
            return {};
        }

        path_indices.push_back(edge_on_path_idx);
        const Edge &edge_on_path = all_edges_original[edge_on_path_idx];

        // Move to the other endpoint of the edge
        if (edge_on_path.u == curr)
        {
            curr = edge_on_path.v;
        }
        else if (edge_on_path.v == curr)
        {
            curr = edge_on_path.u;
        }
        else
        {
            // This indicates a mismatch between pred_edge_idx and edge data
            std::cerr << "Error in reconstruct_path: Edge " << edge_on_path_idx << " does not connect to current vertex " << curr << "." << std::endl;
            return {};
        }
        safeguard_counter++;
    }

    // If loop terminated without reaching start_node (and start_node != end_node) and pred_edge_idx[curr] != -1,
    // it means safeguard_counter reached max_path_len, suggesting a potential cycle in pred_edge_idx or very long path
     if (curr != start_node && start_node != end_node && safeguard_counter > max_path_len)
     {
         std::cerr << "Warning: Path reconstruction exceeded max length from " << start_node << " to " << end_node << ". Potential cycle or path issue." << std::endl;
         return {}; // Return empty path on safeguard hit
     }
     // If loop terminated because pred_edge_idx[curr] == -1 and curr != start_node, no path exists
     if (curr != start_node && start_node != end_node && pred_edge_idx[curr] == -1)
     {
          return {};
     }


    std::reverse(path_indices.begin(), path_indices.end());
    return path_indices;
}


std::pair<long long, std::vector<Edge>> kruskal_edges(
    std::vector<Edge> edges,
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

    DSU dsu(dsu_idx_counter);
    long long mst_cost = 0;
    std::vector<Edge> mst_edges;

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

    // Check if all active vertices are connected in the MST
    bool all_active_connected = (dsu.num_components <= 1); // Should be exactly 1 if connected


    if (all_active_connected) {
         return {mst_cost, mst_edges};
    } else {
         return {INF, {}}; // Return INF if not all active vertices are disconnected
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

    std::vector<Edge> all_edges_original;
    all_edges_original.reserve(num_edges_input);

    // Adjacency list storing {neighbor, original_edge_index} for UNBLOCKED edges
    std::vector<std::vector<std::pair<int, int>>> adj_unblocked;
    if (num_vertices > 0)
    {
       adj_unblocked.resize(num_vertices);
    }


    for (int i = 0; i < num_edges_input; ++i)
    {
        int u, v;
        long long w;
        int blocked_flag;
        std::cin >> u >> v >> w >> blocked_flag;

        if (u < 0 || u >= num_vertices || v < 0 || v >= num_vertices)
        {
            continue;
        }

        if (u == v)
        {
            continue;
        }

        bool is_blocked = (blocked_flag == 1);

        // Store all original edges with their blocked status
        all_edges_original.push_back({u, v, w, (int)all_edges_original.size(), is_blocked});

        // Only add unblocked edges to the adjacency list for Dijkstra
        if (!is_blocked)
        {
             adj_unblocked[u].push_back({v, (int)all_edges_original.size() - 1});
             adj_unblocked[v].push_back({u, (int)all_edges_original.size() - 1});
        }
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
        std::cout << "--- KMB 2-Approximation Algorithm Result ---" << std::endl;
        std::cout << "Total Cost: 0" << std::endl;
        std::cout << "Edges in Steiner Tree (0 edges):" << std::endl;
         auto t2 = std::chrono::high_resolution_clock::now();
         std::cout << "Approx took "
                   << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                   << " milliseconds\n";
        return 0;
    }

    std::map<int, int> terminal_vertex_to_idx;
    for (int i = 0; i < num_terminals; ++i)
    {
        terminal_vertex_to_idx[terminals[i]] = i;
    }


    std::vector<std::vector<long long>> term_dist;
    std::vector<std::vector<int>> term_pred_edge_idx;

    if (num_terminals > 0 && num_vertices > 0)
    {
       term_dist.resize(num_terminals, std::vector<long long>(num_vertices, INF));
       term_pred_edge_idx.resize(num_terminals, std::vector<int>(num_vertices, -1));
    }

    // Run Dijkstra from each terminal on the unblocked graph
    for (int i = 0; i < num_terminals; ++i)
    {
        int start_term = terminals[i];
        std::tie(term_dist[i], term_pred_edge_idx[i]) = dijkstra_with_edges(start_term, num_vertices, all_edges_original, adj_unblocked);
    }

    std::vector<SuperEdge> terminal_graph_super_edges;
    terminal_graph_super_edges.reserve(num_terminals * (num_terminals - 1) / 2);


    // Build the complete graph on terminals using shortest paths
    bool possible_to_connect_all_terminals = true;
    if (num_terminals > 1) {
        for (int i = 0; i < num_terminals; ++i) {
            for (int j = i + 1; j < num_terminals; ++j) {
                 long long path_cost = term_dist[i][terminals[j]];
                 if (path_cost != INF) {
                      SuperEdge new_super_edge = {i, j, path_cost}; // Create the object first
                      terminal_graph_super_edges.push_back(new_super_edge); // Push back the created object
                 }
            }
        }

        // Check if all terminals are in the same connected component in the original unblocked graph
        // by checking reachability from the first terminal to all others.
        for (int i = 1; i < num_terminals; ++i) {
            if (term_dist[0][terminals[i]] == INF) {
                possible_to_connect_all_terminals = false;
                break;
            }
        }
    }


    if (!possible_to_connect_all_terminals || terminal_graph_super_edges.empty())
    {
        std::cout << "--- KMB 2-Approximation Algorithm Result ---" << std::endl;
        std::cout << "Could not connect all terminals using unblocked paths." << std::endl;
        std::cout << "Total Cost: INF" << std::endl;
        std::cout << "Edges in Steiner Tree (0 edges):" << std::endl;
         auto t2 = std::chrono::high_resolution_clock::now();
         std::cout << "Approx took "
                   << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                   << " milliseconds\n";
        return 0;
    }


    DSU terminal_dsu(num_terminals);
    std::vector<SuperEdge> mst_on_terminals_super_edges;
    mst_on_terminals_super_edges.reserve(num_terminals - 1);

    std::sort(terminal_graph_super_edges.begin(), terminal_graph_super_edges.end());

    // Find MST on the terminal graph (complete graph on terminals with shortest path weights)
    for (const auto &super_edge : terminal_graph_super_edges)
    {
        int u_dsu = super_edge.u_idx;
        int v_dsu = super_edge.v_idx;

        if (terminal_dsu.find(u_dsu) != terminal_dsu.find(v_dsu))
        {
            terminal_dsu.unite(u_dsu, v_dsu);
            mst_on_terminals_super_edges.push_back(super_edge);

            if (mst_on_terminals_super_edges.size() == static_cast<size_t>(num_terminals - 1))
            {
                break; // MST on n vertices has n-1 edges
            }
        }
    }


    // Verify if the MST on terminals connected all terminals (should be true if possible_to_connect_all_terminals was true)
    // This check is implicitly done by the size of mst_on_terminals_super_edges and the earlier reachability check.
    // If mst_on_terminals_super_edges.size() != num_terminals - 1 and num_terminals > 1, it means the terminal graph was disconnected.

    std::set<int> Gs_edge_indices;
    std::set<int> Gs_vertices_set;

    // Add all terminals to Gs_vertices_set initially
    for (int t : terminals)
    {
        Gs_vertices_set.insert(t);
    }


    // Reconstruct the original edge paths for each super edge in the terminal MST and build G_S
    for (const auto &super_edge_in_Mt : mst_on_terminals_super_edges)
    {
        int term1_vertex = terminals[super_edge_in_Mt.u_idx];
        int term2_vertex = terminals[super_edge_in_Mt.v_idx];


        // Reconstruct path from term1 to term2 using Dijkstra results from term1
        const std::vector<int> &pred_edge_idx_from_term1 = term_pred_edge_idx[super_edge_in_Mt.u_idx];

        std::vector<int> original_edge_indices_for_path =
            reconstruct_path(term1_vertex, term2_vertex, pred_edge_idx_from_term1, all_edges_original, num_vertices);


        // Add all edges and vertices from the reconstructed path to G_S
        for (int original_edge_idx : original_edge_indices_for_path)
        {
            if (original_edge_idx >= 0 && static_cast<size_t>(original_edge_idx) < all_edges_original.size())
            {
                Gs_edge_indices.insert(original_edge_idx);
                const Edge &original_edge = all_edges_original[original_edge_idx];
                Gs_vertices_set.insert(original_edge.u);
                Gs_vertices_set.insert(original_edge.v);
            } else {
                 std::cerr << "Error: Reconstructed path returned invalid edge index: " << original_edge_idx << std::endl;
                 // This indicates a serious issue in path reconstruction or Dijkstra
            }
        }
    }


    std::vector<Edge> Gs_edges;
    Gs_edges.reserve(Gs_edge_indices.size());
    for (int edge_idx : Gs_edge_indices)
    {
        Gs_edges.push_back(all_edges_original[edge_idx]);
    }
    std::vector<int> Gs_active_vertices(Gs_vertices_set.begin(), Gs_vertices_set.end());
    std::set<int> Gs_active_vertices_set(Gs_active_vertices.begin(), Gs_active_vertices.end());


    // Find MST on the subgraph G_S
    std::pair<long long, std::vector<Edge>> final_steiner_tree_result =
        kruskal_edges(Gs_edges, Gs_active_vertices, Gs_active_vertices_set);

    // Final check: Ensure all original terminals are present in the vertex set of the final MST edges.
    // This is the most robust check for terminal connectivity in the output tree.
    bool all_terminals_in_final_tree = true;
    if (final_steiner_tree_result.first != INF) { // Only check if a tree was found
        std::set<int> final_tree_vertices;
        for(const auto& edge : final_steiner_tree_result.second) {
            final_tree_vertices.insert(edge.u);
            final_tree_vertices.insert(edge.v);
        }
        for(int terminal : terminals) {
            if(final_tree_vertices.find(terminal) == final_tree_vertices.end()) {
                all_terminals_in_final_tree = false;
                break;
            }
        }
    } else {
        all_terminals_in_final_tree = false; // No tree found, so terminals aren't connected
    }


    // The final cost is the cost of the MST on G_S
    std::cout << "--- KMB 2-Approximation Algorithm Result ---" << std::endl;
    if (final_steiner_tree_result.first >= INF || !all_terminals_in_final_tree) {
         std::cout << "Could not construct a connected subgraph spanning terminals using unblocked paths." << std::endl;
         std::cout << "Total Cost: INF" << std::endl;
         std::cout << "Edges in Steiner Tree (0 edges):" << std::endl;
    } else {
         std::cout << "Total Cost: " << final_steiner_tree_result.first << std::endl;
         std::cout << "Edges in Steiner Tree (" << final_steiner_tree_result.second.size() << " edges):" << std::endl;
         for (const auto &edge : final_steiner_tree_result.second)
         {
            // Output format consistent with input (0 for not blocked, 1 for blocked)
            std::cout << edge.u << " " << edge.v << " " << edge.weight << " " << (edge.is_blocked ? 1 : 0) << std::endl;
         }
    }


    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Approx took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds\n";

    return 0;
}
