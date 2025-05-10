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
    bool is_blocked;

    bool operator<(const Edge &other) const
    {
        if (is_blocked != other.is_blocked)
        {
            return !is_blocked;
        }
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
    int u_idx, v_idx;
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
    DSU(int n)
    {
        parent.resize(n);
        for (int i = 0; i < n; ++i)
        {
            parent[i] = i;
        }
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
        }
    }
};

std::pair<std::vector<long long>, std::vector<int>> dijkstra(
    int start_node,
    int num_vertices,
    const std::vector<Edge> &all_edges,
    const std::vector<std::vector<std::pair<int, int>>> &adj)
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

        for (const auto &edge_pair : adj[u])
        {
            int v = edge_pair.first;
            int edge_idx = edge_pair.second;

            long long weight = all_edges[edge_idx].weight;

            if (dist[u] != INF && weight != INF)
            {
                if (dist[u] > INF - weight)
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

std::pair<long long, std::vector<Edge>> kruskal_edges(
    std::vector<Edge> edges,
    const std::vector<int> &active_vertices)
{
    std::sort(edges.begin(), edges.end());

    std::map<int, int> vertex_to_dsu_idx;
    int dsu_idx_counter = 0;
    for (int v_orig : active_vertices)
    {
        vertex_to_dsu_idx[v_orig] = dsu_idx_counter++;
    }

    if (dsu_idx_counter == 0)
    {
        return {0, {}};
    }

    DSU dsu(dsu_idx_counter);
    long long mst_cost = 0;
    std::vector<Edge> mst_edges;

    for (const auto &edge : edges)
    {

        auto it_u = vertex_to_dsu_idx.find(edge.u);
        auto it_v = vertex_to_dsu_idx.find(edge.v);

        if (it_u == vertex_to_dsu_idx.end() || it_v == vertex_to_dsu_idx.end())
        {
            continue;
        }

        int u_dsu = it_u->second;
        int v_dsu = it_v->second;

        if (dsu.find(u_dsu) != dsu.find(v_dsu))
        {
            dsu.unite(u_dsu, v_dsu);
            mst_cost += edge.weight;
            mst_edges.push_back(edge);
        }
    }

    return {mst_cost, mst_edges};
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

    if (curr < 0 || curr >= num_vertices || pred_edge_idx.empty() || pred_edge_idx.size() != static_cast<size_t>(num_vertices))
    {
        std::cerr << "Error in reconstruct_path: Invalid end_node or pred_edge_idx vector size." << std::endl;
        return {};
    }

    size_t safeguard_counter = 0;
    const size_t max_path_len = num_vertices;

    while (curr != start_node && pred_edge_idx[curr] != -1 && safeguard_counter <= max_path_len)
    {
        int edge_on_path_idx = pred_edge_idx[curr];

        if (edge_on_path_idx < 0 || static_cast<size_t>(edge_on_path_idx) >= all_edges_original.size())
        {
            std::cerr << "Error in reconstruct_path: Invalid edge index " << edge_on_path_idx << " in pred_edge_idx[" << curr << "]. Check Dijkstra/pred_edge_idx population." << std::endl;
            return {};
        }

        path_indices.push_back(edge_on_path_idx);
        const Edge &edge_on_path = all_edges_original[edge_on_path_idx];

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
            std::cerr << "Error in reconstruct_path: Edge " << edge_on_path_idx << " (" << edge_on_path.u << "," << edge_on_path.v << ") does not connect to current vertex " << curr << ". Check edge data or pred_edge_idx consistency." << std::endl;
            return {};
        }
        safeguard_counter++;
    }

    if (curr != start_node && start_node != end_node)
    {
        std::cerr << "Warning: Path reconstruction from " << start_node << " to " << end_node << " failed to reach start node. Reached vertex " << curr << ". Check Dijkstra results or graph connectivity." << std::endl;
        return {};
    }

    std::reverse(path_indices.begin(), path_indices.end());
    return path_indices;
}

int main()
{

    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);
    std::cout.tie(NULL);

    auto t1 = std::chrono::high_resolution_clock::now();

    int num_vertices;
    long long num_edges_input;
    std::cout << "Enter number of vertices and number of edges: ";
    std::cin >> num_vertices >> num_edges_input;

    if (num_vertices <= 0 || num_edges_input < 0)
    {
        std::cerr << "Invalid number of vertices or edges." << std::endl;
        return 1;
    }

    std::vector<Edge> all_edges_original;
    if (num_edges_input > 0)
    {
        try
        {
            all_edges_original.reserve(num_edges_input);
        }
        catch (const std::bad_alloc &e)
        {
            std::cerr << "Failed to reserve memory for all_edges_original (" << num_edges_input << " edges): " << e.what() << std::endl;
            return 1;
        }
    }

    std::vector<std::vector<std::pair<int, int>>> adj;
    if (num_vertices > 0)
    {
        try
        {
            adj.resize(num_vertices);
        }
        catch (const std::bad_alloc &e)
        {
            std::cerr << "Failed to allocate memory for adjacency list (" << num_vertices << " vertices): " << e.what() << std::endl;
            return 1;
        }
    }

    std::cout << "Enter edges (u v weight is_blocked(0 or 1)):" << std::endl;
    for (int i = 0; i < num_edges_input; ++i)
    {
        int u, v;
        long long w;
        int blocked_flag;
        std::cin >> u >> v >> w >> blocked_flag;

        if (u < 0 || u >= num_vertices || v < 0 || v >= num_vertices)
        {
            std::cerr << "Warning: Invalid vertex index for edge " << i << " -- skipping edge (" << u << ", " << v << ")." << std::endl;
            continue;
        }

        if (u == v)
        {
            std::cerr << "Warning: Self-loop detected for vertex " << u << " on edge " << i << " -- skipping." << std::endl;
            continue;
        }

        bool is_blocked = (blocked_flag == 1);

        all_edges_original.push_back({u, v, w, (int)all_edges_original.size(), is_blocked});

        if (!is_blocked)
        {

            adj[u].push_back({v, (int)all_edges_original.size() - 1});
            adj[v].push_back({u, (int)all_edges_original.size() - 1});
        }
    }

    int num_terminals_input;
    std::cout << "Enter number of terminals: ";
    std::cin >> num_terminals_input;
    std::vector<int> terminals_input(num_terminals_input);
    std::set<int> terminal_set;
    std::cout << "Enter terminal vertices:" << std::endl;
    for (int i = 0; i < num_terminals_input; ++i)
    {
        std::cin >> terminals_input[i];

        if (terminals_input[i] < 0 || terminals_input[i] >= num_vertices)
        {
            std::cerr << "Warning: Invalid terminal vertex index " << terminals_input[i] << " -- skipping." << std::endl;
            continue;
        }
        terminal_set.insert(terminals_input[i]);
    }

    std::vector<int> terminals(terminal_set.begin(), terminal_set.end());
    int num_terminals = terminals.size();

    if (num_terminals <= 1)
    {
        std::cout << "\n--- KMB 2-Approximation Algorithm Result ---" << std::endl;
        std::cout << "Total Cost: 0" << std::endl;
        std::cout << "Edges in Steiner Tree: (empty for 0 or 1 terminals)" << std::endl;
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
        try
        {
            term_dist.resize(num_terminals, std::vector<long long>(num_vertices, INF));
            term_pred_edge_idx.resize(num_terminals, std::vector<int>(num_vertices, -1));
        }
        catch (const std::bad_alloc &e)
        {
            std::cerr << "Failed to allocate memory for terminal Dijkstra results (" << num_terminals << "x" << num_vertices << "): " << e.what() << std::endl;
            return 1;
        }
    }

    std::cout << "Running Dijkstra from each terminal..." << std::endl;
    for (int i = 0; i < num_terminals; ++i)
    {
        int start_term = terminals[i];
        std::tie(term_dist[i], term_pred_edge_idx[i]) = dijkstra(start_term, num_vertices, all_edges_original, adj);
    }

    std::vector<SuperEdge> terminal_graph_super_edges;
    terminal_graph_super_edges.reserve(num_terminals * (num_terminals - 1) / 2);

    bool possible_to_connect_all_terminals = true;
    if (num_terminals > 1)
    {

        for (int i = 1; i < num_terminals; ++i)
        {
            if (term_dist[0][terminals[i]] == INF)
            {
                std::cerr << "Warning: Terminal " << terminals[i] << " (" << i << ") is not reachable from terminal " << terminals[0] << " (0) using unblocked paths." << std::endl;
                possible_to_connect_all_terminals = false;
                break;
            }
        }

        if (possible_to_connect_all_terminals)
        {
            for (int i = 0; i < num_terminals; ++i)
            {
                for (int j = i + 1; j < num_terminals; ++j)
                {
                    int end_term = terminals[j];
                    long long path_cost = term_dist[i][end_term];

                    if (path_cost != INF)
                    {

                        terminal_graph_super_edges.push_back({i, j, path_cost});
                    }
                    else
                    {

                        std::cerr << "Internal Warning: Unexpected INF path cost between terminals " << terminals[i] << " and " << terminals[j] << " during super-edge creation." << std::endl;
                    }
                }
            }
        }
    }

    if (!possible_to_connect_all_terminals || terminal_graph_super_edges.empty())
    {

        std::cout << "\n--- KMB 2-Approximation Algorithm Result ---" << std::endl;
        std::cout << "Could not connect all terminals using unblocked paths." << std::endl;
        std::cout << "Total Cost: INF" << std::endl;
        return 0;
    }

    DSU terminal_dsu(num_terminals);
    long long mst_on_terminals_cost = 0;
    std::vector<SuperEdge> mst_on_terminals_super_edges;
    mst_on_terminals_super_edges.reserve(num_terminals - 1);

    std::sort(terminal_graph_super_edges.begin(), terminal_graph_super_edges.end());

    std::cout << "Finding MST on terminal graph..." << std::endl;
    for (const auto &super_edge : terminal_graph_super_edges)
    {

        int u_dsu = super_edge.u_idx;
        int v_dsu = super_edge.v_idx;

        if (terminal_dsu.find(u_dsu) != terminal_dsu.find(v_dsu))
        {
            terminal_dsu.unite(u_dsu, v_dsu);
            mst_on_terminals_cost += super_edge.weight;
            mst_on_terminals_super_edges.push_back(super_edge);

            if (mst_on_terminals_super_edges.size() == static_cast<size_t>(num_terminals - 1))
            {
                break;
            }
        }
    }

    bool terminal_mst_fully_connected = true;
    if (num_terminals > 1)
    {

        int first_root = terminal_dsu.find(0);

        for (int i = 1; i < num_terminals; ++i)
        {
            if (terminal_dsu.find(i) != first_root)
            {
                terminal_mst_fully_connected = false;
                break;
            }
        }
    }
    else
    {
        terminal_mst_fully_connected = true;
    }

    if (!terminal_mst_fully_connected)
    {
        std::cout << "\n--- KMB 2-Approximation Algorithm Result ---" << std::endl;
        std::cout << "MST on terminals could not connect all terminals. Graph might be disconnected or paths don't exist." << std::endl;
        std::cout << "Total Cost: INF" << std::endl;
        return 0;
    }

    std::set<int> Gs_edge_indices;
    std::set<int> Gs_vertices_set;

    std::cout << "Reconstructing paths for terminal MST edges and building G_S..." << std::endl;
    for (const auto &super_edge_in_Mt : mst_on_terminals_super_edges)
    {

        int term1_vertex = terminals[super_edge_in_Mt.u_idx];
        int term2_vertex = terminals[super_edge_in_Mt.v_idx];

        const std::vector<int> &pred_edge_idx_from_term1 = term_pred_edge_idx[super_edge_in_Mt.u_idx];

        std::vector<int> original_edge_indices_for_path =
            reconstruct_path(term1_vertex, term2_vertex, pred_edge_idx_from_term1, all_edges_original, num_vertices);

        for (int original_edge_idx : original_edge_indices_for_path)
        {

            if (original_edge_idx >= 0 && static_cast<size_t>(original_edge_idx) < all_edges_original.size())
            {
                Gs_edge_indices.insert(original_edge_idx);
                const Edge &original_edge = all_edges_original[original_edge_idx];
                Gs_vertices_set.insert(original_edge.u);
                Gs_vertices_set.insert(original_edge.v);
            }
            else
            {
                std::cerr << "Error: Reconstructed path returned invalid edge index: " << original_edge_idx << " from path between " << term1_vertex << " and " << term2_vertex << std::endl;
            }
        }
    }

    for (int t : terminals)
    {
        Gs_vertices_set.insert(t);
    }

    std::vector<Edge> Gs_edges;
    Gs_edges.reserve(Gs_edge_indices.size());
    for (int edge_idx : Gs_edge_indices)
    {
        Gs_edges.push_back(all_edges_original[edge_idx]);
    }
    std::vector<int> Gs_active_vertices(Gs_vertices_set.begin(), Gs_vertices_set.end());

    std::cout << "Finding MST on subgraph G_S..." << std::endl;
    std::pair<long long, std::vector<Edge>> final_steiner_tree_result =
        kruskal_edges(Gs_edges, Gs_active_vertices);

    std::cout << "\n--- KMB 2-Approximation Algorithm Result ---" << std::endl;
    std::cout << "Total Cost: " << final_steiner_tree_result.first << std::endl;
    std::cout << "Edges in Steiner Tree (" << final_steiner_tree_result.second.size() << " edges):" << std::endl;
    for (const auto &edge : final_steiner_tree_result.second)
    {
        std::cout << edge.u << " " << edge.v << " " << edge.weight << " " << (edge.is_blocked ? 1 : 0) << std::endl;
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Approx took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds\n";

    return 0;
}