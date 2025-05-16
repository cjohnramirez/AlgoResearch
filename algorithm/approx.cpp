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

const long long BLOCKED_EDGE_PENALTY = 50000LL * 1000000000LL + 7LL;

struct Edge
{
    int u, v;
    long long weight;
    int id;
    bool is_blocked;

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
    int num_components;

    DSU(int n)
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

std::pair<std::vector<long long>, std::vector<int>> dijkstra_with_penalties(
    int start_node,
    int num_vertices,
    const std::vector<Edge> &all_edges_original,
    const std::vector<std::vector<std::pair<int, int>>> &adj_all_edges_indices)
{
    std::vector<long long> dist_effective(num_vertices, INF);
    std::vector<int> pred_edge_idx(num_vertices, -1);

    dist_effective[start_node] = 0;
    std::priority_queue<std::pair<long long, int>, std::vector<std::pair<long long, int>>, std::greater<std::pair<long long, int>>> pq;
    pq.push({0, start_node});

    while (!pq.empty())
    {
        long long d_eff = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d_eff > dist_effective[u])
        {
            continue;
        }

        for (const auto &edge_pair : adj_all_edges_indices[u])
        {
            int v = edge_pair.first;
            int edge_idx = edge_pair.second;

            if (edge_idx < 0 || static_cast<size_t>(edge_idx) >= all_edges_original.size())
            {

                continue;
            }

            const Edge &current_edge = all_edges_original[edge_idx];
            long long true_weight = current_edge.weight;
            long long penalty = current_edge.is_blocked ? BLOCKED_EDGE_PENALTY : 0;
            long long effective_edge_weight = -1;

            if (true_weight > INF - penalty)
            {
                effective_edge_weight = INF;
            }
            else
            {
                effective_edge_weight = true_weight + penalty;
            }

            if (dist_effective[u] != INF && effective_edge_weight != INF)
            {
                if (dist_effective[u] > INF - effective_edge_weight)
                {

                    continue;
                }
                if (dist_effective[u] + effective_edge_weight < dist_effective[v])
                {
                    dist_effective[v] = dist_effective[u] + effective_edge_weight;
                    pred_edge_idx[v] = edge_idx;
                    pq.push({dist_effective[v], v});
                }
            }
        }
    }
    return {dist_effective, pred_edge_idx};
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

    if (start_node == end_node)
        return {};
    if (curr < 0 || curr >= num_vertices || pred_edge_idx.empty() ||
        pred_edge_idx.size() != static_cast<size_t>(num_vertices) ||
        (pred_edge_idx[curr] == -1 && start_node != end_node))
    {
        return {};
    }

    size_t safeguard_counter = 0;
    const size_t max_path_len = num_vertices + 5;

    while (curr != start_node && pred_edge_idx[curr] != -1 && safeguard_counter <= max_path_len)
    {
        int edge_on_path_idx = pred_edge_idx[curr];

        if (edge_on_path_idx < 0 || static_cast<size_t>(edge_on_path_idx) >= all_edges_original.size())
        {
            std::cerr << "Error in reconstruct_path: Invalid edge index " << edge_on_path_idx << " in pred_edge_idx[" << curr << "]." << std::endl;
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
            std::cerr << "Error in reconstruct_path: Edge " << edge_on_path_idx << " with ends (" << edge_on_path.u << "," << edge_on_path.v << ") does not connect to current vertex " << curr << "." << std::endl;
            return {};
        }
        safeguard_counter++;
    }

    if (curr != start_node && start_node != end_node)
    {

        if (safeguard_counter > max_path_len)
        {
            std::cerr << "Warning: Path reconstruction exceeded max length from " << start_node << " to " << end_node << ". Potential cycle or path issue." << std::endl;
        }

        return {};
    }

    std::reverse(path_indices.begin(), path_indices.end());
    return path_indices;
}

std::pair<long long, std::vector<Edge>> kruskal_edges(
    std::vector<Edge> edges,
    const std::vector<int> &active_vertices,
    const std::set<int> &active_vertices_set)
{
    if (active_vertices.empty())
    {
        return {0, {}};
    }
    if (active_vertices.size() == 1)
    {
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

        auto it_u = active_vertices_set.find(edge.u);
        auto it_v = active_vertices_set.find(edge.v);

        if (it_u == active_vertices_set.end() || it_v == active_vertices_set.end())
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

            if (dsu.num_components == 1 && active_vertices.size() > 1)
                break;
            if (mst_edges.size() == active_vertices.size() - 1 && active_vertices.size() > 0)
                break;
        }
    }

    bool all_active_connected = (active_vertices.empty() || dsu.num_components == 1);

    if (all_active_connected)
    {
        return {mst_cost, mst_edges};
    }
    else
    {

        return {INF, {}};
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

        if (num_vertices <= 0)
        {
            std::cerr << "Invalid number of vertices." << std::endl;
            return 1;
        }
        if (num_edges_input < 0)
        {
            std::cerr << "Invalid number of edges." << std::endl;
            return 1;
        }
    }

    std::vector<Edge> all_edges_original;
    all_edges_original.reserve(num_edges_input);

    std::vector<std::vector<std::pair<int, int>>> adj_all_edges_indices;
    if (num_vertices > 0)
    {
        adj_all_edges_indices.resize(num_vertices);
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
        all_edges_original.push_back({u, v, w, (int)all_edges_original.size(), is_blocked});

        adj_all_edges_indices[u].push_back({v, (int)all_edges_original.size() - 1});
        adj_all_edges_indices[v].push_back({u, (int)all_edges_original.size() - 1});
    }

    int num_terminals_input;
    std::cin >> num_terminals_input;
    std::vector<int> terminals_input_raw(num_terminals_input);
    std::set<int> terminal_set;
    for (int i = 0; i < num_terminals_input; ++i)
    {
        std::cin >> terminals_input_raw[i];
        if (terminals_input_raw[i] < 0 || terminals_input_raw[i] >= num_vertices)
        {

            continue;
        }
        terminal_set.insert(terminals_input_raw[i]);
    }

    std::vector<int> terminals(terminal_set.begin(), terminal_set.end());
    int num_terminals = terminals.size();

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

    std::vector<std::vector<long long>> term_dist_effective;
    std::vector<std::vector<int>> term_pred_edge_idx;

    if (num_terminals > 0 && num_vertices > 0)
    {
        term_dist_effective.resize(num_terminals, std::vector<long long>(num_vertices, INF));
        term_pred_edge_idx.resize(num_terminals, std::vector<int>(num_vertices, -1));
    }

    for (int i = 0; i < num_terminals; ++i)
    {
        int start_term = terminals[i];
        if (start_term >= 0 && start_term < num_vertices)
        {
            std::tie(term_dist_effective[i], term_pred_edge_idx[i]) =
                dijkstra_with_penalties(start_term, num_vertices, all_edges_original, adj_all_edges_indices);
        }
    }

    std::vector<SuperEdge> terminal_graph_super_edges;
    if (num_terminals > 1)
    {
        terminal_graph_super_edges.reserve(num_terminals * (num_terminals - 1) / 2);
    }

    bool possible_to_connect_all_terminals = true;
    if (num_terminals > 1)
    {
        for (int i = 0; i < num_terminals; ++i)
        {
            for (int j = i + 1; j < num_terminals; ++j)
            {
                long long effective_path_cost = term_dist_effective[i][terminals[j]];

                if (effective_path_cost < INF)
                {
                    std::vector<int> path_indices = reconstruct_path(terminals[i], terminals[j], term_pred_edge_idx[i], all_edges_original, num_vertices);

                    long long true_path_cost = 0;
                    bool path_reconstruction_ok = true;

                    if (terminals[i] != terminals[j] && path_indices.empty())
                    {

                        if (effective_path_cost == 0)
                        {
                            true_path_cost = 0;
                        }
                        else
                        {

                            path_reconstruction_ok = false;
                        }
                    }
                    else
                    {
                        for (int edge_on_path_idx : path_indices)
                        {
                            if (edge_on_path_idx < 0 || static_cast<size_t>(edge_on_path_idx) >= all_edges_original.size())
                            {

                                path_reconstruction_ok = false;
                                break;
                            }
                            true_path_cost += all_edges_original[edge_on_path_idx].weight;
                        }
                    }

                    if (path_reconstruction_ok)
                    {
                        terminal_graph_super_edges.push_back({i, j, true_path_cost});
                    }
                }
            }
        }

        for (int i = 1; i < num_terminals; ++i)
        {
            if (term_dist_effective[0][terminals[i]] == INF)
            {
                possible_to_connect_all_terminals = false;
                break;
            }
        }
    }

    if (!possible_to_connect_all_terminals || (num_terminals > 1 && terminal_graph_super_edges.empty() && num_edges_input > 0))
    {

        std::cout << "--- KMB 2-Approximation Algorithm Result ---" << std::endl;
        std::cout << "Could not connect all terminals." << std::endl;
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
    if (num_terminals > 1)
    {
        mst_on_terminals_super_edges.reserve(num_terminals - 1);
    }

    std::sort(terminal_graph_super_edges.begin(), terminal_graph_super_edges.end());

    for (const auto &super_edge : terminal_graph_super_edges)
    {
        if (num_terminals <= 1)
            break;
        int u_dsu = super_edge.u_idx;
        int v_dsu = super_edge.v_idx;

        if (terminal_dsu.find(u_dsu) != terminal_dsu.find(v_dsu))
        {
            terminal_dsu.unite(u_dsu, v_dsu);
            mst_on_terminals_super_edges.push_back(super_edge);
            if (mst_on_terminals_super_edges.size() == static_cast<size_t>(num_terminals - 1))
            {
                break;
            }
        }
    }

    if (num_terminals > 1 && mst_on_terminals_super_edges.size() < static_cast<size_t>(num_terminals - 1))
    {
        possible_to_connect_all_terminals = false;
    }

    std::set<int> Gs_edge_indices;
    std::set<int> Gs_vertices_set;

    for (int t : terminals)
    {
        Gs_vertices_set.insert(t);
    }

    if (possible_to_connect_all_terminals)
    {
        for (const auto &super_edge_in_Mt : mst_on_terminals_super_edges)
        {
            int term1_original_vertex = terminals[super_edge_in_Mt.u_idx];
            int term2_original_vertex = terminals[super_edge_in_Mt.v_idx];

            const std::vector<int> &path_preds = term_pred_edge_idx[super_edge_in_Mt.u_idx];

            std::vector<int> original_edge_indices_for_path =
                reconstruct_path(term1_original_vertex, term2_original_vertex, path_preds, all_edges_original, num_vertices);

            if (term1_original_vertex != term2_original_vertex && original_edge_indices_for_path.empty() && super_edge_in_Mt.weight > 0)
            {

                possible_to_connect_all_terminals = false;
                break;
            }

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

                    possible_to_connect_all_terminals = false;
                    break;
                }
            }
            if (!possible_to_connect_all_terminals)
                break;
        }
    }

    std::pair<long long, std::vector<Edge>> final_steiner_tree_result = {INF, {}};

    if (possible_to_connect_all_terminals && num_terminals > 0)
    {
        std::vector<Edge> Gs_edges_vec;
        Gs_edges_vec.reserve(Gs_edge_indices.size());
        for (int edge_idx : Gs_edge_indices)
        {
            Gs_edges_vec.push_back(all_edges_original[edge_idx]);
        }
        std::vector<int> Gs_active_vertices_vec(Gs_vertices_set.begin(), Gs_vertices_set.end());

        if (!Gs_active_vertices_vec.empty())
        {
            final_steiner_tree_result =
                kruskal_edges(Gs_edges_vec, Gs_active_vertices_vec, Gs_vertices_set);
        }
        else if (num_terminals <= 1)
        {
            final_steiner_tree_result = {0, {}};
        }
        else
        {
            final_steiner_tree_result = {INF, {}};
        }

        bool all_terminals_covered_in_final_tree = true;
        if (final_steiner_tree_result.first < INF && num_terminals > 0)
        {
            std::set<int> final_tree_node_set;
            for (const auto &edge : final_steiner_tree_result.second)
            {
                final_tree_node_set.insert(edge.u);
                final_tree_node_set.insert(edge.v);
            }

            if (num_terminals == 1 && !final_tree_node_set.count(terminals[0]) && !Gs_active_vertices_vec.empty())
            {

                if (!Gs_vertices_set.count(terminals[0]))
                    all_terminals_covered_in_final_tree = false;
            }
            else if (num_terminals > 1)
            {

                DSU final_check_dsu(num_vertices);
                for (const auto &edge : final_steiner_tree_result.second)
                {
                    final_check_dsu.unite(edge.u, edge.v);
                }
                for (size_t i = 1; i < terminals.size(); ++i)
                {
                    if (final_check_dsu.find(terminals[0]) != final_check_dsu.find(terminals[i]))
                    {
                        all_terminals_covered_in_final_tree = false;
                        break;
                    }
                }

                if (final_steiner_tree_result.second.empty() && num_terminals > 1)
                {
                    all_terminals_covered_in_final_tree = false;
                }
            }
        }
        else if (num_terminals > 0 && final_steiner_tree_result.first >= INF)
        {
            all_terminals_covered_in_final_tree = false;
        }

        if (!all_terminals_covered_in_final_tree && num_terminals > 0)
        {
            final_steiner_tree_result.first = INF;
            final_steiner_tree_result.second.clear();
        }
    }
    else
    {
        final_steiner_tree_result = {INF, {}};
    }

    std::cout << "--- KMB 2-Approximation Algorithm Result ---" << std::endl;
    if (final_steiner_tree_result.first >= INF)
    {
        std::cout << "Could not construct a connected Steiner tree spanning all terminals." << std::endl;
        std::cout << "Total Cost: INF" << std::endl;
        std::cout << "Edges in Steiner Tree (0 edges):" << std::endl;
    }
    else
    {
        std::cout << "Total Cost: " << final_steiner_tree_result.first << std::endl;
        std::cout << "Edges in Steiner Tree (" << final_steiner_tree_result.second.size() << " edges):" << std::endl;
        std::sort(final_steiner_tree_result.second.begin(), final_steiner_tree_result.second.end());
        for (const auto &edge : final_steiner_tree_result.second)
        {
            std::cout << edge.u << " " << edge.v << " " << edge.weight << " " << (edge.is_blocked ? 1 : 0) << std::endl;
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Approx took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds\n";

    return 0;
}