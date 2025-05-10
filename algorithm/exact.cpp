#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <set>
#include <iomanip>
#include <ctime>
#include <thread>
#include <chrono>

const int INF_EXACT = 1e9;

struct EdgeExact
{
    int u, v;
    int weight;
    int id;
    bool is_blocked;

    bool operator<(const EdgeExact &other) const
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

std::pair<int, std::vector<EdgeExact>> kruskal_exact(
    std::vector<EdgeExact> edges,
    const std::vector<int> &active_vertices)
{
    if (active_vertices.empty())
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

    DSUExact dsu(dsu_idx_counter);
    int mst_cost = 0;
    std::vector<EdgeExact> mst_edges;

    for (const auto &edge : edges)
    {

        if (vertex_to_dsu_idx.find(edge.u) == vertex_to_dsu_idx.end() ||
            vertex_to_dsu_idx.find(edge.v) == vertex_to_dsu_idx.end())
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
        }
    }

    return {mst_cost, mst_edges};
}

void f()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main()
{
    auto t1 = std::chrono::high_resolution_clock::now();
    f();

    int num_vertices, num_edges_input;
    std::cout << "Enter number of vertices and number of edges: ";
    std::cin >> num_vertices >> num_edges_input;

    std::vector<EdgeExact> all_edges_original;
    std::cout << "Enter edges (u v weight is_blocked(0 or 1)):" << std::endl;
    for (int i = 0; i < num_edges_input; ++i)
    {
        int u, v, w, blocked_flag;
        std::cin >> u >> v >> w >> blocked_flag;
        all_edges_original.push_back({u, v, w, i, (blocked_flag == 1)});
    }

    int num_terminals;
    std::cout << "Enter number of terminals: ";
    std::cin >> num_terminals;
    std::vector<int> terminals(num_terminals);
    std::set<int> terminal_set;
    std::cout << "Enter terminal vertices:" << std::endl;
    for (int i = 0; i < num_terminals; ++i)
    {
        std::cin >> terminals[i];
        terminal_set.insert(terminals[i]);
    }

    if (num_terminals <= 1)
    {
        std::cout << "\n--- Exact Algorithm Result ---" << std::endl;
        std::cout << "Total Cost: 0" << std::endl;
        std::cout << "Edges in Steiner Tree:" << std::endl;

        return 0;
    }

    std::vector<int> non_terminals;
    for (int i = 0; i < num_vertices; ++i)
    {
        if (terminal_set.find(i) == terminal_set.end())
        {
            non_terminals.push_back(i);
        }
    }

    int min_steiner_tree_cost = INF_EXACT;
    std::vector<EdgeExact> best_steiner_tree_edges;

    int num_non_terminals = non_terminals.size();

    for (int i = 0; i < (1 << num_non_terminals); ++i)
    {
        std::vector<int> current_active_nodes = terminals;
        std::vector<EdgeExact> current_graph_edges;

        for (int j = 0; j < num_non_terminals; ++j)
        {
            if ((i >> j) & 1)
            {
                current_active_nodes.push_back(non_terminals[j]);
            }
        }

        std::set<int> active_nodes_set(current_active_nodes.begin(), current_active_nodes.end());

        for (const auto &original_edge : all_edges_original)
        {
            if (active_nodes_set.count(original_edge.u) && active_nodes_set.count(original_edge.v))
            {
                current_graph_edges.push_back(original_edge);
            }
        }

        if (current_active_nodes.empty() && num_terminals > 0)
            continue;
        if (current_active_nodes.empty() && num_terminals == 0)
        {
            if (0 < min_steiner_tree_cost)
            {
                min_steiner_tree_cost = 0;
                best_steiner_tree_edges.clear();
            }
            continue;
        }

        std::pair<int, std::vector<EdgeExact>> current_mst_result =
            kruskal_exact(current_graph_edges, current_active_nodes);

        if (!current_mst_result.second.empty() || current_active_nodes.size() == 1)
        {

            std::map<int, int> node_to_dsu_idx;
            int dsu_idx_count = 0;
            for (int node : active_nodes_set)
            {
                node_to_dsu_idx[node] = dsu_idx_count++;
            }

            if (dsu_idx_count == 0 && num_terminals > 0)
                continue;

            DSUExact terminal_connectivity_dsu(dsu_idx_count > 0 ? dsu_idx_count : 1);

            for (const auto &edge_in_mst : current_mst_result.second)
            {

                if (node_to_dsu_idx.count(edge_in_mst.u) && node_to_dsu_idx.count(edge_in_mst.v))
                {
                    terminal_connectivity_dsu.unite(
                        node_to_dsu_idx[edge_in_mst.u],
                        node_to_dsu_idx[edge_in_mst.v]);
                }
            }

            bool all_terminals_connected_in_this_mst = true;
            if (num_terminals > 0)
            {

                if (!node_to_dsu_idx.count(terminals[0]))
                {
                    all_terminals_connected_in_this_mst = false;
                }

                if (all_terminals_connected_in_this_mst)
                {
                    int first_terminal_root = node_to_dsu_idx.count(terminals[0]) ? terminal_connectivity_dsu.find(node_to_dsu_idx[terminals[0]]) : -1;
                    if (first_terminal_root == -1 && num_terminals > 0)
                    {
                        all_terminals_connected_in_this_mst = false;
                    }
                    else
                    {
                        for (size_t k = 1; k < terminals.size(); ++k)
                        {
                            if (!node_to_dsu_idx.count(terminals[k]) ||
                                terminal_connectivity_dsu.find(node_to_dsu_idx[terminals[k]]) != first_terminal_root)
                            {
                                all_terminals_connected_in_this_mst = false;
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                all_terminals_connected_in_this_mst = true;
            }

            if (all_terminals_connected_in_this_mst)
            {
                if (current_mst_result.first < min_steiner_tree_cost)
                {
                    min_steiner_tree_cost = current_mst_result.first;
                    best_steiner_tree_edges = current_mst_result.second;
                }
            }
        }
        else if (num_terminals == 0)
        {
            if (0 < min_steiner_tree_cost)
            {
                min_steiner_tree_cost = 0;
                best_steiner_tree_edges.clear();
            }
        }
        else if (num_terminals > 0 && active_nodes_set.size() == num_terminals)
        {
        }
    }

    std::cout << "\n--- Exact Algorithm Result (Brute-Force Subset Enumeration) ---" << std::endl;
    if (min_steiner_tree_cost == INF_EXACT)
    {
        if (num_terminals > 0)
        {
            std::cout << "Could not connect all terminals." << std::endl;
            std::cout << "Total Cost: INF" << std::endl;
        }
        else
        {
            std::cout << "Total Cost: 0" << std::endl;
            std::cout << "Edges in Steiner Tree (0 edges):" << std::endl;
        }
    }
    else
    {
        std::cout << "Total Cost: " << min_steiner_tree_cost << std::endl;
        std::cout << "Edges in Steiner Tree (" << best_steiner_tree_edges.size() << " edges):" << std::endl;
        for (const auto &edge : best_steiner_tree_edges)
        {
            std::cout << edge.u << " " << edge.v << " " << edge.weight;

            if (edge.is_blocked)
            {
                std::cout << " 0";
            }
            else
            {
                std::cout << " 1";
            }
            std::cout << std::endl;
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Exact took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds\n";

    return 0;
}
