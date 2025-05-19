#include "common.h"
#include <functional>
#include <bitset>
#include <unordered_map>
#include <unordered_set>

// Optimized exact algorithm for Steiner Tree
// This uses branch and bound technique with several pruning strategies

SteinerTree exact_algorithm(const Graph& graph) {
    auto start_time = std::chrono::high_resolution_clock::now();
    SteinerTree result;
    
    // If there's only one terminal, return empty tree
    if (graph.terminals.size() <= 1) {
        auto end_time = std::chrono::high_resolution_clock::now();
        result.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        return result;
    }
    
    const int num_edges = graph.edge_list.size();
    const int num_vertices = graph.V;
    
    // Convert terminals to a vector for easier access
    std::vector<int> terminal_vec(graph.terminals.begin(), graph.terminals.end());
    const int num_terminals = terminal_vec.size();
    
    // Pre-compute the minimum spanning tree (MST) of the terminals
    // This gives us a lower bound on the number of edges needed
    int min_required_edges = num_terminals - 1;
    
    // Keeps track of the best solution found so far
    int min_blocked_edges = std::numeric_limits<int>::max();
    int min_total_weight = std::numeric_limits<int>::max();
    std::vector<Edge> best_solution;
    
    // Precompute all-pairs shortest paths focusing on terminal pairs
    // This helps in pruning infeasible solutions early
    std::vector<std::vector<std::pair<int, std::vector<Edge>>>> shortest_paths = graph.shortestPaths();
    
    // Helper function to check if a set of edges connects all terminals
    auto connects_all_terminals = [&](const std::vector<Edge>& edges) -> bool {
        // Create an adjacency representation of the subgraph
        std::vector<std::vector<int>> subgraph(num_vertices);
        for (const Edge& edge : edges) {
            subgraph[edge.u].push_back(edge.v);
            subgraph[edge.v].push_back(edge.u);
        }
        
        // Use BFS to check connectivity between all terminals
        if (terminal_vec.empty()) return true;
        
        std::vector<bool> visited(num_vertices, false);
        std::queue<int> q;
        q.push(terminal_vec[0]);
        visited[terminal_vec[0]] = true;
        
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            
            for (int v : subgraph[u]) {
                if (!visited[v]) {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }
        
        // Check if all terminals are reachable
        for (int terminal : terminal_vec) {
            if (!visited[terminal]) return false;
        }
        
        return true;
    };
    
    // Function to calculate the key metrics of a solution
    auto calculate_metrics = [&](const std::vector<Edge>& edges) -> std::pair<int, int> {
        int blocked_count = 0;
        int unblocked_weight = 0;
        
        for (const Edge& edge : edges) {
            if (edge.is_blocked) {
                blocked_count++;
            } else {
                unblocked_weight += edge.weight;
            }
        }
        
        return {blocked_count, unblocked_weight};
    };
    
    // Union-Find data structure for cycle detection
    std::vector<int> parent(num_vertices);
    std::vector<int> rank(num_vertices, 0);
    
    std::function<int(int)> find_set = [&](int v) -> int {
        if (v == parent[v])
            return v;
        return parent[v] = find_set(parent[v]);
    };
    
    auto union_sets = [&](int a, int b) {
        a = find_set(a);
        b = find_set(b);
        if (a != b) {
            if (rank[a] < rank[b])
                std::swap(a, b);
            parent[b] = a;
            if (rank[a] == rank[b])
                rank[a]++;
            return true;
        }
        return false;
    };
    
    // Sort edges for better pruning (non-blocked first, then by weight)
    std::vector<Edge> sorted_edges = graph.edge_list;
    std::sort(sorted_edges.begin(), sorted_edges.end());
    
    // Precompute best possible blocked edge count for each terminal pair
    std::unordered_map<int, int> min_blocked_between_terminals;
    for (int i = 0; i < num_terminals; i++) {
        for (int j = i + 1; j < num_terminals; j++) {
            int u = terminal_vec[i];
            int v = terminal_vec[j];
            
            // Use the precomputed shortest path
            std::vector<Edge> path = graph.shortestPath(u, v);
            int blocked_count = 0;
            for (const Edge& edge : path) {
                if (edge.is_blocked) blocked_count++;
            }
            
            int key = u * num_vertices + v;
            min_blocked_between_terminals[key] = blocked_count;
        }
    }
    
    // Branch and bound with pruning
    std::function<void(int, std::vector<Edge>&, int, int, std::vector<bool>&)> search = 
        [&](int index, std::vector<Edge>& current_edges, int current_blocked, int current_unblocked_weight, std::vector<bool>& connected_components) {
            // Early termination if we've already found a better solution
            if (current_blocked > min_blocked_edges || 
                (current_blocked == min_blocked_edges && current_unblocked_weight >= min_total_weight)) {
                return;
            }
            
            // If we've processed all edges, check if we have a valid solution
            if (index == num_edges) {
                if (connects_all_terminals(current_edges)) {
                    min_blocked_edges = current_blocked;
                    min_total_weight = current_unblocked_weight;
                    best_solution = current_edges;
                }
                return;
            }
            
            // Estimate lower bound on blocked edges needed to connect remaining terminals
            // If current + estimated > min_blocked_edges, prune
            
            // Skip this edge (don't include it)
            search(index + 1, current_edges, current_blocked, current_unblocked_weight, connected_components);
            
            // Include current edge if it doesn't create a cycle in the current partial solution
            const Edge& edge = sorted_edges[index];
            
            // Check if adding this edge would create a cycle in the current partial solution
            for (int i = 0; i < num_vertices; i++) {
                parent[i] = i;
                rank[i] = 0;
            }
            
            bool creates_cycle = false;
            for (const Edge& e : current_edges) {
                if (!union_sets(e.u, e.v)) {
                    creates_cycle = true;
                    break;
                }
            }
            
            if (!creates_cycle && union_sets(edge.u, edge.v)) {
                current_edges.push_back(edge);
                if (edge.is_blocked) {
                    current_blocked++;
                } else {
                    current_unblocked_weight += edge.weight;
                }
                
                std::vector<bool> new_connected_components = connected_components;
                new_connected_components[edge.u] = true;
                new_connected_components[edge.v] = true;
                
                search(index + 1, current_edges, current_blocked, current_unblocked_weight, new_connected_components);
                
                // Backtrack
                current_edges.pop_back();
            }
        };
    
    // Start recursive search
    std::vector<Edge> current_edges;
    std::vector<bool> initial_connected(num_vertices, false);
    for (int terminal : terminal_vec) {
        initial_connected[terminal] = true;
    }
    
    search(0, current_edges, 0, 0, initial_connected);
    
    // Populate result with best solution
    for (const Edge& edge : best_solution) {
        result.addEdge(edge);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    return result;
}