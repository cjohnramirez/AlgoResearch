#include "common.h"
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file>" << std::endl;
        return 1;
    }
    
    std::string input_file = argv[1];
    Graph graph = Graph::parseInput(input_file);
    
    std::cout << "Running KMB 2-approximation algorithm..." << std::endl;
    SteinerTree kmb_result = kmb_algorithm(graph);
    std::cout << "KMB 2-approximation algorithm complete." << std::endl;
    kmb_result.print_results();
    
    std::cout << "Running exact brute-force algorithm..." << std::endl;
    std::cout << "(Warning: This may take a long time for large graphs)" << std::endl;
    SteinerTree exact_result = exact_algorithm(graph);
    std::cout << "Exact brute-force algorithm complete." << std::endl;
    exact_result.print_results();
    
    // Compare results
    std::cout << "Comparison:" << std::endl;
    std::cout << "KMB 2-approximation: " << kmb_result.blocked_edges.size() << " blocked edges, total weight: " 
              << kmb_result.total_weight << " (runtime: " << kmb_result.runtime << " ms)" << std::endl;
    std::cout << "Exact brute-force: " << exact_result.blocked_edges.size() << " blocked edges, total weight: " 
              << exact_result.total_weight << " (runtime: " << exact_result.runtime << " ms)" << std::endl;
    
    // Calculate approximation ratio
    double weight_ratio = static_cast<double>(kmb_result.total_weight) / exact_result.total_weight;
    std::cout << "Approximation ratio (weight): " << weight_ratio << std::endl;
    
    return 0;
}