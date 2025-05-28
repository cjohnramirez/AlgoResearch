#include "common.h"
#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_file>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    Graph graph = Graph::parseInput(input_file);

    std::ofstream outfile("results.txt");

    if (!outfile.is_open())
    {
        std::cerr << "Error: Could not open results.txt for writing." << std::endl;
        return 1;
    }

    if (outfile.is_open())
    {
        outfile << "Running KMB 2-approximation algorithm..." << std::endl;
        SteinerTree kmb_result = kmb_algorithm(graph);
        outfile << "KMB 2-approximation algorithm complete." << std::endl;
        kmb_result.print_results();
    
        outfile << "Running exact brute-force algorithm..." << std::endl;
        outfile << "(Warning: This may take a long time for large graphs)" << std::endl;
        SteinerTree exact_result = exact_algorithm(graph);
        outfile << "Exact brute-force algorithm complete." << std::endl;
        exact_result.print_results();
    
        // Compare results
        outfile << "Comparison:" << std::endl;
        outfile << "KMB 2-approximation: " << kmb_result.blocked_edges.size() << " blocked edges, blocked weight: "
                  << kmb_result.blockedWeight() << ", unblocked weight: "
                  << kmb_result.total_weight << " (runtime: " << kmb_result.runtime << " ms)" << std::endl;
        outfile << "Exact brute-force: " << exact_result.blocked_edges.size() << " blocked edges, blocked weight: "
                  << exact_result.blockedWeight() << ", unblocked weight: "
                  << exact_result.total_weight << " (runtime: " << exact_result.runtime << " ms)" << std::endl;
    
        // Calculate approximation ratio
        double weight_ratio = static_cast<double>(kmb_result.blockedWeight()) / exact_result.blockedWeight();
        outfile << "Approximation ratio (blocked weight): " << weight_ratio << std::endl;
    }
    outfile.close();

    return 0;
}