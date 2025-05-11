import networkx as nx
import matplotlib.pyplot as plt

def graph_partitioning(G, plotting=True):
    """Partition a directed graph into a list of subgraphs that contain
    only entirely supported or entirely unsupported nodes.
    """
    # Categorize nodes by their node_type attribute
    supported_nodes = {n for n, d in G.nodes(data="node_type") if d == "supported"}
    unsupported_nodes = {n for n, d in G.nodes(data="node_type") if d == "unsupported"}

    # Make a copy of the graph.
    H = G.copy()
    # Remove all edges connecting supported and unsupported nodes.
    H.remove_edges_from(
        (n, nbr, d)
        for n, nbrs in G.adj.items()
        if n in supported_nodes
        for nbr, d in nbrs.items()
        if nbr in unsupported_nodes
    )
    H.remove_edges_from(
        (n, nbr, d)
        for n, nbrs in G.adj.items()
        if n in unsupported_nodes
        for nbr, d in nbrs.items()
        if nbr in supported_nodes
    )

    # Collect all removed edges for reconstruction.
    G_minus_H = nx.DiGraph()
    G_minus_H.add_edges_from(set(G.edges) - set(H.edges))

    if plotting:
        # Plot the stripped graph with the edges removed.
        _node_colors = [c for _, c in H.nodes(data="node_color")]
        _pos = nx.spring_layout(H)
        plt.figure(figsize=(8, 8))
        nx.draw_networkx_edges(H, _pos, alpha=0.3, edge_color="k")
        nx.draw_networkx_nodes(H, _pos, pos=_pos, node_color=_node_colors)
        nx.draw_networkx_labels(H, _pos, font_size=14)
        plt.axis("off")
        plt.title("The stripped graph with the edges removed.")
        plt.show()
        # Plot the edges removed.
        _pos = nx.spring_layout(G_minus_H)
        plt.figure(figsize=(8, 8))
        ncl = [G.nodes[n]["node_color"] for n in G_minus_H.nodes]
        nx.draw_networkx_edges(G_minus_H, _pos, alpha=0.3, edge_color="k")
        nx.draw_networkx_nodes(G_minus_H, _pos, node_color=ncl)
        nx.draw_networkx_labels(G_minus_H, _pos, font_size=14)
        plt.axis("off")
        plt.title("The removed edges.")
        plt.show()

    # Find the connected components in the stripped undirected graph.
    # And use the sets, specifying the components, to partition
    # the original directed graph into a list of directed subgraphs
    # that contain only entirely supported or entirely unsupported nodes.
    subgraphs = [
        H.subgraph(c).copy() for c in nx.connected_components(H.to_undirected())
    ]

    return subgraphs, G_minus_H

def visualize_graph(edges, terminal_vertices=None): # Added terminal_vertices as an optional argument.
    """
    Visualizes an undirected graph with optional terminal vertices, handling disconnected subgraphs,
    and can also perform graph partitioning if node attributes 'node_type' and 'node_color' are present.

    Args:
        edges: A list of tuples, where each tuple represents an edge (u, v, weight, is_blocked).
        terminal_vertices: (Optional) A list of vertex numbers that are terminal.
    """
    # Create an undirected graph
    graph = nx.Graph()

    # Add edges
    for u, v, weight, is_blocked in edges:
        graph.add_edge(u, v)

    # Check for node attributes to determine if graph partitioning should be applied.
    if all(graph.nodes[n].get('node_type') for n in graph.nodes()):
        print("Performing graph partitioning...")
        subgraphs, removed_edges = graph_partitioning(graph.copy(), plotting=True) #make a copy to not alter the original graph
        # Plot the results: every subgraph in the list.
        for i, subgraph in enumerate(subgraphs):
            _pos = nx.spring_layout(subgraph)
            plt.figure(figsize=(8, 8))
            nx.draw_networkx_edges(subgraph, _pos, alpha=0.3, edge_color="k")
            node_color_list_c = [subgraph.nodes[n].get("node_color", "skyblue") for n in subgraph.nodes()]
            nx.draw_networkx_nodes(subgraph, _pos, node_color=node_color_list_c)
            nx.draw_networkx_labels(subgraph, _pos, font_size=14)
            plt.axis("off")
            plt.title(f"Subgraph {i+1}")
            plt.show()

        # Reconstruct the graph and plot
        G_ex_r = nx.DiGraph()
        # Composing all subgraphs.
        for subgraph in subgraphs:
            G_ex_r = nx.compose(G_ex_r, subgraph)
        # Adding the previously stored edges.
        G_ex_r.add_edges_from(removed_edges.edges())

        # Check that the original graph and the reconstructed graphs are isomorphic.
        if nx.is_isomorphic(graph, G_ex_r):
            print("Original and reconstructed graphs are isomorphic.")
        else:
            print("Original and reconstructed graphs are NOT isomorphic.")

        # Plot the reconstructed graph.
        node_color_list_r = [G_ex_r.nodes[n].get("node_color", "skyblue") for n in G_ex_r.nodes()]
        pos_r = nx.spring_layout(G_ex_r)
        plt.figure(figsize=(8, 8))
        nx.draw_networkx_edges(G_ex_r, pos_r, alpha=0.3, edge_color="k")
        nx.draw_networkx_nodes(G_ex_r, pos_r, node_color=node_color_list_r)
        nx.draw_networkx_labels(G_ex_r, pos_r, font_size=14)
        plt.axis("off")
        plt.title("The reconstructed graph.")
        plt.show()
        return

    # Check for disconnected subgraphs
    subgraphs = list(nx.connected_components(graph))
    num_subgraphs = len(subgraphs)

    # Layout strategy:
    if num_subgraphs > 1:
        # 1. Extract disconnected subgraphs
        subgraph_list = list(nx.connected_component_subgraphs(graph))

        # 2. Calculate positions for each subgraph separately
        positions = {}
        for i, subgraph in enumerate(subgraph_list):
            # Use spring layout for each subgraph
            subgraph_pos = nx.spring_layout(subgraph, k=5, iterations=200)

            # Calculate a translation vector to spread out subgraphs
            x_offset = i * 3  # Adjust the multiplier (3) for desired spacing
            y_offset = 0

            # Add the subgraph positions to the main positions dictionary, applying the offset
            for node, (x, y) in subgraph_pos.items():
                positions[node] = (x + x_offset, y + y_offset)
        pos = positions

    else:
        pos = nx.spring_layout(graph)

    # Node colors: default is skyblue, terminal are green
    node_colors = ['skyblue'] * len(graph)
    if terminal_vertices: # Only color nodes if terminal_vertices are provided.
        for i, node in enumerate(graph.nodes()):
            if node in terminal_vertices:
                node_colors[i] = 'green'

    # Draw the graph
    nx.draw(graph, pos, with_labels=True, node_size=800, node_color=node_colors, font_size=10)

    # Show the plot
    plt.title("Graph Visualization")
    plt.show()



if __name__ == "__main__":
    # Example Usage
    # Create a directed graph
    G_ex = nx.DiGraph()
    G_ex.add_nodes_from(["In"], node_type="input", node_color="b")
    G_ex.add_nodes_from(["A", "C", "E", "F"], node_type="supported", node_color="g")
    G_ex.add_nodes_from(["B", "D"], node_type="unsupported", node_color="r")
    G_ex.add_nodes_from(["Out"], node_type="output", node_color="m")
    G_ex.add_edges_from(
        [
            ("In", "A"),
            ("A", "B"),
            ("B", "C"),
            ("B", "D"),
            ("D", "E"),
            ("C", "F"),
            ("E", "F"),
            ("F", "Out"),
        ])

    # Plot the original graph.
    node_color_list = [G_ex.nodes[n]["node_color"] for n in G_ex.nodes()]
    pos = nx.spring_layout(G_ex)
    plt.figure(figsize=(8, 8))
    nx.draw_networkx_edges(G_ex, pos, alpha=0.3, edge_color="k")
    nx.draw_networkx_nodes(G_ex, pos, node_color=node_color_list)
    nx.draw_networkx_labels(G_ex, pos, font_size=14)
    plt.axis("off")
    plt.title("The original graph.")
    plt.show()

    # Visualize the graph.  The visualize_graph function will now perform the partitioning and plotting.
    edges = list(G_ex.edges.data(data=False)) # convert to the format expected by visualize_graph
    visualize_graph(edges)
    # Get edge input from the user
    edges = []
    print("Enter edges in the format 'u v weight is_blocked' (e.g., '1 2 10 0').")
    print("Enter 'done' to finish entering edges.")
    while True:
        line = input("> ")
        if line.lower() == 'done':
            break
        try:
            u, v, weight, is_blocked = line.split()
            u = int(u)
            v = int(v)
            weight = float(weight)
            is_blocked = int(is_blocked)  # Convert to integer
            if is_blocked not in (0, 1):
                raise ValueError("is_blocked must be 0 or 1")
            edges.append((u, v, weight, is_blocked))
        except ValueError as e:
            print(f"Invalid input format. Please use 'u v weight is_blocked' with numbers for u, v, and weight, and 0 or 1 for is_blocked.  Error: {e}")

    # Get terminal vertices from the user
    terminal_vertices = []
    print("Enter terminal vertices separated by spaces (e.g., '1 3 5').")
    line = input("> ")
    try:
        terminal_vertices = list(map(int, line.split()))
    except ValueError:
        print("Invalid input format. Please use numbers separated by spaces.")
        terminal_vertices = []
    visualize_graph(edges, terminal_vertices)
