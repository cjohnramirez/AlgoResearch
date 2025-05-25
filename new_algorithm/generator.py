import random

def generate_steiner_graph():
    """
    Generates an undirected, weighted graph for the Steiner tree problem
    based on user-provided parameters.
    """
    while True:
        try:
            num_vertices = int(input("Enter the number of vertices (e.g., 35): "))
            if num_vertices <= 1:
                print("Number of vertices must be at least 2.")
                continue
            break
        except ValueError:
            print("Invalid input. Please enter an integer.")

    while True:
        try:
            num_edges = int(input(f"Enter the number of edges (minimum {num_vertices - 1} for connectivity): "))
            if num_edges < num_vertices - 1:
                print(f"Number of edges must be at least {num_vertices - 1} to ensure connectivity.")
                continue
            break
        except ValueError:
            print("Invalid input. Please enter an integer.")

    while True:
        try:
            num_terminals = int(input(f"Enter the number of terminals (up to {num_vertices}): "))
            if num_terminals <= 0 or num_terminals > num_vertices:
                print(f"Number of terminals must be between 1 and {num_vertices}.")
                continue
            break
        except ValueError:
            print("Invalid input. Please enter an integer.")

    while True:
        try:
            num_blocked_edges = int(input(f"Enter the number of blocked edges (up to {num_edges}): "))
            if num_blocked_edges < 0 or num_blocked_edges > num_edges:
                print(f"Number of blocked edges must be between 0 and {num_edges}.")
                continue
            break
        except ValueError:
            print("Invalid input. Please enter an integer.")

    print("\n--- Generating Graph ---")

    # Generate Terminals
    terminals = random.sample(range(num_vertices), num_terminals)
    terminals.sort() # Sort for consistent output

    # Initialize Graph and Edges Set
    edges = set() # To store (u, v) tuples to avoid duplicates
    graph_edges = [] # List to store (u, v, weight, is_blocked)

    # Ensure Connectivity (using a modified spanning tree approach)
    connected_vertices = set()
    available_vertices = set(range(num_vertices))
    start_node = random.randint(0, num_vertices - 1)
    connected_vertices.add(start_node)
    available_vertices.remove(start_node)

    # First, add edges to guarantee connectivity (forming a spanning tree)
    while len(connected_vertices) < num_vertices and len(graph_edges) < num_edges:
        u = random.choice(list(connected_vertices))
        # Try to connect to an unconnected vertex first
        possible_v = list(available_vertices - connected_vertices)
        if possible_v:
            v = random.choice(possible_v)
        else:
            # If all are connected, then pick any other vertex that doesn't have an edge yet
            v = random.choice(list(set(range(num_vertices)) - {u} - {node for edge_tuple in edges for node in edge_tuple if u in edge_tuple and node != u}))


        if u != v and (u, v) not in edges and (v, u) not in edges:
            weight = random.randint(1, 100)
            graph_edges.append((u, v, weight, 0)) # 0 for not blocked initially
            edges.add(tuple(sorted((u, v))))
            if v in available_vertices: # Only move if it was truly an 'available' (unconnected) vertex
                connected_vertices.add(v)
                available_vertices.discard(v) # Use discard just in case v was already connected via another path
        # If the randomly picked u and v didn't form a new edge, try again in the loop

    # Add Remaining Edges to meet num_edges constraint and increase density
    while len(graph_edges) < num_edges:
        u = random.randint(0, num_vertices - 1)
        v = random.randint(0, num_vertices - 1)

        if u != v and (u, v) not in edges and (v, u) not in edges:
            weight = random.randint(1, 100)
            graph_edges.append((u, v, weight, 0))
            edges.add(tuple(sorted((u, v))))

    # Block Edges
    if num_blocked_edges > 0 and len(graph_edges) > 0:
        # Create a list of indices to choose from, avoiding duplicates
        eligible_indices = list(range(len(graph_edges)))
        if len(eligible_indices) >= num_blocked_edges:
            edge_indices_to_block = random.sample(eligible_indices, num_blocked_edges)
            for index in edge_indices_to_block:
                u, v, weight, _ = graph_edges[index]
                graph_edges[index] = (u, v, weight, 1) # Mark as blocked
        else:
            print(f"Warning: Not enough unique edges ({len(graph_edges)}) to block {num_blocked_edges}. Blocking all available edges.")
            for i in range(len(graph_edges)):
                u, v, weight, _ = graph_edges[i]
                graph_edges[i] = (u, v, weight, 1)

    # Format Output
    print(f"\n{num_vertices} {num_edges} {num_terminals}")
    print(" ".join(map(str, terminals)))
    for u, v, weight, is_blocked in graph_edges:
        print(f"{u} {v} {weight} {is_blocked}")

# Run the graph generation function
generate_steiner_graph()