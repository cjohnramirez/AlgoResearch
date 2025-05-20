import osmnx as ox
import folium
import networkx as nx
import random
import sys
import os

def extract_network_data(G, terminals, blocked_edges):
    """
    Extract network data in the required format.
    """
    num_vertices = len(G.nodes)
    num_edges = len(G.edges)
    num_terminals = len(terminals)
    
    data_string = f"{num_vertices} {num_edges} {num_terminals}\n"
    data_string += " ".join(map(str, terminals)) + "\n"
    
    # Format edges: v1 v2 weight is_blocked
    for u, v, data in G.edges(data=True):
      is_blocked = 1 if (u, v) in blocked_edges or (v, u) in blocked_edges else 0
      weight = int(round(data.get('length', 1.0)))  # Use length as integer weight, default to 1
      data_string += f"{u} {v} {weight} {is_blocked}\n"
    
    return data_string

def visualize_solution(G, terminals, blocked_edges, solution_file, output_html='solution_network.html'):
    """
    Visualize the road network with terminals, blocked edges, and solution edges highlighted.
    
    Parameters:
    -----------
    G : networkx.Graph
        The road network graph
    terminals : list
        List of terminal node IDs
    blocked_edges : list
        List of blocked edge tuples (u, v)
    solution_file : str
        Path to the solution file containing selected edges
    output_html : str, optional
        Path to save the HTML visualization
    
    Returns:
    --------
    folium.Map or None
        The map object if visualization is successful, None otherwise
    """
    try:
        # Read solution edges from file
        solution_edges = []
        with open(solution_file, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 2:  # Ensure there are at least two values (u, v)
                    # Convert to integers and add as edge
                    u, v = int(parts[0]), int(parts[1])
                    solution_edges.append((u, v))
                    # Also add reverse edge since we're working with an undirected graph
                    solution_edges.append((v, u))
        
        # Get the first node's coordinates for map centering
        first_node = list(G.nodes())[0]
        y = G.nodes[first_node]['y']
        x = G.nodes[first_node]['x']
        
        # Create a folium map
        m = folium.Map(location=[y, x], zoom_start=15, tiles='CartoDB Positron')
        
        # Add all edges to the map
        for u, v, data in G.edges(data=True):
            coords = []
            if 'geometry' in data:
                # If there's geometry data, use it
                coords = [(lat, lon) for lon, lat in data['geometry'].coords]
            else:
                # Otherwise, use node coordinates
                coords = [
                    (G.nodes[u]['y'], G.nodes[u]['x']),
                    (G.nodes[v]['y'], G.nodes[v]['x'])
                ]
            
            # Determine edge color and weight based on its status
            if (u, v) in solution_edges or (v, u) in solution_edges:
                color = 'green'
                weight = 4
                opacity = 0.9
            elif (u, v) in blocked_edges or (v, u) in blocked_edges:
                color = 'red'
                weight = 3
                opacity = 0.7
            else:
                color = 'blue'
                weight = 2
                opacity = 0.5
            
            # Add the edge to the map
            folium.PolyLine(
                coords,
                color=color,
                weight=weight,
                opacity=opacity,
                tooltip=f"Edge: {u}-{v}, Length: {data.get('length', 0):.2f}"
            ).add_to(m)
        
        # Add nodes to the map
        for node, data in G.nodes(data=True):
            # Determine node color based on whether it's a terminal
            color = 'green' if node in terminals else 'gray'
            radius = 8 if node in terminals else 5
            
            # Add the node to the map
            folium.CircleMarker(
                location=(data['y'], data['x']),
                radius=radius,
                color='black',
                fill=True,
                fill_color=color,
                fill_opacity=0.7,
                tooltip=f"Node ID: {node}"
            ).add_to(m)
        
        # Add a legend
        legend_html = '''
        <div style="position: fixed; bottom: 50px; left: 50px; z-index: 1000; background-color: white; padding: 10px; border: 2px solid grey; border-radius: 5px">
            <p><span style="color: green; font-size: 24px;">●</span> Terminal</p>
            <p><span style="color: gray; font-size: 18px;">●</span> Vertex</p>
            <p><span style="color: red; font-size: 18px; font-weight: bold;">—</span> Blocked Edge</p>
            <p><span style="color: blue; font-size: 18px;">—</span> Normal Edge</p>
            <p><span style="color: green; font-size: 18px; font-weight: bold;">—</span> Solution Edge</p>
        </div>
        '''
        m.get_root().html.add_child(folium.Element(legend_html))
        
        # Save the map to an HTML file
        m.save(output_html)
        print(f"Solution visualization saved to {output_html}")
        return m
    except Exception as e:
        print(f"Error during solution visualization: {e}")
        return None

def visualize_network(G, terminals, blocked_edges, output_html='road_network.html'):
    """
    Visualize the road network with terminals and blocked edges.
    """
    try:
        # Get the first node's coordinates for map centering
        first_node = list(G.nodes())[0]
        y = G.nodes[first_node]['y']
        x = G.nodes[first_node]['x']
        
        # Create a folium map
        m = folium.Map(location=[y, x], zoom_start=15,tiles='CartoDB Positron')
        
        # Add all edges to the map
        for u, v, data in G.edges(data=True):
            coords = []
            if 'geometry' in data:
                # If there's geometry data, use it
                coords = [(lat, lon) for lon, lat in data['geometry'].coords]
            else:
                # Otherwise, use node coordinates
                coords = [
                    (G.nodes[u]['y'], G.nodes[u]['x']),
                    (G.nodes[v]['y'], G.nodes[v]['x'])
                ]
            
            # Determine edge color based on whether it's blocked
            color = 'red' if (u, v) in blocked_edges or (v, u) in blocked_edges else 'black'
            weight = 4 if (u, v) in blocked_edges or (v, u) in blocked_edges else 2
            
            # Add the edge to the map
            folium.PolyLine(
                coords,
                color=color,
                weight=weight,
                opacity=0.7,
                tooltip=f"Edge: {u}-{v}, Length: {data.get('length', 0):.2f}"
            ).add_to(m)
        
        # Add nodes to the map
        for node, data in G.nodes(data=True):
            # Determine node color based on whether it's a terminal
            color = 'green' if node in terminals else 'gray'
            radius = 8 if node in terminals else 5
            
            # Add the node to the map
            folium.CircleMarker(
                location=(data['y'], data['x']),
                radius=radius,
                color='black',
                fill=True,
                fill_color=color,
                fill_opacity=0.7,
                tooltip=f"Node ID: {node}"
            ).add_to(m)
        
        # Add a legend
        legend_html = '''
        <div style="position: fixed; bottom: 50px; left: 50px; z-index: 1000; background-color: white; padding: 10px; border: 2px solid grey; border-radius: 5px">
            <p><span style="color: green; font-size: 24px;">●</span> Terminal</p>
            <p><span style="color: gray; font-size: 18px;">●</span> Vertex</p>
            <p><span style="color: red; font-size: 18px; font-weight: bold;">—</span> Blocked Edge</p>
            <p><span style="color: blue; font-size: 18px;">—</span> Normal Edge</p>
        </div>
        '''
        m.get_root().html.add_child(folium.Element(legend_html))
        
        # Save the map to an HTML file
        m.save(output_html)
        print(f"Map visualization saved to {output_html}")
        return m
    except Exception as e:
        print(f"Error during visualization: {e}")
        return None

def select_random_terminals(G, num_terminals):
    """
    Select random nodes as terminals.
    """
    if num_terminals > len(G.nodes):
        print(f"Warning: Requested more terminals ({num_terminals}) than nodes ({len(G.nodes)}). Using all nodes.")
        return list(G.nodes())
    
    return random.sample(list(G.nodes()), num_terminals)

def select_random_blocked_edges(G, num_blocked):
    """
    Select random edges as blocked edges.
    """
    if num_blocked > len(G.edges):
        print(f"Warning: Requested more blocked edges ({num_blocked}) than edges ({len(G.edges)}). Using all edges.")
        return list(G.edges())
    
    return random.sample(list(G.edges()), num_blocked)

def get_integer_input(prompt, default_value, min_value=1):
    """
    Get integer input from the user with validation.
    """
    while True:
        try:
            user_input = input(f"{prompt} [{default_value}]: ").strip()
            if not user_input:
                return default_value
            value = int(user_input)
            if value < min_value:
                print(f"Value must be at least {min_value}. Using default: {default_value}")
                return default_value
            return value
        except ValueError:
            print(f"Please enter a valid integer. Using default: {default_value}")
            return default_value

def get_input(prompt, default_value):
    """
    Get string input from the user.
    """
    user_input = input(f"{prompt} [{default_value}]: ").strip()
    if not user_input:
        return default_value
    return user_input

def get_yes_no_input(prompt, default_value=True):
    """
    Get yes/no input from the user.
    """
    default_str = "Y/n" if default_value else "y/N"
    while True:
        user_input = input(f"{prompt} [{default_str}]: ").strip().lower()
        if not user_input:
            return default_value
        if user_input in ['y', 'yes']:
            return True
        if user_input in ['n', 'no']:
            return False
        print(f"Please enter 'y' or 'n'. Using default: {default_str}")

def limit_graph_size(G, max_vertices, max_edges):
    """
    Limit the graph to a maximum number of vertices and edges.
    """
    if len(G.nodes) <= max_vertices and len(G.edges) <= max_edges:
        return G
    
    # Start with a small section of the graph
    start_node = list(G.nodes())[0]
    subgraph_nodes = {start_node}
    edge_count = 0
    
    # BFS to add nodes until we reach limits
    frontier = [start_node]
    visited = set(frontier)
    
    while frontier and len(subgraph_nodes) < max_vertices and edge_count < max_edges:
        current_node = frontier.pop(0)
        
        for neighbor in G.neighbors(current_node):
            if neighbor not in visited and len(subgraph_nodes) < max_vertices:
                visited.add(neighbor)
                subgraph_nodes.add(neighbor)
                frontier.append(neighbor)
                edge_count += 1
                
                if edge_count >= max_edges:
                    break
    
    # Create a subgraph with the selected nodes
    subgraph = G.subgraph(subgraph_nodes).copy()
    
    print(f"Created subgraph with {len(subgraph.nodes)} vertices and {len(subgraph.edges)} edges")
    return subgraph

def renumber_nodes(G):
    """
    Renumber the nodes with simple sequential integers.
    Creates a new graph with simplified node IDs.
    """
    # Create a mapping from old node IDs to new sequential integers
    mapping = {node: i for i, node in enumerate(G.nodes())}
    
    # Create a new graph with the new node IDs
    H = nx.Graph()
    
    # Add nodes with their attributes
    for old_id, new_id in mapping.items():
        H.add_node(new_id, **G.nodes[old_id])
    
    # Add edges with their attributes
    for u, v, data in G.edges(data=True):
        H.add_edge(mapping[u], mapping[v], **data)
    
    # Return the new graph and the mapping
    return H, mapping

def main():
    try:
        # Get user inputs
        print("Road Network Visualization and Data Extraction")
        print("==============================================")
        
        # Get location
        location = "Cagayan de Oro, PH"
        
        # Get network type
        network_type = "drive"
        
        # Get output filenames
        output_html = f"{get_input("Enter output HTML filename", "road_network")}.html"
        output_data = f"{get_input("Enter output data filename", "network_data")}.txt"
        
        # Download the street network
        print(f"\nDownloading street network for {location}...")
        try:
            G = ox.graph_from_place(location, network_type=network_type)
        except Exception as e:
            print(f"Error downloading network: {e}")
            print("Trying with a simpler approach...")
            try:
                G = ox.graph_from_address(location, network_type=network_type)
            except Exception as e2:
                print(f"Error with second attempt: {e2}")
                print("Using a default location (Manhattan, NY)...")
                G = ox.graph_from_place("Manhattan, NY", network_type=network_type)
        
        # Convert to undirected graph for simplicity
        G_undirected = G.to_undirected()
        
        # Make sure it's a connected graph
        largest_cc = max(nx.connected_components(G_undirected), key=len)
        G_connected = G_undirected.subgraph(largest_cc).copy()
        
        print(f"Network downloaded: {len(G_connected.nodes)} nodes, {len(G_connected.edges)} edges")
        
        # Ask for graph size limits
        print("\nSpecify graph size limits:")
        max_vertices = get_integer_input("Maximum number of vertices", min(100, len(G_connected.nodes)))
        max_edges = get_integer_input("Maximum number of edges", min(200, len(G_connected.edges)))
        
        # Limit graph size if needed
        G_limited = limit_graph_size(G_connected, max_vertices, max_edges)
        
        # Ask if user wants to renumber nodes
        simplify_nodes = get_yes_no_input("\nSimplify node IDs (convert to sequential integers)", True)
        
        if simplify_nodes:
            G_simplified, node_mapping = renumber_nodes(G_limited)
            print("Node IDs simplified to sequential integers")
            # Use the simplified graph for the rest of the process
            G_final = G_simplified
        else:
            G_final = G_limited
        
        # Get terminal and blocked edge counts
        print("\nSpecify terminals and blocked edges:")
        num_terminals = get_integer_input("Number of terminal nodes", 
                                         min(5, len(G_final.nodes)), 
                                         min_value=1)
        num_blocked = get_integer_input("Number of blocked edges", 
                                       min(3, len(G_final.edges)), 
                                       min_value=0)
        
        # Select terminals and blocked edges
        terminals = select_random_terminals(G_final, num_terminals)
        blocked_edges = select_random_blocked_edges(G_final, num_blocked)
        
        print(f"\nSelected {len(terminals)} terminals and {len(blocked_edges)} blocked edges")
        
        # Visualize the network
        print("\nCreating visualization...")
        visualize_network(G_final, terminals, blocked_edges, output_html)
        
        # Extract data in the required format
        data_string = extract_network_data(G_final, terminals, blocked_edges)
        
        # Save the data to a file
        with open(output_data, 'w') as f:
            f.write(data_string)
        
        print(f"Network data saved to {output_data}")

        visualize_sol = get_yes_no_input("\nVisualize a solution file?", False)
        
        if visualize_sol:
            solution_file = get_input("Enter solution file path", "solution.txt")
            solution_html = f"{get_input('Enter solution HTML filename', 'solution_network')}.html"
            
            # Call the solution visualization function
            visualize_solution(G_final, terminals, blocked_edges, solution_file, solution_html)
            
            # Open the solution HTML file if possible
            try:
                print(f"\nAttempting to open solution visualization in browser...")
                html_path = os.path.abspath(solution_html)
                os.system(f'start {html_path}' if sys.platform == 'win32' else 
                         f'open {html_path}' if sys.platform == 'darwin' else 
                         f'xdg-open {html_path}')
            except Exception as e:
                print(f"Could not automatically open the solution visualization: {e}")
                print(f"Please open {os.path.abspath(solution_html)} manually in your browser.")
        
        # [existing code for opening the original visualization...]
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
        
        # Open the HTML file if possible
        try:
            print(f"\nAttempting to open visualization in browser...")
            html_path = os.path.abspath(output_html)
            os.system(f'start {html_path}' if sys.platform == 'win32' else 
                     f'open {html_path}' if sys.platform == 'darwin' else 
                     f'xdg-open {html_path}')
        except Exception as e:
            print(f"Could not automatically open the visualization: {e}")
            print(f"Please open {os.path.abspath(output_html)} manually in your browser.")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()