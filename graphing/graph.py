import networkx as nx
import matplotlib.pyplot as plt

def parse_graph_file(file_path):
    with open(file_path, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    G = nx.Graph()
    terminals = []
    steiner_exact_edges = []
    steiner_approx_edges = []

    mode = None
    for line in lines:
        if line.startswith("Vertices"):
            mode = "vertices"
            continue
        elif line.startswith("Edges (u v weight is_blocked)"):
            mode = "edges"
            continue
        elif line.startswith("Terminals Count"):
            mode = "terminals_count"
            continue
        elif line.startswith("Terminals"):
            mode = "terminals"
            continue
        elif line.startswith("Edges in Steiner Tree: Exact"):
            mode = "steiner_exact"
            continue
        elif line.startswith("Edges in Steiner Tree: Approx"):
            mode = "steiner_approx"
            continue

        if mode == "vertices":
            num_vertices, num_edges = map(int, line.split())
            for i in range(num_vertices):
                G.add_node(i)
        elif mode == "edges":
            u, v, w, blocked = map(int, line.split())
            G.add_edge(u, v, weight=w, blocked=bool(blocked))
        elif mode == "terminals_count":
            # You can store this if needed
            continue
        elif mode == "terminals":
            terminals.extend(map(int, line.split()))
        elif mode == "steiner_exact":
            u, v, w, blocked = map(int, line.split())
            steiner_exact_edges.append((u, v))
        elif mode == "steiner_approx":
            u, v, w, blocked = map(int, line.split())
            steiner_approx_edges.append((u, v))

    return G, terminals, steiner_exact_edges, steiner_approx_edges

def draw_graph(G, terminals, steiner_edges, title):
    pos = nx.spring_layout(G, seed=42, k=0.5)  
    plt.figure(figsize=(8, 6))

    # Draw all edges
    blocked_edges = [(u, v) for u, v in G.edges if G[u][v]['blocked']]
    normal_edges = [(u, v) for u, v in G.edges if not G[u][v]['blocked']]

    nx.draw_networkx_edges(G, pos, edgelist=normal_edges, width=1, edge_color='black', alpha=0.2)
    nx.draw_networkx_edges(G, pos, edgelist=blocked_edges, width=1, edge_color='gray', style='dashed')
    nx.draw_networkx_edges(G, pos, edgelist=steiner_edges, width=1, edge_color='black')

    # Draw nodes
    nx.draw_networkx_nodes(G, pos, node_size=500, node_color='gray')
    nx.draw_networkx_nodes(G, pos, nodelist=terminals, node_size=600, node_color='black')

    # Draw labels
    nx.draw_networkx_labels(G, pos, font_size=10, font_color='white')
    


    plt.axis('off')
    plt.title(title)
    plt.show()

if __name__ == "__main__":
    graph_file = r"c:\Users\jcram\Downloads\Coding\AlgoResearch\graphing\graph.txt"
    G, terminals, steiner_exact_edges, steiner_approx_edges = parse_graph_file(graph_file)
    draw_graph(G, terminals, steiner_exact_edges, "Exact Steiner Tree")
    draw_graph(G, terminals, steiner_approx_edges, "Approx Steiner Tree")
