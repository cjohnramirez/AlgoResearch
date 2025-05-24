import folium
import networkx as nx

# Create a sample graph
G = nx.Graph()
G.add_node(1, x=0, y=0)
G.add_node(2, x=10, y=10)
G.add_node(3, x=20, y=0)
terminals = {1, 3}

# Create a map centered around node 2
m = folium.Map(location=[10, 10], zoom_start=4)

# Draw nodes
for node, data in G.nodes(data=True):
    color = 'green' if node in terminals else 'gray'
    radius = 8 if node in terminals else 5
    stroke_width = 8 if node in terminals else 0.5  # different stroke widths

    folium.CircleMarker(
        location=(data['y'], data['x']),
        radius=radius,
        color='black',       # Stroke color
        weight=stroke_width, # Stroke width
        stroke=True,
        fill=True,
        fill_color=color,
        fill_opacity=0.7,
        tooltip=f"Node {node}"
    ).add_to(m)

m.save("nodes_map.html")
