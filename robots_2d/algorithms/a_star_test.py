import itertools

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

from robots_2d.algorithms.a_star import a_star

if __name__ == "__main__":

    n_nodes = 25    # The number of nodes
    k = 3           # The number of connected neareset neighbors

    # Sample the positions of the nodes
    points = np.random.rand(n_nodes, 2) * 10

    # Compute the distance matrix
    diff = points[:, np.newaxis, :] - points[np.newaxis, :, :]
    distance_matrix = np.linalg.norm(diff, axis=2)

    # Construct the graph
    graph = nx.Graph()

    for node in range(n_nodes):

        # The closes neighbor is always itself
        neighbors = np.argsort(distance_matrix[node])[1:k+1]

        graph.add_node(int(node))

        for neighbor in neighbors:

            graph.add_edge(
                node,
                int(neighbor),
                weight = distance_matrix[node, neighbor]
            )

    # Make sure the graph is fully connected
    while not nx.is_connected(graph):
        components = list(nx.connected_components(graph))

        # Pick two nodes from two different components
        a = next(iter(components[0]))
        b = next(iter(components[1]))

        graph.add_edge(
            a,
            b,
            weight = distance_matrix[a, b],
        )

    # Call A-Star
    start, end = np.unravel_index(
        np.argmax(distance_matrix),
        distance_matrix.shape
    )

    shortest_path = a_star(
        graph,
        int(start),
        int(end),
        lambda node_a, node_b: distance_matrix[node_a, node_b]
    )

    # Check for correctness with networkx's implementation
    nx_shortest_path = nx.astar_path(
        graph,
        int(start),
        int(end),
        heuristic = lambda node_a, node_b: distance_matrix[node_a, node_b],
        weight = "weight",
    )

    if not shortest_path == nx_shortest_path:
        raise ValueError(
            f"The found shortest path don't match: {shortest_path} != {nx_shortest_path}!"
        )

    # Visualize the graph
    nx.draw(
        graph,
        pos = points,
        with_labels = True,
        edge_color = "black",
        width = 2,
    )

    # Extract the edges belonging to the shortest path
    path_edges = list(itertools.pairwise(shortest_path))

    # Draw the shortest path
    nx.draw_networkx_edges(
        graph,
        pos = points,
        edgelist = path_edges,
        edge_color = "green",
        width = 3,
    )

    nx.draw_networkx_nodes(
        graph,
        pos = points,
        nodelist = [start],
        node_color = "green",
        node_size = 300,
    )

    nx.draw_networkx_nodes(
        graph,
        pos = points,
        nodelist = [end],
        node_color = "red",
        node_size = 300,
    )

    plt.show()