import heapq

import numpy as np


def a_star(graph, start, goal, heuristic):

    # The list of open nodes and their estimated total costs
    open = [(heuristic(start, goal), start)]

    # A dictionary telling whether a node is closed
    closed = set()

    # The parent node for the shortes path
    parent = {
        node: None for node in graph.nodes
    }

    # The costs from the start to the corresponding node
    past_cost = {
        node: np.inf for node in graph.nodes
    }
    past_cost[start] = 0

    # Start the main loop
    while open:

        # Get the node from the open list with the lowest estimated cost
        _, current = heapq.heappop(open)

        # Ignore nodes that have already been processed
        if current in closed:
            continue

        # Append the current node to the closed list
        closed.add(current)

        # Check whether the goal has been reached
        if current == goal:

            # Reconstruct path using parents
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()

            # Return indices of shortest path
            return path

        # Iterate over each neighbor
        for neighbor in graph.neighbors(current):

            # Only consider neighbors that are not closed
            if neighbor in closed:
                continue

            # Compute cost to neighbor via current
            tentative_past_cost = (
                past_cost[current]
                + graph[current][neighbor]["weight"]
            )

            # Check whether this path is an improvement
            if tentative_past_cost < past_cost[neighbor]:

                # Update cost
                past_cost[neighbor] = tentative_past_cost

                # Update parent information
                parent[neighbor] = current

                # Compute new estimate for neighbor
                est_total_cost = past_cost[neighbor] + heuristic(neighbor, goal)

                # Insert new estimate in open list
                heapq.heappush(
                    open,
                    (est_total_cost, neighbor)
                )

    raise ValueError(
        f"Unable to find path between {start} and {goal}!"
    )