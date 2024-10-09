
import networkx as nx
import matplotlib.pyplot as plt
import random
import math
from typing import Dict, List, Optional, Tuple

# Local Imports
try:
    from graph_env.benchmark_env_loader import MAPFBenchmark

except ImportError:
    from graph_env.graph_env.benchmark_env_loader import MAPFBenchmark


MIN_WEIGHT = 0
MAX_WEIGHT = 10


def generate_benchmark_layout(
    map_file: str,
    scenario_file: Optional[str] = None
    ) -> (nx.Graph, dict):

    root = "/home/vguillet/ros2_ws/src/graph_env/graph_env/Moving_AI_Lab/street-map/"
    benchmark = MAPFBenchmark(map_file=root+map_file, scenario_file=scenario_file)

    print("> Graph connected:", nx.is_connected(benchmark.graph))

    # > Check graph connectedness
    if not nx.is_connected(benchmark.graph):
        print("!!! Graph is not connected. Reducing to the largest connected component...")
        # Find all connected components
        components = nx.connected_components(benchmark.graph)

        # Get the largest connected component
        largest_component = max(components, key=len)

        # Create a subgraph containing only the largest component
        graph = benchmark.graph.subgraph(largest_component).copy()

        # Generate corresponding pos
        unfiltered_pos = benchmark.pos
        pos = {node: unfiltered_pos[node] for node in graph}

    else:
        graph = benchmark.graph
        pos = benchmark.pos

    # > Check for self-loops
    self_loops = list(nx.selfloop_edges(graph))
    print(f"> Graph contains self-loops: {self_loops is not None}")

    # Draw the graph
    #nx.draw(graph, pos=pos, node_size=0.1)
    #plt.show()

    return graph, pos


def generate_grid_layout(
        num_nodes: int,
        connectivity_percent: float = 0.5
    ) -> (nx.Graph, dict):
    """
    Generate a grid layout of nodes. The grid is generated as a 2D grid graph, and edges are removed randomly while
    ensuring all nodes remain reachable.

    :param num_nodes: The number of nodes in the grid.
    :param connectivity_percent: The percentage of edges to remove while ensuring all nodes remain reachable.

    :return: A tuple containing the graph and the positions of the nodes.
    """
    random.seed(0)  # set random seed for repeatable graph generation

    # -> Calculate number of rows and columns
    rows = int(num_nodes ** 0.5)
    cols = num_nodes // rows

    # -> Create 2D grid graph
    G = nx.grid_2d_graph(rows, cols)

    # -> Assign positions to nodes, where positions are tuples of (row, column)
    pos = dict(zip(G.nodes(), [(i, j) for i in range(rows) for j in range(cols)]))

    # -> Randomly remove edges while ensuring all nodes remain reachable
    edges = list(G.edges())

    edge_removed_count = 0

    random.shuffle(edges)
    for edge in edges:
        # -> Exit is connectivity percent is reached
        if edge_removed_count >= len(edges) - math.floor(len(edges) * connectivity_percent):
            break

        G.remove_edge(*edge)
        if not nx.is_connected(G):
            G.add_edge(*edge)
        else:
            # -> Check if all nodes are still reachable
            for node in G.nodes():
                if not nx.has_path(G, source=(0, 0), target=node):
                    G.add_edge(*edge)
                    break

            else:
                edge_removed_count += 1

        if len(G.edges()) == num_nodes - 1:
            break  # exit loop when all edges have been considered

    # randomly assign weights to edges
    for (u, v) in G.edges():
        # G.edges[u, v]['weight'] = random.randint(MIN_WEIGHT, MAX_WEIGHT)
        G.edges[u, v]['weight'] = random.choice([MIN_WEIGHT, MAX_WEIGHT])
    return G, pos


def generate_random_layout(
        num_nodes: int,
        connectivity_percent: float = 0.5
        ) -> (nx.Graph, dict):
    """
    Generate a random layout of nodes. The graph is generated as a fully connected graph, and edges are removed randomly
    while ensuring all nodes remain reachable.

    :param num_nodes: The number of nodes in the graph.
    :param connectivity_percent: The percentage of edges to remove while ensuring all nodes remain reachable.

    :return: A tuple containing the graph and the positions of the nodes.
    """
    random.seed(0)  # set random seed for repeatable graph generation

    # -> Create a fully connected graph
    G = nx.complete_graph(num_nodes)

    # -> Randomly shuffle the node positions
    pos = {i: (random.uniform(0, 1), random.uniform(0, 1)) for i in range(num_nodes)}

    # -> Assign positions to the nodes
    nx.set_node_attributes(G, pos, "pos")

    # -> Randomly remove edges while ensuring all nodes remain reachable
    edges = list(G.edges())

    edge_removed_count = 0

    random.shuffle(edges)
    for edge in edges:
        # -> Exit is connectivity percent is reached
        if edge_removed_count >= len(edges) - math.floor(len(edges) * connectivity_percent):
            break

        G.remove_edge(*edge)
        if not nx.is_connected(G):
            G.add_edge(*edge)
        else:
            # -> Check if all nodes are still reachable
            for node in G.nodes():
                if not nx.has_path(G, source=(0, 0), target=node):
                    G.add_edge(*edge)
                    break

            else:
                edge_removed_count += 1

        if len(G.edges()) == num_nodes - 1:
            break  # exit loop when all edges have been considered

    # -> Randomly assign weights to edges
    for (u, v) in G.edges():
        G.edges[u, v]['weight'] = random.randint(MIN_WEIGHT, MAX_WEIGHT)

    return G, pos


def generate_star_layout(
        num_nodes: int,
        num_branches: int
        ) -> (nx.Graph, dict):
    """
    Generate a star layout of nodes. The graph is generated as a star graph with a specified number of branches.

    :param num_nodes: The number of nodes in the star.
    :param num_branches: The number of branches in the star.

    :return: A tuple containing the graph and the positions of the nodes.
    """
    random.seed(0)  # set random seed for repeatable graph generation

    G = nx.Graph()

    # Add the center node
    G.add_node(0)

    # Set the position of the center node
    pos = {0: (0.5, 0.5)}

    # Calculate the angle between the branches
    angle = 2 * math.pi / num_branches

    # Set the radius of the star
    radius = 0.4

    # Calculate the number of nodes per branch
    num_nodes_per_branch = int((num_nodes - 1) / num_branches)

    # Loop over the branches
    for i in range(num_branches):

        # Calculate the angle of the current branch
        branch_angle = angle * i

        # Loop over the nodes in the current branch
        for j in range(num_nodes_per_branch):
            # Calculate the ID of the current node
            node_id = i * num_nodes_per_branch + j + 1

            # Calculate the angle of the current node
            # theta = branch_angle + (j/(num_nodes_per_branch-1)) * angle
            theta = branch_angle

            # Calculate the position of the current node
            x = 0.5 + radius * math.cos(theta) * (j + 1)
            y = 0.5 + radius * math.sin(theta) * (j + 1)

            # Add the current node to the graph
            G.add_node(node_id)

            # Set the position of the current node
            pos[node_id] = (x, y)

            # Add an edge between the center node and the current node
            G.add_edge(0, node_id)

    # Return the graph and the positions
    return G, pos


if __name__ == "__main__":
    # Example usage:
    # G, pos = generate_star_layout(num_nodes=28, num_branches=8)
    # G, pos = generate_random_layout(num_nodes=13)
    G, pos = generate_grid_layout(num_nodes=13, connectivity_percent=0.8)
    nx.draw(G, pos=pos)
    plt.show()