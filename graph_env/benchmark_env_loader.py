from collections import defaultdict
from itertools import cycle
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import networkx as nx  # Import the NetworkX library for graph-based operations
import numpy as np  # Import NumPy for matrix operations
from random import shuffle  # Import shuffle to randomize lists


# Class to benchmark Multi-Agent Path Finding (MAPF)
class MAPFBenchmark:
    def __init__(self, map_file: str, scenario_file: str) -> None:
        # Initialize class variables
        self.__map_type = 4  # Indicates the connectivity type (4 or 8-connected)
        self.__graph = nx.Graph()  # Initialize an empty graph

        # Initialize default data structures for scenarios
        self.__map = np.array([0])  # Placeholder for the map matrix
        self.__scenario_start: Dict[int, List[Tuple[int, int]]] = defaultdict(list)  # Start positions for each scenario
        self.__scenario_target: Dict[int, List[Tuple[int, int]]] = defaultdict(list)  # Target positions for each scenario
        self.__scenario_cost: Dict[int, List[float]] = defaultdict(list)  # Cost for each scenario

        # Parse the map and scenario files
        self.__parse_map(map_file)
        #self.__parse_scenario(scenario_file)

    # Property to access the graph
    @property
    def graph(self) -> nx.Graph:
        return self.__graph

    @property
    def pos(self) -> dict:
        return dict(self.__graph.nodes.data("pos"))

    # Property to access the map matrix
    @property
    def matrix(self) -> np.array:
        return self.__map

    # Private method to parse the map file and construct the graph
    def __parse_map(self, map_file: str) -> bool:
        f = open(map_file)

        # Parse the first line, which should define the map type
        l = f.readline()
        if l[:4] != "type":
            return False

        # Read map dimensions
        l = f.readline()  # Read height
        if l[:6] != "height":
            return False
        map_height = int(l[7:])

        l = f.readline()  # Read width
        if l[:5] != "width":
            return False
        map_width = int(l[6:])

        # Initialize a matrix to represent the map
        self.__map = np.zeros((map_height, map_width), dtype=int)

        # Skip to the map content
        l = f.readline()
        if l[:3] == "map":
            l = f.readline()

        # Parse the map and construct the graph nodes
        for i in range(map_height):
            for j in range(map_width):
                if l[j] in "TO@":  # "T", "O", "@" represent obstacles
                    self.__map[i, j] = 0  # Mark obstacle
                else:
                    self.__map[i, j] = 1  # Free space
                    self.__graph.add_node((i, j), pos=(j, map_height - i))  # Add graph node for free space
            l = f.readline()

        f.close()

        # Add edges between neighboring nodes in the graph
        for i in range(map_height):
            for j in range(map_width):
                if (i, j) in self.__graph.nodes:
                    for c in self.__neighbors(i, j):
                        self.__graph.add_edge((i, j), c, weight=1)

        return True

    # Private method to find neighbors of a node based on connectivity type
    def __neighbors(self, i: int, j: int):
        # Check neighbors in 4-connected directions (up, down, left, right)
        if i > 0 and self.__map[i - 1, j] == 1:
            yield (i - 1, j)
        if i < self.__map.shape[0] - 1 and self.__map[i + 1, j] == 1:
            yield (i + 1, j)
        if j > 0 and self.__map[i, j - 1] == 1:
            yield (i, j - 1)
        if j < self.__map.shape[1] - 1 and self.__map[i, j + 1] == 1:
            yield (i, j + 1)

        # If map is 8-connected, also check diagonals
        if self.__map_type == 8:
            # Diagonal checks with additional constraints
            if i > 0 and j > 0 and self.__map[i - 1, j - 1] == 1:
                yield (i - 1, j - 1)
            if i < self.__map.shape[0] - 1 and j < self.__map.shape[1] - 1 and self.__map[i + 1, j + 1] == 1:
                yield (i + 1, j + 1)
            if i < self.__map.shape[0] - 1 and j > 0 and self.__map[i + 1, j - 1] == 1:
                yield (i + 1, j - 1)
            if i > 0 and j < self.__map.shape[1] - 1 and self.__map[i - 1, j + 1] == 1:
                yield (i - 1, j + 1)

    # String representation of the object
    def __str__(self) -> str:
        return f"""
            map size: {self.__map.shape}
            scenarios: {self.number_of_scenario}
        """

    # Method to draw the graph and scenarios or solutions
    def draw(
            self,
            scenario: Optional[int] = None,
            solution: Optional[List[List[Tuple[int, int]]]] = None,
    ):
        # Draw the graph
        nx.draw(self.graph, pos=self.pos)
        plt.show()

        nx.draw_networkx(
            self.graph,
            pos=self.pos,
            with_labels=False,
            node_size=1,
            edge_color="grey",
            node_color="grey",
            node_shape=",",
        )

        # If a scenario is specified, draw start and target positions
        if scenario is not None:
            starts = self.__scenario_start[scenario]
            targets = self.__scenario_target[scenario]
            colors = cycle("bgrcmyk")  # Cycle through different colors for each agent
            color = [next(colors) for _ in range(len(starts))]

            nx.draw_networkx(
                self.__graph,
                nodelist=starts,
                pos=dict(self.__graph.nodes.data("pos")),
                with_labels=False,
                node_size=3,
                node_color=color,
                node_shape="^",
                edge_color="grey",
            )
            nx.draw_networkx(
                self.__graph,
                nodelist=targets,
                pos=dict(self.__graph.nodes.data("pos")),
                with_labels=False,
                node_size=3,
                node_color=color,
                node_shape="s",
                edge_color="grey",
            )

        # If a solution is provided, draw the paths
        elif solution is not None:
            colors = cycle("bgrcmyk")
            for plan in solution:
                color = next(colors)
                edges = list(zip(plan[:-1], plan[1:]))  # Create edges from the solution path
                nx.draw_networkx(
                    self.__graph,
                    nodelist=plan,
                    edgelist=edges,
                    pos=dict(self.__graph.nodes.data("pos")),
                    with_labels=False,
                    node_size=3,
                    node_color=color,
                    edge_color=color,
                    width=3,
                )

    # Private method to parse the scenario file
    def __parse_scenario(self, scenario_file: str) -> bool:
        f = open(scenario_file)
        l = f.readline()  # Read version line

        l = f.readline()  # Start reading the scenario data
        while l:
            t = l.split("\t")
            scen = int(t[0])  # Scenario number
            self.__scenario_start[scen].append((int(t[5]), int(t[4])))  # Start position
            self.__scenario_target[scen].append((int(t[7]), int(t[6])))  # Target position
            self.__scenario_cost[scen].append(float(t[8]))  # Cost
            l = f.readline()

        return True

    # Property to get the number of scenarios
    @property
    def number_of_scenario(self) -> int:
        return len(self.__scenario_start)

    # Method to get the start and target positions of a scenario
    def scenario(self, i: int) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
        return self.__scenario_start[i], self.__scenario_target[i]

    # Method to shuffle target positions for scenarios while ensuring uniqueness
    def scenario_from(self, scenarios: list):
        starts, targets = [], []
        for num_s in scenarios:
            start, target = self.scenario(num_s)
            starts += start
            targets += target
        check_all_different = starts + targets
        all_diferent = set(check_all_different)
        if len(set(all_diferent)) == len(check_all_different):
            shuffle(targets)  # Shuffle target positions if valid
            return starts, targets
        else:
            return None, None  # Return None if positions are not unique

    # Method to get the cost of a scenario
    def scenario_costs(self, i: int) -> List[float]:
        return self.__scenario_cost[i]


if __name__ == "__main__":
    root = "/home/vguillet/ros2_ws/src/environment_node/environment_node/MAF_Environments/Graph/Moving_AI_Lab/street-map/"
    benchmark = MAPFBenchmark(root+"Paris_0_256.map", "data/scen_1.txt")
    print(benchmark)
    benchmark.draw(1)
    # starts, targets = benchmark.scenario_from([1, 2, 3])
    # print(starts, targets)
    # benchmark.draw(solution=[starts, targets])