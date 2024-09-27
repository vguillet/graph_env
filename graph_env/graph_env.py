
##################################################################################################################

# Built-in/Generic Imports
from json import dumps, loads
from pprint import pprint
import sys
import os

import networkx as nx
# Libs
from networkx import Graph
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Local Imports
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from .graph_generator import *
from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

try:
    from orchestra_config.sim_config import *
    from rlb_simple_sim.Scenario import Scenario
    from maaf_tools.tools import *

except ModuleNotFoundError:
    from orchestra_config.orchestra_config.sim_config import *
    from rlb_simple_sim.rlb_simple_sim.Scenario import Scenario
    from maaf_tools.maaf_tools.tools import *

##################################################################################################################


class graph_env(Node):
    def __init__(self):
        # Initialize the Node
        super().__init__("graph_env")

        # -> Declare launch parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("scenario_id", "Scenario_0_full_intercession_no_recompute_0_interventionism_0.json"),
            ]
        )

        # -> Get launch parameters configuration
        scenario_id = self.get_parameter("scenario_id").get_parameter_value().string_value

        self.scenario = Scenario(scenario_id=scenario_id, load_only=True, logger=self.get_logger())

        # -> Get the launch parameters
        self.graph_type = self.scenario.environment_type
        self.num_nodes = self.scenario.env_size * self.scenario.env_size
        self.connectivity_percent = self.scenario.env_connectivity
        self.num_branches = 0    # TODO: Cleanup
        self.graph_seed = self.scenario.seed

        # -> Generate the graph
        if self.graph_type == "grid":
            self.graph, self.pos = generate_grid_layout(
                num_nodes=self.num_nodes,
                connectivity_percent=self.connectivity_percent
            )

        elif self.graph_type == "star":
            self.graph, self.pos = generate_star_layout(
                num_nodes=self.num_nodes,
                num_branches=self.num_branches
            )

        elif self.graph_type == "random":
            self.graph, self.pos = generate_random_layout(
                num_nodes=self.num_nodes
            )

        elif self.graph_type == "MAPF":
            self.graph, self.pos = generate_benchmark_layout(
                map_file=self.scenario.environment_path
            )

        # ----- Compute shortest paths from all to tasks
        self.compute_shortest_paths()

        # ----- Create the publisher
        self.env_publisher = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=topic_environment,
            qos_profile=qos_env
        )

        # ----- Create the timer
        self.timer = self.create_timer(
            timer_period_sec=1,
            callback=self.publish_env
        )

        # -> Publish the environment
        # self.publish_env()

        # -> Display the graph
        nx.draw(self.graph, pos=self.pos)
        # plt.show()

    def compute_shortest_paths(self):
        # -> Display the graph
        # nx.draw(self.graph, pos=self.pos)
        # plt.show()

        # -> Retrieve path from scenario ID
        shortest_paths_path = f"/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Configs/{self.scenario.scenario_id.split('_')[1]}_shortest_paths.json"

        # -> If path exists, parse it
        if os.path.exists(shortest_paths_path):
            print(f"Loading shortest paths from file: {shortest_paths_path}")

            with open(shortest_paths_path, "r") as f:
                all_pairs_shortest_paths = loads(f.read())
                self.all_pairs_shortest_paths = {eval(k): {eval(k2): v2 for k2, v2 in v.items()} for k, v in all_pairs_shortest_paths.items()}

                self.get_logger().info(f"         > {self}: Loaded shortest paths from file")

        else:
            self.get_logger().info(f"         > {self}: Computing all shortest paths...")

            # -> List all task nodes from scenario
            task_node_locs = [(goto_task["instructions"]["x"], goto_task["instructions"]["y"]) for goto_task in self.scenario.goto_tasks]

            # -> Compute all shortest paths from all nodes to all task nodes
            matching_nodes = [node for node, position in self.pos.items() if position in task_node_locs]

            print(f"Matching: {len(matching_nodes)}/{len(self.scenario.goto_tasks)}:", matching_nodes)
            print("Computing all shortest paths from all nodes to all task nodes...")
            self.all_pairs_shortest_paths = {}

            for i, task_node_loc in enumerate(task_node_locs):
                print(f"> Computing shortest paths - {i+1}/{len(task_node_locs)}")

                # > Find node corresponding to task
                task_node = [node for node, position in self.pos.items() if position == task_node_loc][0]

                # > Find paths from node to all other nodes
                paths = dict(nx.single_source_shortest_path(G=self.graph, source=task_node))

                # > Add paths from each node to the task node
                for source_node, path in paths.items():
                    # > Invert path
                    path.reverse()

                    # > Record path from source to task node
                    if source_node not in self.all_pairs_shortest_paths.keys():
                        self.all_pairs_shortest_paths[str(source_node)] = {}

                    self.all_pairs_shortest_paths[str(source_node)][str(task_node)] = path

            self.get_logger().info(f"         > {self}: Done computing all shortest paths")

            # -> Cache results to file
            with open(shortest_paths_path, "w") as f:
                all_pairs_shortest_paths = {str(k): {str(k2): v2 for k2, v2 in v.items()} for k, v in self.all_pairs_shortest_paths.items()}
                f.write(dumps(all_pairs_shortest_paths))

                print(f"Saved shortest paths to file: {self.scenario.scenario_id}")

    def env_callback(self, msg: TeamCommStamped):
        if msg.meta_action == "environment update":
            environment = json_to_graph(graph_json=msg.memo)

            self.graph = environment["graph"]
            self.pos = environment["pos"]

            # -> Display the graph with weights on edges
            nx.draw(self.graph, pos=self.pos, with_labels=True, node_size=500, node_color="skyblue", font_size=8)
            nx.draw_networkx_edge_labels(self.graph, pos=self.pos, edge_labels={(u, v): d["weight"] for u, v, d in self.graph.edges(data=True)})

            plt.show()

    def publish_env(self):
        self.get_logger().info("Publishing environment")
        # Create the message
        msg = TeamCommStamped()
        msg.stamp = self.get_clock().now().to_msg()

        msg.source = "graph_env"
        msg.target = "all"

        msg.meta_action = "environment update"
        # environment_dict = graph_to_json(graph=self.graph, pos=self.pos, shortest_paths=self.all_pairs_shortest_paths)
        environment_dict = graph_to_json(graph=self.graph, pos=self.pos, shortest_paths=None)

        msg.memo = dumps(environment_dict)

        # Publish the message
        self.env_publisher.publish(msg)


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = graph_env()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()
