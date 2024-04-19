
##################################################################################################################

# Built-in/Generic Imports
from json import dumps, loads
from pprint import pprint

# Libs
from networkx import Graph
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Local Imports
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from .graph_generator import *
from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
from orchestra_config.sim_config import *
from rlb_simple_sim.Scenario import Scenario
from maaf_tools.tools import *

##################################################################################################################


class graph_env(Node):
    def __init__(self):
        # Initialize the Node
        super().__init__("graph_env")

        # -> Declare launch parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("scenario_id", "simple_sim"),
            ]
        )

        # -> Get launch parameters configuration
        scenario_id = self.get_parameter("scenario_id").get_parameter_value().string_value

        self.scenario = Scenario(scenario_id=scenario_id, load_only=True, logger=self.get_logger())

        # -> Get the launch parameters
        self.graph_type = "grid"    # TODO: Cleanup
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

        # -> Compute all shortest paths between all node pairs
        # self.all_pairs_shortest_paths = dict(nx.all_pairs_shortest_path(self.graph))

        # pprint(self.all_pairs_shortest_paths)

        # -> Publish the environment
        # self.publish_env()

        # -> Display the graph
        nx.draw(self.graph, pos=self.pos)
        # plt.show()

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
        msg.memo = dumps(graph_to_json(graph=self.graph, pos=self.pos))

        # Publish the message
        self.env_publisher.publish(msg)


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = graph_env()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()
