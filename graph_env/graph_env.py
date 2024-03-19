
##################################################################################################################

# Built-in/Generic Imports
from json import dumps, loads

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

        # -> Publish the environment
        # self.publish_env()

        # -> Display the graph
        nx.draw(self.graph, pos=self.pos)
        # plt.show()

    def env_callback(self, msg: TeamCommStamped):
        if msg.meta_action == "environment update":
            self.graph, self.pos = self.json_to_graph(msg.memo)

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
        msg.memo = self.graph_to_json()

        # Publish the message
        self.env_publisher.publish(msg)

    def graph_to_json(self):
        """
        Convert the graph and the positions to a JSON string.

        :return: The JSON string representation of the graph.
        """

        data = {
            "env_type": "graph",
            "graph": nx.node_link_data(self.graph),
            "pos": {str(k): v for k, v in self.pos.items()}
        }

        return dumps(data)

    def json_to_graph(self, graph_json: str) -> (Graph, dict):
        """
        Convert a JSON string to a graph.

        :param graph_json: The JSON string representation of the graph.

        :return: A graph object.
        :return: A dictionary containing the positions of the nodes.
        """
        data = loads(graph_json)

        graph = nx.node_link_graph(data["graph"])
        pos = {eval(k): v for k, v in data["pos"].items()}

        return graph, pos


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = graph_env()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()
