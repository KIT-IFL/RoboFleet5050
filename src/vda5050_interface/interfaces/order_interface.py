from datetime import datetime
from typing import List, Dict, Any, Tuple
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher

class OrderInterface:
    """
    VDA5050 order interface class.
    """
    def __init__(self, config_data:dict, logging:object, map_id:str, order_topic:str, agentId:str) -> None:
        """
        Initialize the OrderInterface class.

        :param config_data: Data from the configuration file.
        :param logging: Logging object.
        :param map_id: Map ID for the order messages.
        :param order_topic: MQTT topic for the order messages.
        :param agentId: Agent ID.
        """
        self.logging = logging
        self.map_id = map_id
        self.order_topic = order_topic
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=order_topic,
                                            client_id=f'order_publisher_agent_{self.agentId}', logging=self.logging)


    def generate_order_message(self, agent:Any, orderId:str, order_updateId:int, nodes:List, edges:List) -> None:
        """
        Generate the VDA5050 order message.
        
        :param agent: Agent object.
        :param fleet_management: FleetManagement object.
        :param orderId: Order ID of the VDA5050 order.
        :param order_updateId: Order update ID of the VDA5050 order.
        :param nodes: List of nodes of the VDA5050 order.
        :param edges: List of edges of the VDA5050 order.
        """
        # Generate the node message.
        nodes_msg = []
        for node in nodes:
            node_msg = {
                "nodeId": node['nodeId'],
                "sequenceId": node["sequenceId"],
                "released": node["released"],
                "nodePosition": {
                    "x": node["x"],
                    "y": node["y"],
                    "mapId": self.map_id
                },
                "actions": node["actions"]
            }
            nodes_msg.append(node_msg)

        # Generate the edge message.
        edges_msg = []
        for edge in edges:
            edge_msg = {
                "edgeId": str(edge["edgeId"]),
                "sequenceId": edge["sequenceId"],
                "released": edge["released"],
                "startNodeId": str(edge["startNodeId"]),
                "endNodeId": str(edge["endNodeId"]),
                "actions": []
            }
            edges_msg.append(edge_msg)

        with agent.agents_object.order_header_id_lock:
            # Generate the order message.
            order_msg = {
                "headerId": agent.agents_object.order_header_id,
                "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-3] + "Z",
                "version": "V2.1.0",
                "manufacturer": "IFL",
                "serialNumber": f"IFL_{agent.agentId}",
                "orderId": orderId,
                "orderUpdateId": order_updateId,
                "nodes": nodes_msg,
                "edges": edges_msg
            }

            # Publish the order message to the MQTT broker.
            self.mqtt_publisher.publish(order_msg, qos=0)

            # Increment the header ID by 1.
            agent.agents_object.order_header_id += 1

        # Save the order message as a JSON file.
        # with open(f"data/output_files/order_msg.json", "w") as order_file:
        #     json.dump(order_msg, order_file, indent=4)
