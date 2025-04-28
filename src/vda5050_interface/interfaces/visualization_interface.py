from datetime import datetime
import json
from typing import List, Dict, Any, Tuple
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher

class VisualizationInterface:
    """
    VDA5050 visualization interface class.
    """
    def __init__(self, config_data:dict, logging:object, visualization_topic:str, agentId:str) -> None:
        """
        Initialize the VisualizationInterface class.
        
        :param config_data: Data from the configuration file.
        :param logging: Logging object.
        :param visualization_topic: MQTT topic for the visualization messages.
        :param agentId: Agent ID.
        """
        self.logging = logging
        self.visualization_topic = visualization_topic
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=visualization_topic,
                                            client_id=f'visulaization_publisher_agent_{self.agentId}', logging=self.logging)

        # The headerId is defined per topic and incremented by 1 with each sent (but not necessarily received) message.
        self.header_id = 1

    def generate_visualization_message(self, agent:Any, agvPosition:dict) -> None:
        """
        Generate the VDA5050 visualization message.
        
        :param agent: Agent object.
        :param agvPosition: AGV position.
        """
        visualization_msg = {
            "headerId": self.header_id,
            "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-3] + "Z",
            "version": "V2.1.0",
            "manufacturer": "IFL",
            "serialNumber": agent.agentId,
            "agvPosition": agvPosition
        }

        # Publish the visualization message.
        self.mqtt_publisher.publish(visualization_msg, qos=0)

        # Increment the header ID by 1.
        self.header_id += 1