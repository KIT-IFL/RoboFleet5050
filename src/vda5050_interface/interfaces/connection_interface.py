from datetime import datetime
import json
from typing import List, Dict, Any, Tuple
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher

class ConnectionInterface:
    """
    VDA 5050 Connection Interface class.
    """
    def __init__(self, config_data:dict, logging:object, connection_topic:str, agentId:str) -> None:
        """
        Initialize the ConnectionInterface class.

        :param config_data: Data from the configuration file.
        :param logging: Logging object.
        :param connection_topic: MQTT topic for the connection messages.
        :param agentId: Agent ID.
        """
        self.logging = logging
        self.connection_topic = connection_topic
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=connection_topic,
                                            client_id=f'connection_publisher_agent_{self.agentId}', logging=self.logging)

        # The headerId is defined per topic and incremented by 1 with each sent (but not necessarily received) message.
        self.header_id = 1

    def generate_connection_message(self, agent:Any, connectionState:str) -> None:
        """
        Generate the VDA5050 connection message.
        
        :param agent: Agent object.
        :param connectionState: Connection state of the agent.
        """
        connection_msg = {
            "headerId": self.header_id,
            "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-3] + "Z",
            "version": "V2.1.0",
            "manufacturer": "IFL",
            "serialNumber": f"IFL_{agent.agentId}",
            "connected": connectionState
        }

        # Publish the connection message.
        self.mqtt_publisher.publish(connection_msg, qos=0)

        # Increment the header ID by 1.
        self.header_id += 1
