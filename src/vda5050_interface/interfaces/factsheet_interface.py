from datetime import datetime
import json
from typing import List, Dict, Any, Tuple
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher

class FactsheetInterface:
    """
    VDA 5050 Factsheet Interface class.
    """
    def __init__(self, config_data:dict, logging:object, factsheet_topic:str, agentId:str) -> None:
        """
        Initialize the FactsheetInterface class.

        :param config_data: Data from the configuration file.
        :param logging: Logging object.
        :param factsheet_topic: MQTT topic for the factsheet messages.
        :param agentId: Agent ID.
        """
        self.logging = logging
        self.factsheet_topic = factsheet_topic
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=factsheet_topic,
                                            client_id=f'factsheet_publisher_agent_{self.agentId}', logging=self.logging)

        # The headerId is defined per topic and incremented by 1 with each sent (but not necessarily received) message.
        self.header_id = 1

    def generate_factsheet_message(self, agent:Any, factsheet:Dict) -> None:
        """
        Generate the VDA5050 factsheet message.
        
        :param agent: Agent object.
        :param factsheet: Factsheet of the agent.
        """
        factsheet_msg = {
            "headerId": self.header_id,
            "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-3] + "Z",
            "version": "V2.1.0",
            "manufacturer": "IFL",
            "serialNumber": f"IFL_{agent.agentId}",
            "factsheet": factsheet
        }

        # Publish the factsheet message.
        self.mqtt_publisher.publish(factsheet_msg, qos=0)

        # Increment the header ID by 1.
        self.header_id += 1
