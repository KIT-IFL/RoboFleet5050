from datetime import datetime
import json
from typing import List, Dict, Any, Tuple
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher

class InstantActionsInterface:
    """
    VDA 5050 Instant Actions Interface class.
    """
    def __init__(self, config_data:dict, logging:object, instantActions_topic:str, agentId:str) -> None:
        """
        Initialize the InstantActionsInterface class.

        :param config_data: Data from the configuration file.
        :param logging: Logging object.
        :param instantActions_topic: MQTT topic for the instant actions messages.
        :param agentId: Agent ID.
        """
        self.logging = logging
        self.instantActions_topic = instantActions_topic
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=instantActions_topic,
                                            client_id=f'instantAction_publisher_agent_{self.agentId}', logging=self.logging)

        # The headerId is defined per topic and incremented by 1 with each sent (but not necessarily received) message.
        self.header_id = 1
    
    def generate_instant_actions_message(self, agent:Any, actions:List) -> None:
        """
        Generate the VDA5050 instant action message..
        
        :param agent: Agent object.
        :param actions: List of actions for the agent.
        """
        # Generate the action message.
        actions_msg = []
        for action in actions:
            action_msg = {
                    "actionType": action["actionType"],
                    "actionId": action["actionUUID"],
                    "blockingType": action["blockingType"],
            }
            actions_msg.append(action_msg)

        instantActions_msg = {
            "headerId": self.header_id,
            "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-3] + "Z",
            "version": "V2.1.0",
            "manufacturer": "IFL",
            "serialNumber": f"IFL_{agent.agentId}",
            "actions": actions_msg
        }

        # Publish the instant action message.
        self.mqtt_publisher.publish(instantActions_msg, qos=0)

        # Increment the header ID by 1.
        self.header_id += 1
