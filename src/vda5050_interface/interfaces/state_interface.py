from datetime import datetime
from typing import List, Dict, Any, Tuple
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher

class StateInterface:
    """
    VDA5050 state interface class.
    """
    def __init__(self, config_data:dict, logging:object, state_topic:str, agentId:str) -> None:
        """
        Initialize the StateInterface class.

        :param config_data: Data from the configuration file.
        :param logging: Logging object.
        :param state_topic: MQTT topic for the state messages.
        :param agentId: Agent ID.
        """
        self.logging = logging
        self.state_topic = state_topic
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=state_topic,
                                            client_id=f'state_publisher_agent_{self.agentId}', logging=self.logging)


    def generate_state_message(self, agent:Any, orderID:str, order_updateID:int, lastNodeId:str, lastNodeSequenceId:int,
                               nodeStates:List, edgeStates:List, agvPosition:Dict, driving:bool, actionStates:List,
                               batteryState:Dict, errors:Dict, safetyState:Dict, operatingMode:str="AUTOMATIC") -> None:
        """
        Generate the VDA5050 state message.

        :param agent: Agent object.
        :param orderID: Order ID of the VDA5050 order.
        :param order_updateID: Order update ID of the VDA5050 order.
        :param lastNodeId: ID of the last node.
        :param lastNodeSequenceId: Sequence ID of the last node.
        :param nodeStates: List of node states.
        :param edgeStates: List of edge states.
        :param agvPosition: AGV position.
        :param driving: Driving state.
        :param actionStates: List of action states.
        :param batteryState: Battery state.
        :param operatingMode: Operating mode.
        :param errors: Errors.
        :param safetyState: Safety state.
        :param operatingMode: Operating mode. Default is "AUTOMATIC".
        """
        with agent.agents_object.state_header_id_lock:
            # Generate the state message.
            state_msg = {
                "headerId": agent.agents_object.state_header_id,
                "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-3] + "Z",
                "version": "V2.1.0",
                "manufacturer": "IFL",
                "serialNumber": agent.agentId,
                "orderId": orderID,
                "orderUpdateId": order_updateID,
                "lastNodeId": lastNodeId,
                "lastNodeSequenceId": lastNodeSequenceId,
                "nodeStates": nodeStates,
                "edgeStates": edgeStates,
                "agvPosition": agvPosition,
                "driving": driving,
                "actionStates": actionStates,
                "batteryState": batteryState,
                "operatingMode": operatingMode,
                "errors": errors,
                "safetyState": safetyState
            }

            # Publish the state message.
            self.mqtt_publisher.publish(state_msg, qos=0)

            # Increment the header ID by 1.
            agent.agents_object.state_header_id += 1
