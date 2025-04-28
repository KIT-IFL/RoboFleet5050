import json
from vda5050_interface.mqtt_clients.mqtt_subscriber import MQTTSubscriber

class AgentsInitialization:
    """
    Class for initializing the digital twin agents based on the position of the real agents.
    """
    def __init__(self, config_data: dict, agents_initialization_data: dict, logging: object, initialization_event: object) -> None:
        """
        Initialize the AgentsInitialization object.

        :param config_data: The data from the configuration file.
        :param agents_initialization_data: The data from the agents initialization file.
        :param logging: The logging object.
        :param initialization_event: The event for synchronization.
        """
        self.logging = logging
        self.agents_initialization_data = agents_initialization_data
        self.mqtt_clients = {}
        self.agentIds = [agent['agentId'] for agent in agents_initialization_data['agents']]
        self.subscribe_to_state_topic(config_data=config_data, logging=logging, agents_initialization_data=agents_initialization_data)
        self.initialization_event = initialization_event
        self.state_messages_received = 0
        self.agents_send_initial_state = []

    def subscribe_to_state_topic(self, config_data, logging, agents_initialization_data) -> None:
        """
        Subscribe to the state topic of the agents.

        :param agents_initialization_data:
        """
        for agent in agents_initialization_data['agents']:
            mqtt_subscriber_state = MQTTSubscriber(config_data=config_data, logging=logging, on_message=self.initial_state_callback,
                                                   channel=agent['stateTopic'], client_id=f'state_initialization_subscriber_agent_{agent['agentId']}')
            self.mqtt_clients[agent['agentId']] = mqtt_subscriber_state
    
    def initial_state_callback(self, client, userdata, msg) -> None:
        """
        Callback function for processing the initial state message of the agents.
        The position of the agents in the agentsInitialization file is updated based on the initial state message of the real agents.
        The serialNumber of the agent in the initial state message must match the agentId in the agentsInitialization file.

        :param client: The MQTT client.
        :param userdata: The user data.
        :param msg: The message received from the MQTT broker.
        """
        self.logging.info(f"Client {client._client_id} received message `{msg.payload.decode()}` from topic `{msg.topic}`.")  # `{msg.payload.decode()}`

        # Extract the message data and update the agent state.
        message_data = json.loads(msg.payload.decode())
        self.logging.info(f"Initial state message: {message_data}.")

        # Check if the message is from a valid agent.
        if message_data['serialNumber'] not in self.agentIds:
            self.logging.warning(f"The agent ID {message_data['serialNumber']} does not match any agent ID in the agentsInitialization_file.")
            return

        # Update the agents initial position.
        for agent in self.agents_initialization_data['agents']:
            if agent['agentId'] == message_data['serialNumber'] and agent['agentId'] not in self.agents_send_initial_state:
                agent['agentPosition']['x'] = message_data['agvPosition']['x']
                agent['agentPosition']['y'] = message_data['agvPosition']['y']
                agent['agentPosition']['theta'] = message_data['agvPosition']['theta']
                self.agents_send_initial_state.append(agent['agentId'])
                self.state_messages_received += 1
                break

        # Check if all agents have sent their initial state message.
        if self.state_messages_received == len(self.agents_initialization_data['agents']):
            self.logging.info(f"Number of agents that have sent their initial state message: {self.state_messages_received}. All agents have sent their initial state message.")
            self.logging.info(f"Agents initial positions: {self.agents_initialization_data['agents']}.")
            self.remove_mqtt_subscribers()
            self.initialization_event.set()
        else:
            self.logging.info(f"Number of agents that have sent their initial state message: {self.state_messages_received}.")

    def remove_mqtt_subscribers(self) -> None:
        """
        Remove the MQTT subscribers.
        """
        for agent in self.agents_initialization_data['agents']:
            self.mqtt_clients[agent['agentId']].client.loop_stop()
            self.mqtt_clients[agent['agentId']].client.disconnect()
            self.logging.info(f"Disconnected client {self.mqtt_clients[agent['agentId']].client._client_id} from MQTT broker.")

    def update_agents_initialization_file(self) -> dict:
        """
        Update the agents initialization file with the initial positions of the agents.

        :return: The updated agents initialization data.
        """
        return self.agents_initialization_data
