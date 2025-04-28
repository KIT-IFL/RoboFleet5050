import math
import json
import time
import threading
from vda5050_interface.mqtt_clients.mqtt_subscriber import MQTTSubscriber
from vda5050_interface.interfaces.order_interface import OrderInterface


class Agents:
    """
    Class for managing the digital twin agents of the simulated/real agents.
    """
    def __init__(self, config_data, graph, agents_initialization_data, logging) -> None:
        """
        Initialize the Agents object.

        :param config_data: The data from the configuration file.
        :param graph: The graph object based on which the agents are controlled by the fleet manager.
        :param agents_initialization_data: The data from the agents initialization file.
        :param logging: The logging object.
        """
        self.start_time = 0
        self.logging = logging
        self.config_data = config_data
        self.graph = graph
        self.order_header_id = 1
        self.order_header_id_lock = threading.Lock()
        self.map_id = agents_initialization_data['mapId']
        self.agents = self.get_agents(agents_initialization_data)
    
    def get_agents(self, agents_initialization_data) -> list:
        """
        Initialize the digital twins of the agents based on the data from the agents initialization file.
        
        :param agents_initialization_data: The data from the agents initialization file.
        :return: A list of digital twin agent objects.
        """
        agents = []
        for agent_initialization_data in agents_initialization_data['agents']:
            agentId = agent_initialization_data['agentId']
            agent_pos = (agent_initialization_data['agentPosition']['x'], agent_initialization_data['agentPosition']['y'])
            agentNodeId = self.get_agent_node(agent_pos)
            agent_theta = agent_initialization_data['agentPosition']['theta']
            agent_velocity = agent_initialization_data['agentVelocity']
            agent_rotation_velocity = agent_initialization_data['agentRotationVelocity']
            agent_state = "INITIALIZING"
            
            agent = Agent(self, agentId, agent_pos, agentNodeId, agent_theta, agent_velocity,
                          agent_rotation_velocity, agent_state, agent_initialization_data)
            agents.append(agent)

        return agents
    
    def get_agent_node(self, agent_pos) -> str:
        """
        Get the node ID of the digital twin agent based on its position.
        Function places the digital twin agent on the node closest to its position.

        :param agent_pos: The position of the digital twin agent.
        :return: The node ID of the digital twin agent.
        """
        min_dist = math.inf
        agentNodeId = None
        for nodeId, node in self.graph.nodes.items():
            dist = math.dist(agent_pos, node['pos'])
            if dist < min_dist:
                min_dist = dist
                agentNodeId = nodeId
                if min_dist == 0:
                    break

        return agentNodeId


class Agent:
    """
    Class for generating the digital twin agent object.
    """
    def __init__(self, agents_object, agentId, agent_pos, agentNodeId, agent_theta, agent_velocity, agent_rotation_velocity,
                 agent_state, agent_initialization_data, task=None, path=None) -> None:
        """
        Initialize the digital twin agent object.

        :param agents_object: The Agents object.
        :param agentId: The ID of the agent.
        :param agent_pos: The position of the agent.
        :param agentNodeId: The node ID of the agent.
        :param agent_theta: The angle of the agent.
        :param agent_velocity: The velocity of the agent in m/s.
        :param agent_rotation_velocity: The rotation velocity of the agent in rad/s.
        :param agent_state: The state of the agent. Can be INITIALIZING, IDLE, PLANNING or EXECUTING. Default is INITIALIZING.
        :param agent_initialization_data: The data from the agents initialization file.
        :param task: The task of the agent. Default is None.
        :param path: The path of the agent. Default is None.
        """
        # Static attributes.
        self.agents_object = agents_object
        self.agentId = agentId
        self.velocity = agent_velocity
        self.rotation_velocity = agent_rotation_velocity
        self.state_topic = agent_initialization_data['stateTopic']
        self.order_topic = agent_initialization_data['orderTopic']
        self.mqtt_subscriber_state = MQTTSubscriber(config_data=self.agents_object.config_data, logging=self.agents_object.logging, on_message=self.state_callback,
                                                    channel=self.state_topic, client_id=f'state_subscriber_agent_{self.agentId}')
        self.order_interface = OrderInterface(config_data=self.agents_object.config_data, logging=self.agents_object.logging,
                                              map_id=self.agents_object.map_id, order_topic=self.order_topic, agentId=self.agentId)

        # Dynamic attributes.
        self.agent_state = agent_state
        self.position = agent_pos
        self.theta = agent_theta
        self.task = task
        self.path = path
        self.path_start_time = 0
        self.path_start_time_delay = 0
        self.orderId = ""
        self.order_updateId = 0
        self.lastNodeId = agentNodeId
        self.lastNodeSequenceId = 0
        self.nodeStates = []
        self.edgeStates = []
        self.actionStates = []
        self.agvPosition = {'positionInitialized':True, 'x':self.position[0], 'y':self.position[1],
                            'theta':self.theta, 'mapId':self.agents_object.map_id}
        self.agvPosition_visualiazion = {'positionInitialized':True, 'x':self.position[0], 'y':self.position[1],
                                         'theta':self.theta, 'mapId':self.agents_object.map_id}
        self.driving = False
        self.loaded = False
        self.pick_action_uuid = None
        self.drop_action_uuid = None
        self.error = False
        self.waiting = False
        self.nextNodeId = None
        self.nextNodeIndex = 1
        self.batteryState = {"batteryCharge": 80, "charging": False}
        self.safetyState = {"eStop": "NONE", "fieldViolation": False}
        self.operatingMode = "AUTOMATIC"
        self.collision = False

        # Threads and events.
        self.next_node_event = threading.Event()
        self.station_event = threading.Event()

    def update_agent_state(self, agent_state, position, theta, task, path, orderId, order_updateId,
                           lastNodeId, lastNodeSequenceId, driving) -> None:
        """
        Update the state of the digital twin agent.

        :param agent_state: The state of the agent.
        :param position: The position of the agent.
        :param theta: The angle of the agent.
        :param task: The task of the agent.
        :param path: The path of the agent.
        :param orderId: The order ID.
        :param order_updateId: The order update ID.
        :param lastNodeId: The last node ID.
        :param lastNodeSequenceId: The last node sequence ID.
        :param driving: The driving state.
        :param loaded: The loaded state.
        """
        self.agent_state = agent_state
        self.position = position
        self.theta = theta
        self.task = task
        self.path = path
        self.orderId = orderId
        self.order_updateId = order_updateId
        self.lastNodeId = lastNodeId
        self.lastNodeSequenceId = lastNodeSequenceId
        self.agvPosition = {'positionInitialized':True, 'x':self.position[0], 'y':self.position[1],
                            'theta':self.theta, 'mapId':self.agents_object.map_id}
        self.driving = driving
        self.nextNodeId = None
        self.nextNodeIndex = 1

        if self.path is not None and self.nextNodeIndex < len(self.path['nodes']):
            # Set the next expected node ID.
            self.nextNodeId = self.path['nodes'][self.nextNodeIndex]['nodeId']
            self.nextNodeIndex += 1
    
    def state_callback(self, client, userdata, msg) -> None:
        """
        Callback function for the state message.

        :param client: MQTT client.
        :param userdata: User data.
        :param msg: Message. 
        """
        self.agents_object.logging.info(f"Client {self.mqtt_subscriber_state.client_id} received message `{msg.payload.decode()}` from topic `{msg.topic}`.")  # `{msg.payload.decode()}`

        # Extract the message data and update the agent state.
        message_data = json.loads(msg.payload.decode())

        if self.nextNodeId == message_data['lastNodeId']:
            # Agent has reached the next node.
            self.next_node_event.set()

            if self.path is not None and self.nextNodeIndex < len(self.path['nodes']):
                # Set the next expected node ID.
                self.nextNodeId = self.path['nodes'][self.nextNodeIndex]['nodeId']
                self.nextNodeIndex += 1
        
        if message_data['lastNodeId'] is not None and message_data['lastNodeId'] != "":
            if self.agents_object.graph.nodes[message_data['lastNodeId']]['type'] == 'station':
                # Agent has reached a station.
                self.station_event.set()

        self.update_agent_state_callback(message_data)

    def update_agent_state_callback(self, message_data) -> None:
        """
        Update the state of the digital twin agent based on the state message.

        :param message_data: The data from the state message.
        """
        if message_data['lastNodeId'] != "":
            self.lastNodeId = message_data['lastNodeId']
            self.lastNodeSequenceId = message_data['lastNodeSequenceId']
        self.nodeStates = message_data['nodeStates']
        self.edgeStates = message_data['edgeStates']
        self.actionStates = message_data['actionStates']
        self.agvPosition = message_data['agvPosition']
        self.driving = message_data['driving']
        self.position = (self.agvPosition['x'], self.agvPosition['y'])
        if self.agvPosition['theta'] < -math.pi or self.agvPosition['theta'] > math.pi:
            # Normalize the angle to be within the range of -pi to pi.
            self.agvPosition['theta'] = (self.agvPosition['theta'] + math.pi) % (2 * math.pi) - math.pi
            self.agents_object.logging.warning(f"Agent {self.agentId} has an invalid angle. Normalizing the angle to be within the range of -pi to pi.")
        self.theta = self.agvPosition['theta']

        for action in self.actionStates:
            # Adjust the loaded state of the agent based on the action status.
            if action['actionId'] == self.drop_action_uuid and action['actionStatus'] == 'FINISHED':
                # Agent has dropped the load.
                self.loaded = False
                if self.task is not None:
                    self.task['task_completed'] = True
            elif action['actionId'] == self.pick_action_uuid and action['actionStatus'] == 'FINISHED':
                # Agent has picked up the load.
                self.loaded = True

            # Remove finished actions from the action states.
            actionStates_copy = self.actionStates.copy()
            if action['actionStatus'] == 'FINISHED':
                actionStates_copy.remove(action)
            self.actionStates = actionStates_copy

        if self.nodeStates == [] and self.edgeStates == [] and self.actionStates == [] and self.orderId != "":
            if self.task is not None and self.task['task_completed']:
                # Agent has completed the task. Set the agent to IDLE.
                if self.path is not None:
                    self.agents_object.logging.info(f"Agent {self.agentId} has completed task {self.task['task_id']} and is IDLE at node {self.lastNodeId} at time {round(time.time() - self.agents_object.start_time, 2)}. Planned path end time: {self.path['nodes'][-1]['reservationTime']}.")
                else:
                    self.agents_object.logging.info(f"Agent {self.agentId} has completed task {self.task['task_id']} and is IDLE at node {self.lastNodeId} at time {round(time.time() - self.agents_object.start_time, 2)}.")
                self.path = None
                self.task = None
                self.agent_state = "IDLE"
            elif self.task is not None and not self.task['task_completed'] and not self.waiting and not self.error:
                # Agent has not completed the task. Plan a path for the agent to fulfill the task.
                self.agents_object.logging.info(f"Task {self.task['task_id']} is re-executed by agent {self.agentId}. Start node: {self.task['start_node']}. Goal node: {self.task['goal_node']} at time {round(time.time() - self.agents_object.start_time, 2)}.")
                self.path = None
                self.agent_state = "PLANNING"
            elif self.task is None and not self.waiting:
                # Agent has no task assigned. Set the agent to IDLE.
                if self.path is not None:
                    self.agents_object.logging.info(f"Agent {self.agentId} is IDLE at node {self.lastNodeId} at time {round(time.time() - self.agents_object.start_time, 2)}. Planned path end time: {self.path['nodes'][-1]['reservationTime']}.")
                else:
                    self.agents_object.logging.info(f"Agent {self.agentId} is IDLE at node {self.lastNodeId} at time {round(time.time() - self.agents_object.start_time, 2)}.")
                self.path = None
                self.agent_state = "IDLE"
            else:
                # Agent is waiting for the next node.
                self.agents_object.logging.info(f"Agent {self.agentId} is waiting for the next node at node {self.lastNodeId} at time {round(time.time() - self.agents_object.start_time, 2)}.")

    def request_and_monitor_process(self, node, node_resource) -> None:
        """
        Request the node resource of the agent's initial node and start a process to monitor the position.

        :param node: The node of the agent.
        """
        # Request the resource of the node.
        node_request = node_resource.request()

        if not node_request.is_set():
            raise Exception(f"Failed to reserve node {node}.")

        else:
            self.agents_object.logging.info(f"Agent {self.agentId} reserved node {node} / {self.agents_object.graph.nodes[node]['pos']} at time {round(time.time() - self.agents_object.start_time, 2)}.")
    
        # Wait for the agent's position to change.
        monitor_agent_position_thread = threading.Thread(target=self.monitor_agent_position, args=(node,))
        monitor_agent_position_thread.start()
        monitor_agent_position_thread.join()

        # Release the node resource.
        node_resource.release()

    def monitor_process(self, node, node_resource) -> None:
        """
        Start a process to monitor the node of an agent.

        :param node: The current node of the agent.
        :param node_resource: Simpy resource of the node.
        """
        # Wait for the agent's node to change.
        monitor_agent_position_thread = threading.Thread(target=self.monitor_agent_position, args=(node,))
        monitor_agent_position_thread.start()
        monitor_agent_position_thread.join()

        # Release the node resource.
        node_resource.release()

    def monitor_agent_position(self, node) -> None:
        """
        Monitor the node of an agent and trigger an event when it changes.

        :param node: The current node of the agent.
        """
        initial_node = node
        if self.lastNodeId != initial_node:
            raise ValueError(f"Agent {self.agentId} is not at the initial node.")

        while True:
            if self.lastNodeId != initial_node and self.lastNodeId != "":
                # Node has changed. End the process.
                self.agents_object.logging.info(f"Agent {self.agentId} released node {initial_node} / {self.agents_object.graph.nodes[initial_node]['pos']} at time {round(time.time() - self.agents_object.start_time, 2)}.")
                return

            # Agent has not changed node. Position is monitored at a certain frequency.
            time.sleep(0.1)

    def reserve_node_infinitly(self, path_planning, node, required_reservation) -> None:
        """
        Reserve the node for the agent infinitely.

        :param path_planning: Path planning object.
        :param node: The node to be reserved.
        :param required_reservation: The required reservation.
        """
        # Reserve the node.
        path_planning.reserve_path(self, [node], [required_reservation])

        # Request the node resource and monitor the agent's position.
        monitor_thread = threading.Thread(target=self.request_and_monitor_process, args=(node, path_planning.graph.node_resources[node]))
        monitor_thread.start()
