import math
import json
import time
import pygame
import threading
from pygame_screen_record import ScreenRecorder
from fleet_management.graph import Graph
from vda5050_interface.mqtt_clients.mqtt_subscriber import MQTTSubscriber
from vda5050_interface.interfaces.state_interface import StateInterface
from mobile_robot_simulation.visualization import Visualization


class AgentSimulation:
    """
    Class for managing the simulated agents and simulating their movements.
    """
    def __init__(self, config_data, lif_data, agents_initialization_data, logging) -> None:
        """
        Initialize the AgentSimulation object.

        :param config_data: The data from the configuration file.
        :param agent_image: The image representing the agent.
        :param lif_data: The data from the LIF file.
        :param agents_initialization_data: The data from the agents initialization file.
        :param logging: The logging object.
        """
        self.start_time = 0
        self.running = True
        self.logging = logging
        self.config_data = config_data
        self.graph = Graph(lif_data=lif_data)
        self.state_header_id = 1
        self.state_header_id_lock = threading.Lock()
        self.map_id = agents_initialization_data['mapId']
        self.agents = self.get_simulated_agents(agents_initialization_data)
        self.visualization = Visualization(config_data, self.graph)
        self.current_collisions = set()
        self.agent_rotation_diameter = config_data['agent_rotation_diameter']
        self.last_order_header_id = None
        self.fps = config_data['pygame_fps']

    def get_simulated_agents(self, agents_initialization_data) -> list:
        """
        Initialize the simulated agents based on the data from the agents initialization file.
        
        :param agents_initialization_data: The data from the agents initialization file.
        :return: A list of simulated agent objects.
        """
        agents = []
        for agent_initialization_data in agents_initialization_data['agents']:
            agentId = agent_initialization_data['agentId']
            agent_pos = (agent_initialization_data['agentPosition']['x'], agent_initialization_data['agentPosition']['y'])
            agentNodeId = self.get_agent_node(agent_pos)
            agent_theta = agent_initialization_data['agentPosition']['theta']
            agent_velocity = agent_initialization_data['agentVelocity']
            agent_rotation_velocity = agent_initialization_data['agentRotationVelocity']
            agent_state = "IDLE"
            
            agent = SimulatedAgent(self, agentId, agent_pos, agentNodeId, agent_theta, agent_velocity,
                                   agent_rotation_velocity, agent_state, agent_initialization_data)
            agents.append(agent)

        return agents

    def update_agents_position(self, agents_initialization_data) -> None:
        """
        Update the position of the simulated agents based on the data from the agents initialization file.

        :param agents_initialization_data: The data from the agents initialization file.
        """
        for agent in agents_initialization_data['agents']:
            agentId = agent['agentId']
            agent_pos = (agent['agentPosition']['x'], agent['agentPosition']['y'])
            agentNodeId = self.get_agent_node(agent_pos)
            agent_theta = agent['agentPosition']['theta']
            # Normalize the angle to the range [-pi, pi].
            agent_theta = (agent_theta + math.pi) % (2 * math.pi) - math.pi

            for simulated_agent in self.agents:
                if simulated_agent.agentId == agentId:
                    simulated_agent.position = agent_pos
                    simulated_agent.next_node = agentNodeId
                    simulated_agent.theta = agent_theta
                    simulated_agent.agvPosition = {'positionInitialized':True, 'x':agent_pos[0], 'y':agent_pos[1],
                                                   'theta':agent_theta, 'mapId':self.map_id}

    def get_agent_node(self, agent_pos) -> str:
        """
        Get the node ID of the simulated agent based on its position.
        Function places the simulated agent on the node closest to its position.

        :param agent_pos: The position of the simulated agent.
        :return: The node ID of the simulated agent.
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

    def move_towards(self, agent, next_node) -> None:
        """
        Move the simulated agent towards the next node.

        :param agent: The simulated agent object to move.
        :param next_node: The next node to move the simulated agent towards.
        """
        # Calculate the movement direction and distance.
        dx = self.graph.nodes[next_node]['pos'][0] - agent.position[0]
        dy = self.graph.nodes[next_node]['pos'][1] - agent.position[1]
        distance = math.hypot(dx, dy)
        direction = math.atan2(dy, dx)

        # Calculate the angle difference.
        theta_diff = self.calculate_angle_difference(agent.theta, direction)

        # Rotate the agent towards the next node.
        angle_step = agent.rotation_velocity / self.fps
        if abs(theta_diff) > angle_step:
            if theta_diff > 0:
                agent.theta += angle_step
            else:
                agent.theta -= angle_step
        else:
            agent.theta = direction
        # Normalize the angle to the range [-pi, pi].
        agent.theta = (agent.theta + math.pi) % (2 * math.pi) - math.pi

        # Move the agent towards the next node, if the direction is correct.
        if abs(theta_diff) <= angle_step and distance > 0:
            dx /= distance
            dy /= distance
            agent.position = (round(agent.position[0] + dx * agent.velocity / self.fps, 3), 
                              round(agent.position[1] + dy * agent.velocity / self.fps, 3))
        
        agent.agvPosition = {'positionInitialized':True, 'x':agent.position[0], 'y':agent.position[1],
                             'theta':agent.theta, 'mapId':self.map_id}

    def update(self, agent) -> None:
        """
        Update the state of the simulated agent.

        :param agent: The simulated agent object to update.
        """
        if agent.active_action:
            # Check if the agent has finished the action.
            if time.time() - agent.action_start_time > self.config_data['process_time_loading_unloading']:
                agent.active_action = False
                agent.action_start_time = None
            else:
                # Wait for the action to finish.
                return

        if len(agent.nodeStates) == 0:
            # The agent has no more nodes to move to.
            return
        if not agent.nodeStates[0]['released']:
            # The next node is not released yet.
            return
        if len(agent.edgeStates) > 0 and not agent.edgeStates[0]['released']:
            # The next edge is not released yet.
            return

        # Move the agent towards the next node of tha agent's path.
        self.move_towards(agent, agent.next_node)
        
        # Check if the agent has reached the next node.
        if math.hypot(agent.position[0] - self.graph.nodes[agent.next_node]['pos'][0],
                      agent.position[1] - self.graph.nodes[agent.next_node]['pos'][1]) < (agent.velocity / self.fps):
            agent.lastNodeId = agent.nodeStates[0]['nodeId']
            agent.lastNodeSequenceId = agent.nodeStates[0]['sequenceId']
            agent.position = self.graph.nodes[agent.lastNodeId]['pos']

            # Check if the agent has any actions to perform at the next node.
            if agent.nodeStates[0]['actions'] != []:
                for action in agent.nodeStates[0]['actions']:
                    if action['actionType'] == 'pick':
                        for actionState in agent.actionStates:
                            if actionState['actionId'] == action['actionId']:
                                actionState['actionStatus'] = 'FINISHED'
                                agent.loaded = True
                                agent.active_action = True
                                agent.action_start_time = time.time()
                    elif action['actionType'] == 'drop':
                        for actionState in agent.actionStates:
                            if actionState['actionId'] == action['actionId']:
                                actionState['actionStatus'] = 'FINISHED'
                                agent.loaded = False
                                agent.active_action = True
                                agent.action_start_time = time.time()

            agent.nodeStates.pop(0)
            agent.edgeStates.pop(0)

            if len(agent.nodeStates) > 0:
                # Update the next node of the agent.
                agent.next_node = agent.nodeStates[0]['nodeId']

            else:
                # The agent has reached the last node of the path.
                agent.next_node = None
                agent.driving = False
                agent.agent_state = "IDLE"

            agent.agvPosition = {'positionInitialized':True, 'x':agent.position[0], 'y':agent.position[1],
                                 'theta':agent.theta, 'mapId':self.map_id}

            if self.config_data['fleet_management_mode'] == "SIMULATED_AGENTS":
                # Generate and send the state message to the fleet manager.
                agent.state_interface.generate_state_message(agent, orderID=agent.orderId, order_updateID=agent.order_updateId, lastNodeId=agent.lastNodeId,
                                                            lastNodeSequenceId=agent.lastNodeSequenceId, nodeStates=agent.nodeStates,
                                                            edgeStates=agent.edgeStates, agvPosition=agent.agvPosition, driving=agent.driving,
                                                            actionStates=agent.actionStates, batteryState=agent.batteryState, errors=agent.errors,
                                                            safetyState=agent.safetyState, operatingMode=agent.operatingMode)
                agent.last_state_message_time = time.time()

    def calculate_angle_difference(self, angle1, angle2) -> float:
        """
        Calculate the difference between two angles.

        :param angle1: The first angle in radians.
        :param angle2: The second angle in radians.
        :return: The difference between the two angles in radians.
        """
        return (angle2 - angle1 + math.pi) % (2 * math.pi) - math.pi

    def check_collisions(self) -> None:
        """
        Check for collisions between the simulated agents.
        """
        # Reset the new collisions set.
        new_collisions = set()

        for agent in self.agents:
            agent.safetyState['fieldViolation'] = False

        # Iterate over each pair of agents once.
        for i, agent1 in enumerate(self.agents):
            for j in range(i + 1, len(self.agents)):
                agent2 = self.agents[j]
                distance = math.hypot(agent1.position[0] - agent2.position[0], agent1.position[1] - agent2.position[1])
                if distance < self.agent_rotation_diameter:
                    # Collision detected between agent1 and agent2.
                    collision_pair = tuple(sorted((agent1.agentId, agent2.agentId)))
                    new_collisions.add(collision_pair)
                    if collision_pair not in self.current_collisions:
                        self.logging.warning(f"Collision detected between agent {agent1.agentId} and agent {agent2.agentId} at time {round(time.time() - self.start_time, 2)}.")
                    agent1.safetyState['fieldViolation'] = True
                    agent2.safetyState['fieldViolation'] = True

        # Update the current collisions set.
        self.current_collisions = new_collisions

    def start_threads(self, start_time, task_management, task_assignment, fleet_management) -> None:
        """
        Share the start time and start the threads.

        :param start_time: The start time of the simulation.
        :param task_management: The task management object.
        :param task_assignment: The task management object.
        :param fleet_management: The fleet management object.
        """
        # Share the start time.
        task_management.start_time = start_time
        task_assignment.start_time = start_time
        fleet_management.start_time = start_time
        fleet_management.path_planning.start_time = start_time
        fleet_management.agents_object.start_time = start_time

        # Start the threads.
        task_management.task_generation_thread.start()
        task_assignment.assignment_thread.start()
        fleet_management.path_planning_thread.start()

    def run(self, start_time, run_time,
            task_management, task_assignment, fleet_management) -> None:
        """
        Run the agent simulation.

        :param start_time: The start time of the simulation.
        :param run_time: The time in seconds to run the simulation.
        :param task_management: The task management object.
        :param task_assignment: The task management object.
        :param fleet_management: The fleet management object.
        """
        # Share the start time and start the threads.
        self.start_time = round(start_time, 2)
        self.start_threads(self.start_time, task_management, task_assignment, fleet_management)

        # Start the screen recorder.
        recorder = ScreenRecorder(self.fps)
        recorder.start_rec()
        
        # Start the agent simulation.
        self.running = True

        while self.running:
            # Fill the off-screen surface with white.
            self.visualization.offscreen_surface.fill((255, 255, 255))
            
            # Event handling.
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.stop()
                    break

            # Check if the simulation run time has passed.
            time_running = round(time.time() - self.start_time, 2)
            if time_running > run_time:
                self.stop()
                break

            # Update the position of the simulated agents based on their transportation tasks.
            for agent in self.agents:
                self.update(agent)
            
            # Check for collisions between the simulated agents.
            self.check_collisions()
            
            # Draw graph and agents on the screen.
            self.visualization.draw_graph(agents=self.agents)
            self.visualization.draw_agents(agents=self.agents)

            # Display collision message if a collision is detected.
            if self.current_collisions:
                self.visualization.display_collision_message()
            
            # Copy the off-screen surface to the main screen.
            self.visualization.screen.blit(self.visualization.offscreen_surface, (0, 0))

            # Update display with 30 FPS.
            pygame.display.flip()
            self.visualization.clock.tick(self.fps)
        
        # Stop the screen recorder and save the recording.
        recorder.stop_rec()
        recorder.save_recording("data/output_files/recorded_videos/simulation_recording.mp4")

        # Stop the simulation.
        pygame.quit()
        self.logging.info(f"Agent simulation finished at time {round(time.time() - self.start_time, 2)}.")

    def stop(self) -> None:
        """
        Stop the agent simulation.
        """
        self.running = False


class SimulatedAgent:
    """
    Class for generating the simulated agent object.
    """
    def __init__(self, agents_object, agentId, agent_pos, agentNodeId, agent_theta, agent_velocity, agent_rotation_velocity,
                 agent_state, agent_initialization_data, next_node=None) -> None:
        """
        Initialize the simulated agent object.

        :param agents_object: The AgentSimulation object.
        :param agentId: The ID of the agent.
        :param agent_pos: The position of the agent.
        :param agentNodeId: The node ID of the agent.
        :param agent_theta: The angle of the agent.
        :param agent_velocity: The velocity of the agent in m/s.
        :param agent_rotation_velocity: The rotation velocity of the agent in rad/s.
        :param agent_state: The state of the agent. Can be IDLE, PLANNING or EXECUTING. Default is IDLE.
        :param agent_initialization_data: The data from the agents initialization file.
        :param next_node: The next node ID of the agents path. Default is None.
        """
        # Static attributes.
        self.agents_object = agents_object
        self.agentId = agentId
        self.velocity = agent_velocity
        self.rotation_velocity = agent_rotation_velocity
        self.state_topic = agent_initialization_data['stateTopic']
        self.order_topic = agent_initialization_data['orderTopic']
        self.mqtt_subscriber_order = MQTTSubscriber(config_data=self.agents_object.config_data, logging=self.agents_object.logging, on_message=self.order_callback,
                                                    channel=self.order_topic, client_id=f'order_subscriber_simulated_agent_{self.agentId}')
        self.state_interface = StateInterface(config_data=self.agents_object.config_data, logging=self.agents_object.logging,
                                              state_topic=self.state_topic, agentId=self.agentId)

        # Dynamic attributes.
        self.agent_state = agent_state
        self.position = agent_pos
        self.theta = agent_theta
        self.next_node = next_node
        self.orderId = ""
        self.order_updateId = 0
        self.lastNodeId = ""
        self.lastNodeSequenceId = 0
        self.nodeStates = []
        self.edgeStates = []
        self.agvPosition = {'positionInitialized':True, 'x':self.position[0], 'y':self.position[1],
                            'theta':self.theta, 'mapId':self.agents_object.map_id}
        self.driving = False
        self.actionStates = []
        self.batteryState = {"batteryCharge": 80, "charging": False}
        self.errors = []
        self.safetyState = {"eStop": "NONE", "fieldViolation": False}
        self.operatingMode = "AUTOMATIC"
        self.collision = False
        self.loaded = False
        self.active_action = False
        self.action_start_time = None

        self.used_orderIds = set()
        self.last_orderUpdateId = 0

        if self.agents_object.config_data['fleet_management_mode'] == "SIMULATED_AGENTS":
            # Start the regular state message thread.
            self.last_state_message_time = time.time()
            self.regular_state_thread = threading.Thread(target=self.regular_state_message, args=())
            self.regular_state_thread.start()

    def regular_state_message(self, timeout=30) -> None:
        """
        Send a state message latest every 30 seconds.

        :param timeout: The time in seconds to wait before sending an automatic state message. Default is 30 seconds.
        """
        while self.agents_object.running:
            if time.time() - self.last_state_message_time > timeout:
                self.state_interface.generate_state_message(agent=self, orderID=self.orderId, order_updateID=self.order_updateId, lastNodeId=self.lastNodeId,
                                                            lastNodeSequenceId=self.lastNodeSequenceId, nodeStates=self.nodeStates,
                                                            edgeStates=self.edgeStates, agvPosition=self.agvPosition, driving=self.driving,
                                                            actionStates=self.actionStates, batteryState=self.batteryState, errors=self.errors,
                                                            safetyState=self.safetyState, operatingMode=self.operatingMode)
                self.last_state_message_time = time.time()
            time.sleep(1)

    def order_callback(self, client, userdata, msg) -> None:
        """
        Callback function for the order message.
        """
        self.agents_object.logging.info(f"Client {self.mqtt_subscriber_order.client_id} received message `{msg.payload.decode()}` from topic `{msg.topic}`.")  # `{msg.payload.decode()}`

        # Extract the message data, check correctness, and update the agent order.
        message_data = json.loads(msg.payload.decode())
        self.order_message_validation(message_data)
        self.update_agent_order_callback(message_data)
    
    def update_agent_order_callback(self, message_data) -> None:
        """
        Update the order of the simulated agent based on the order message data.

        :param message_data: The data from the order message.
        """
        self.orderId = message_data['orderId']
        self.order_updateId = message_data['orderUpdateId']

        if self.order_updateId == 0:
            # New order.
            self.nodeStates = message_data['nodes']
            self.edgeStates = message_data['edges']
            self.actionStates = []
            self.driving = True
            self.agent_state = "EXECUTING"

            # Check if the agent is on first node of the order.
            if self.position == (self.nodeStates[0]['nodePosition']['x'], self.nodeStates[0]['nodePosition']['y']):
                # If yes, remove the first node from the nodeStates list.
                self.lastNodeId = message_data['nodes'][0]['nodeId']
                self.lastNodeSequenceId = message_data['nodes'][0]['sequenceId']
                self.nodeStates.pop(0)
            else:
                # If not, add a edge to the start node at the beginning of the edgeStates list.
                egde_to_start_node = { "edgeId": "egde_to_start_node", "sequenceId": 0,"released": True}
                self.edgeStates.insert(0, egde_to_start_node)

            # Update the next node of the agent, if order was not a single node order. If agent is on the node of a single node order, no next node exists.
            if len(self.nodeStates) > 0:
                self.next_node = self.nodeStates[0]['nodeId']

            # It is assumed that actions are not changed with the order updates. If actions are changed, the actionStates list should be updated.
            # The initial action status is 'WAITING'.
            for node in self.nodeStates:
                if node['actions'] != []:
                    for action in node['actions']:
                        actionState = {"actionId": action['actionId'], "actionType": action['actionType'], "actionStatus": "WAITING"}
                        self.actionStates.append(actionState)
        
        else:
            # Order update. Update the node and edge states.
            nodeStates = [node for node in self.nodeStates if node['released']]
            if len(nodeStates) == 0:
                for node in message_data['nodes']:
                    if node['sequenceId'] >= self.nodeStates[0]['sequenceId']:
                        nodeStates.append(node)
            else:
                for node in message_data['nodes']:
                    if node['sequenceId'] > nodeStates[-1]['sequenceId']:
                        nodeStates.append(node)
            self.nodeStates = nodeStates
            edgeStates = [edge for edge in self.edgeStates if edge['released']]
            if len(edgeStates) == 0:
                for edge in message_data['edges']:
                    if edge['sequenceId'] >= self.edgeStates[0]['sequenceId']:
                        edgeStates.append(edge)
            else:
                for edge in message_data['edges']:
                    if edge['sequenceId'] > edgeStates[-1]['sequenceId']:
                        edgeStates.append(edge)
            self.edgeStates = edgeStates

        if self.agents_object.config_data['fleet_management_mode'] == "SIMULATED_AGENTS":
            # Generate and send the state message to the fleet manager. State message is required to be sent after an order message is received.
            self.state_interface.generate_state_message(agent=self, orderID=self.orderId, order_updateID=self.order_updateId, lastNodeId=self.lastNodeId,
                                                        lastNodeSequenceId=self.lastNodeSequenceId, nodeStates=self.nodeStates,
                                                        edgeStates=self.edgeStates, agvPosition=self.agvPosition, driving=self.driving,
                                                        actionStates=self.actionStates, batteryState=self.batteryState, errors=self.errors,
                                                        safetyState=self.safetyState, operatingMode=self.operatingMode)
            self.last_state_message_time = time.time()

    def order_message_validation(self, message_data):
        """
        Validate the correctness of the order message.

        :param message_data: The data from the order message.
        """
        # Check the header ID of the order message.
        if self.agents_object.last_order_header_id is not None and self.agents_object.last_order_header_id + 1 != message_data['headerId']:
            self.agents_object.logging.error(f"Order message header ID mismatch. Expected {self.agents_object.last_order_header_id + 1}, got {message_data['headerId']}.")
        self.agents_object.last_order_header_id = message_data['headerId']
        
        # Check the order ID of the order message.
        if message_data['orderId'] in self.used_orderIds:
            # Check the order update ID of the order message.
            if message_data['orderUpdateId'] == 0:
                self.agents_object.logging.error(f"Order message order ID already used, but should be unique, if order update ID is 0. Order ID: {message_data['orderId']}.")
                self.last_orderUpdateId = 0
            elif self.last_orderUpdateId + 1 != message_data['orderUpdateId']:
                self.agents_object.logging.error(f"Order message order update ID mismatch. Expected {self.last_orderUpdateId + 1}, got {message_data['orderUpdateId']}.")
                self.last_orderUpdateId = message_data['orderUpdateId']
                self.check_first_base_node(message_data)
            else:
                self.last_orderUpdateId += 1
                self.check_first_base_node(message_data)
        
        else:
            self.used_orderIds.add(message_data['orderId'])
            # Check the order update ID of the order message.
            if message_data['orderUpdateId'] != 0:
                self.agents_object.logging.error(f"Order message order update ID should be 0 for a new order. Order update ID: {message_data['orderUpdateId']}.")
            # Check the first node sequence ID of the order message.
            if message_data['nodes'][0]['sequenceId'] == 0:
                self.agents_object.logging.error(f"Order message first node sequence ID should be 1. Got 0.")
            self.last_orderUpdateId = message_data['orderUpdateId']

        # Check if the order message contains at least one base node.
        if not message_data['nodes'][0]['released']:
            self.agents_object.logging.error(f"Order message should contain at least one base node. First node is not released.")

    def check_first_base_node(self, message_data) -> None:
        """
        Check the first base node of the order message. Must be the same as the last base node of the previous order.

        :param message_data: The data from the order message.
        """
        last_base_node = None
        for node in self.nodeStates:
            if node['released']:
                last_base_node = node
            else:
                break
        
        if last_base_node is not None and last_base_node['nodeId'] != message_data['nodes'][0]['nodeId']:
            self.agents_object.logging.error(f"Order message first base node is not the same as the last base node of the previous order. First base node: {message_data['nodes'][0]['nodeId']}, last base node: {last_base_node['nodeId']}.")
