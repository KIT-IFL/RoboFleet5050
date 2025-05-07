import time
import math
import uuid
import threading
from sortedcontainers import SortedList


class FleetManagement:
    """
    Class for managing the fleet of simulated/real agents.
    """
    def __init__(self, config_data, graph, agents_object, task_management) -> None:
        """
        Initialize the fleet management object.
        
        :param config_data: The configuration data.
        :param graph: The graph object.
        :param agents_object: The agents object managing the digital twin agents.
        :param task_management: The task management object.
        """
        self.start_time = 0
        self.config_data = config_data
        self.graph = graph
        self.agents_object = agents_object
        self.task_management=task_management
        self.agents_initialized = False
        self.orderId = 1
        self.path_planning = PathPlanning(config_data=self.config_data, graph=self.graph)
        self.path_planning_thread = threading.Thread(target=self.path_planning_manager)
    
    def path_planning_manager(self) -> None:
        """
        Paths are planned for the agents that are in the PLANNING state.
        At the beginning the agents are initialized by moving them to the dwelling nodes.
        """
        # Reserve the initial nodes of the agents for a infinite time.
        for agent in self.agents_object.agents:
            # Reserve the node for the agent.
            self.path_planning.reserve_path(agent, [agent.lastNodeId], [(self.start_time, float('inf'))])

            # Request the node resource and monitor the agent's position.
            monitor_thread = threading.Thread(target=agent.request_and_monitor_process,
                                              args=(agent.lastNodeId, self.graph.node_resources[agent.lastNodeId]))
            monitor_thread.start()

        # Initialize the agents by moving them to the dwelling nodes.
        if not self.agents_initialized:
            self.agents_object.logging.info(f"Initializing agents on dwelling nodes at time {round(time.time() - self.start_time, 2)}.")
            self.initialize_agents_on_dwelling_nodes()
            self.agents_initialized = True

        while self.agents_initialized:
            for agent in self.agents_object.agents:
                if agent.agent_state == "PLANNING":
                    if agent.task['start_node'] == agent.lastNodeId or agent.loaded:
                        # The agent is at the start node of the task or the agent is already loaded.
                        # Plan the path to the goal node and the closest dwelling node.
                        start_node = agent.lastNodeId
                        closest_dwelling_node = self.graph.get_closest_dwelling_node(self.graph.nodes[agent.task['goal_node']]['pos'],
                                                                                     self.graph.dwelling_nodes)
                        goal_nodes = [agent.task['goal_node'], closest_dwelling_node]
                        path = self.path_planning.plan_agent_path(agent, start_node, goal_nodes, path_type='START-GOAL')

                    else:
                        # The agent is not at the start node of the task and not loaded.
                        # Plan the path to the start node, to the goal node and the closest dwelling node.
                        start_node = agent.lastNodeId
                        closest_dwelling_node = self.graph.get_closest_dwelling_node(self.graph.nodes[agent.task['goal_node']]['pos'],
                                                                                     self.graph.dwelling_nodes)
                        goal_nodes = [agent.task['start_node'], agent.task['goal_node'], closest_dwelling_node]
                        path = self.path_planning.plan_agent_path(agent, start_node, goal_nodes, path_type='CURRENT-START-GOAL')

                    if path is not None:
                        # Path found for the agent to move to the goal node.
                        agent.update_agent_state(agent_state="EXECUTING", position=agent.position, theta=agent.theta,
                                                task=agent.task, path=path, orderId=str(self.orderId), order_updateId=agent.order_updateId,
                                                lastNodeId=agent.lastNodeId, lastNodeSequenceId=agent.lastNodeSequenceId, driving=True)
                        self.orderId += 1

                        # Move the agent along the planned path.
                        move_thread = threading.Thread(target=self.move_agent, args=(agent,))
                        move_thread.start()
                    
                    else:
                        # No path found for the agent to move to the goal node.
                        raise ValueError(f"No path found for agent {agent.agentId} to move to the goal node {agent.task['goal_node']}.")

            time.sleep(0.5)
        
        # Restart the path planning manager.
        self.path_planning_manager()

    def initialize_agents_on_dwelling_nodes(self) -> None:
        """
        Initialize the agents by moving them to the dwelling nodes.
        """
        # The number of dwelling nodes must be equal or larger as the number of mobile robots.
        if len(self.graph.dwelling_nodes) < len(self.agents_object.agents):
            raise ValueError(f"The number of dwelling nodes must be equal or larger as the number of mobile robots.")

        agents_not_on_dwelling_nodes = []
        agents_blocking_dwelling_nodes = {}
        dwelling_nodes_blocked = []

        # Check if all agents are on dwelling nodes.
        for agent in self.agents_object.agents:
            # Check if the agent is blocking a dwelling node.
            agent_blocking_dwelling_node = self.check_agent_blocking_dwelling_node(agent)
            
            if agent_blocking_dwelling_node:                
                dwelling_nodes_blocked.append(agent.lastNodeId)
                agents_blocking_dwelling_nodes[agent.agentId] = agent.lastNodeId

            # Check if the agent is exactly positioned on a dwelling node.
            agent_on_dwelling_node = self.check_agent_on_dwelling_node(agent)

            if not agent_on_dwelling_node:
                agents_not_on_dwelling_nodes.append(agent)

            else:
                # Update the agent state.
                if agent.task is None:
                    # Agent is IDLE at the dwelling node. No task assigned.
                    agent.agents_object.logging.info(f"Agent {agent.agentId} is IDLE at node {agent.lastNodeId} at time {round(time.time() - self.start_time, 2)}.")
                    agent.update_agent_state(agent_state="IDLE", position=agent.position, theta=agent.theta,
                                            task=agent.task, path=None, orderId="", order_updateId=0,
                                            lastNodeId=agent.lastNodeId, lastNodeSequenceId=agent.lastNodeSequenceId, driving=False)
                elif agent.task['task_completed']:
                    # Agent is IDLE at the dwelling node. Task completed.
                    agent.agents_object.logging.info(f"Agent {agent.agentId} is IDLE at node {agent.lastNodeId} at time {round(time.time() - self.start_time, 2)}.")
                    agent.update_agent_state(agent_state="IDLE", position=agent.position, theta=agent.theta,
                                            task=agent.task, path=None, orderId="", order_updateId=0,
                                            lastNodeId=agent.lastNodeId, lastNodeSequenceId=agent.lastNodeSequenceId, driving=False)
                else:
                    # Agent is PLANNING at the dwelling node. Task is re-executed.
                    agent.agents_object.logging.info(f"Task {agent.task['task_id']} is re-executed by agent {agent.agentId} at time {round(time.time() - self.start_time, 2)}. Start node: {agent.task['start_node']}. Goal node: {agent.task['goal_node']}.")
                    agent.update_agent_state(agent_state="PLANNING", position=agent.position, theta=agent.theta,
                                            task=agent.task, path=None, orderId="", order_updateId=0,
                                            lastNodeId=agent.lastNodeId, lastNodeSequenceId=agent.lastNodeSequenceId, driving=False)

        # Get the dwelling nodes that are not blocked by agents.
        dwelling_nodes = self.graph.dwelling_nodes.copy()
        for dwelling_node in dwelling_nodes_blocked:
            dwelling_nodes.remove(dwelling_node)

        # Plan the path for the agents to move to available dwelling nodes.
        while agents_not_on_dwelling_nodes != []:
            agents_not_on_dwelling_nodes_copy = agents_not_on_dwelling_nodes.copy()
            for agent in agents_not_on_dwelling_nodes_copy:
                start_node = agent.lastNodeId
                if agent.agentId in agents_blocking_dwelling_nodes:
                    # The agent is blocking a dwelling node but is not exactly positioned on it. Move the agent exactly to the dwelling node.
                    goal_node = agents_blocking_dwelling_nodes[agent.agentId]
                else:
                    # The agent is not blocking a dwelling node. Move the agent to the closest dwelling node.
                    goal_node = self.graph.get_closest_dwelling_node(agent.position, dwelling_nodes)
                path = self.path_planning.plan_agent_path(agent, start_node, [goal_node], path_type='INITIALIZATION')
                if path is not None:
                    # Path found for the agent to move to the dwelling node.
                    agents_not_on_dwelling_nodes.remove(agent)
                    if goal_node in dwelling_nodes:
                        # Dwelling node can only be removed if the agent was not blocking it before.
                        dwelling_nodes.remove(goal_node)

                    agent.update_agent_state(agent_state="EXECUTING", position=agent.position, theta=agent.theta,
                                            task=agent.task, path=path, orderId=str(self.orderId), order_updateId=0,
                                            lastNodeId=agent.lastNodeId, lastNodeSequenceId=agent.lastNodeSequenceId, driving=True)
                    self.orderId += 1

                    # Move the agent along the planned path.
                    move_thread = threading.Thread(target=self.move_agent, args=(agent,))
                    move_thread.start()

    def move_agent(self, agent) -> None:
        """
        Move the agent along the planned path.
        The VDA5050 is used as communication interface to send order messages to the simulated/real agents 
        and receive state messages from the simulated/real agents.
        
        :param agent: The digital twin agent object.
        """
        # Define current node and goal node of the agent and initialize variables.
        current_node = agent.path['nodes'][0]['nodeId']
        goal_node = agent.path['nodes'][-1]['nodeId']
        node_index = 0
        node_resources_monitor = []
        next_node_2_reserved = False
        node_request = None

        # Order variables.
        order_updateId = 0
        current_sequenceId = 1
        new_nodes_released = False
        path_nodes = agent.path['nodes'].copy()
        path_edges = agent.path['edges'].copy()
        path_nodes_complete = path_nodes.copy()

        # Check if the agent is at the start node of the planned path.
        if current_node != agent.lastNodeId:
            raise ValueError(f"Agent {agent.agentId} is not at the current node.")
        
        # Wait for the start time of the planned path.
        delay_time = agent.path_start_time - (time.time() - self.start_time)
        if delay_time > 0:
            agent.agents_object.logging.info(f"Start time of agent {agent.agentId} is delayed by {delay_time} seconds at time {round(time.time() - self.start_time, 2)}.")
            time.sleep(delay_time)
        
        agent.agents_object.logging.info(f"Agent {agent.agentId} is starting the path execution at time {round(time.time() - self.start_time, 2)}.")

        while True:
            if agent.path is not None:
                if current_node == goal_node and len(agent.path['nodes']) == 1:
                    # Move to the initial node of the agent. Initial node is a dwelling node.
                    # Generate and send the order message.
                    path_nodes[0]['released'] = True
                    agent.order_interface.generate_order_message(agent=agent, orderId=agent.orderId, order_updateId=order_updateId,
                                                                nodes=path_nodes, edges=path_edges)
                    
                    # Wait for the real agent to complete the move.
                    agent.next_node_event.wait()
                    agent.next_node_event.clear()

                    agent.agents_object.logging.info(f"Agent {agent.agentId} is at node {current_node} / {self.graph.nodes[current_node]['pos']} at time {round(time.time() - self.start_time, 2)}.")
                    
                    # End the process moving the agent.
                    return

            # Check if the agent has completed the task.
            task_completed = self.check_agent_task_completion(agent, goal_node, path_nodes, path_nodes_complete)
            if task_completed:
                # End the process moving the agent.
                return

            # Get the next node of the agent.
            next_node = agent.path['nodes'][node_index + 1]['nodeId']
            node_index += 1
            new_nodes_released = False

            if next_node != current_node:
                # Move to the next node.
                waiting_move = False

                if not next_node_2_reserved:
                    # Reserve the next node.
                    agent.waiting = True
                    node_reserved, node_resource, _ = self.reserve_node(agent, next_node, node_request, "next")
                    node_resources_monitor.append(node_resource)
                    if not node_reserved:
                        # Node was not reserved. Timeout occurred.
                        return
                    # Release the nodes and edges that are reserved by the agent and update the current sequence ID.
                    path_nodes, path_edges = self.release_nodes_edges(path_nodes, path_edges, current_sequenceId)
                    current_sequenceId += 2
                    agent.waiting = False
                    new_nodes_released = True
                
                elif next_node_2_reserved:
                    # The node is already reserved.
                    next_node_2_reserved = False
                
                # Get the second next node of the agent if it exists and try to reserve it.
                iterator = 1
                while node_index + iterator < len(agent.path['nodes']):
                    next_node_2 = agent.path['nodes'][node_index + iterator]['nodeId']
                    if next_node_2 != next_node:
                        # Try to reserve the second next node for the agent to move to.
                        node_reserved_2, node_resource, node_request = self.reserve_node(agent, next_node_2, None, "second_next")
                        if node_reserved_2:
                            next_node_2_reserved = True
                            node_resources_monitor.append(node_resource)
                            # Release the nodes and edges that are reserved by the agent and update the current sequence ID.
                            path_nodes, path_edges = self.release_nodes_edges(path_nodes, path_edges, current_sequenceId)
                            current_sequenceId += 2
                            new_nodes_released = True
                        break
                    iterator += 1

            else:
                # Waiting move. The node is already reserved.
                waiting_move = True
                self.graph.nodes[next_node]["utilisation_order"].pop(0)
          
            if waiting_move:
                # Waiting moves are implicitly executed by the agents when waiting due to the utilisation order of the nodes or the reservation of the nodes.
                pass

            else:
                # Move the agent from the current node to the next node.
                agent.agents_object.logging.info(f"Agent {agent.agentId} is moving from node {current_node} / {self.graph.nodes[current_node]['pos']} to node {next_node} / {self.graph.nodes[next_node]['pos']} at time {round(time.time() - self.start_time, 2)}.")

                if new_nodes_released:
                    # Generate and send the order message.
                    agent.order_interface.generate_order_message(agent=agent, orderId=agent.orderId, order_updateId=order_updateId,
                                                                nodes=path_nodes, edges=path_edges)
                    
                    # Update the path nodes and edges and the order update ID.
                    path_nodes, path_edges = self.update_nodes_edges(path_nodes, path_edges, current_sequenceId)
                    order_updateId += 1

                # Wait for the real agent to complete the move.
                agent.next_node_event.wait()
                agent.next_node_event.clear()

            # Update the position of the agent.
            current_node = next_node
            agent.agents_object.logging.info(f"Agent {agent.agentId} is at node {next_node} / {self.graph.nodes[next_node]['pos']} at time {round(time.time() - self.start_time, 2)}.")
            
            # Monitor the agent's position. Position is already monitored when move is a waiting move.
            if not waiting_move:
                monitor_thread = threading.Thread(target=agent.monitor_process, args=(agent.lastNodeId, node_resources_monitor[0]))
                monitor_thread.start()
                node_resources_monitor.pop(0)

    def update_nodes_edges(self, path_nodes, path_edges, current_sequenceId) -> tuple[list[dict], list[dict]]:
        """
        Update the list of nodes and edges.
        All nodes and edges with a sequence ID smaller than the current sequence ID are removed.
        
        :param path_nodes: List of nodes.
        :param path_edges: List of edges.
        :param current_sequenceId: Current sequence ID.
        """
        nodes_copy = path_nodes.copy()
        for node in nodes_copy:
            if node["sequenceId"] < current_sequenceId:
                path_nodes.remove(node)
            else:
                break
        
        edges_copy = path_edges.copy()
        for edge in edges_copy:
            if edge["sequenceId"] < current_sequenceId:
                path_edges.remove(edge)
            else:
                break

        return path_nodes, path_edges

    def release_nodes_edges(self, path_nodes, path_edges, current_sequenceId) -> tuple[list[dict], list[dict]]:
        """
        Release the nodes and edges that are reserved by the agent.

        :param path_nodes: List of nodes.
        :param path_edges: List of edges.
        :param current_sequenceId: Current sequence ID.
        :return: Updated list of nodes and edges.
        """
        for node in path_nodes:
            if node["sequenceId"] <= current_sequenceId + 2:
                node["released"] = True
            else:
                break
        for edge in path_edges:
            if edge["sequenceId"] <= current_sequenceId + 2:
                edge["released"] = True
            else:
                break

        return path_nodes, path_edges

    def check_agent_task_completion(self, agent, goal_node, path_nodes, path_nodes_complete) -> bool:
        """
        Check if the task of the agent has been completed.

        :param agent: Digital twin agent object.
        :param goal_node: Goal node of the agent.
        :param path_nodes: List of unreached nodes of the agent's path.
        :param path_nodes_complete: List of nodes of the complete agent's path.
        :return: True if the task has been completed, False otherwise.
        """
        if agent.lastNodeId == goal_node and len(path_nodes) == 1:
            # Agent has reached the goal node. Task has been completed.
            agent.agents_object.logging.info(f"Agent {agent.agentId} has reached the goal node {goal_node} / {self.graph.nodes[goal_node]['pos']} at time {round(time.time() - self.start_time, 2)}.")
                        
            # Update the agent state.
            agent.update_agent_state(agent_state=agent.agent_state, position=agent.position, theta=agent.theta,
                                     task=agent.task, path=agent.path, orderId=agent.orderId, order_updateId=agent.order_updateId,
                                     lastNodeId=agent.lastNodeId, lastNodeSequenceId=agent.lastNodeSequenceId, driving=agent.driving)

            # Remove all reservations of the agent along the path.
            self.path_planning.delete_agent_path_reservations(agent, path_nodes_complete)

            return True

        return False

    def reserve_node(self, agent, next_node, node_request=None, node_order_type="next") -> tuple[bool, object, object]:
        """
        Try to reserve the next node for the agent to move to.
        
        :param agent: Digital twin agent object.
        :param next_node: Next node to reserve on the path of the agent.
        :param node_request: Node reservation request. Default is None.
        :param node_order_type: Type of the node order of the agents path. Can be "next" or "second_next". Default is "next".
        :return: True if the node has been reserved, False otherwise. Also return the node resource and the node reservation request.
        """
        # Get the resource of the next node to reserve it.
        node_resource = self.graph.node_resources[next_node]

        # Maximum duration to wait for the node resource in seconds.
        node_reservation_timeout = self.config_data["node_reservation_timeout"]

        node_reserved = False
        while not node_reserved:
            if node_request is None:
                # Request the resource of the node. The order of utilization of the node must be considered.
                # The order of utilization is defined to avoid deadlocks when movements of agents are delayed.
                if self.graph.nodes[next_node]["utilisation_order"][0][0] == agent.agentId:
                    # The agent is the next agent in the list to utilize the node.
                    node_request = node_resource.request()
                elif node_order_type == "next":
                    # Wait until the agent is the next agent in the list to utilize the node.
                    time_before = time.time() - self.start_time
                    elapsed_time = 0
                    while self.graph.nodes[next_node]["utilisation_order"][0][0] != agent.agentId:
                        time.sleep(0.1)
                        elapsed_time = time.time() - self.start_time - time_before
                        if elapsed_time >= node_reservation_timeout:
                            # The agent has been waiting for too long been the next agent in the list to utilize the node. Timeout occurred.
                            # Set the agent's error flag.
                            agent.error = True
                            self.check_agent_errors()
                            agent.agents_object.logging.error(f"Agent {agent.agentId} has been waiting for node {next_node} / {self.graph.nodes[next_node]['pos']} due to the utilisation order {self.graph.nodes[next_node]['utilisation_order']} longer than {node_reservation_timeout} seconds at time {round(time.time() - self.start_time, 2)}.")
                            return False, None, None

                    if elapsed_time > 0.01:
                        agent.agents_object.logging.info(f"Agent {agent.agentId} has been waiting for node {next_node} / {self.graph.nodes[next_node]['pos']} due to the utilisation order {self.graph.nodes[next_node]['utilisation_order']} for {elapsed_time} seconds at time {round(time.time() - self.start_time, 2)}.")

                    node_request = node_resource.request()
                
                else:
                    # The agent is not the next agent in the list to utilize the node. No waiting is required yet (node_order_type == "second_next").
                    agent.agents_object.logging.info(f"Agent {agent.agentId} requested the node {next_node} / {self.graph.nodes[next_node]['pos']} at time {round(time.time() - self.start_time, 2)} but is not the next agent in the utilisation order.")
                    return False, None, None

            # Wait for the node to be reserved.
            # Node can be aquiered at the same time the resource is released by the previous agent. No waiting or buffer time in between.
            time_before = time.time() - self.start_time
            elapsed_time = 0
            while not node_request.is_set() and node_order_type == "next":
                # Waiting for the node resource is only required when the node is next node in the path of the agent (node_order_type == "next").
                time.sleep(0.1)
                elapsed_time = time.time() - self.start_time - time_before
                if elapsed_time >= node_reservation_timeout:
                    # Node was not reserved. Timeout occurred.
                    # Set the agent's error flag.
                    agent.error = True
                    self.check_agent_errors()
                    agent.agents_object.logging.error(f"Node request {next_node} / {self.graph.nodes[next_node]['pos']} of agent {agent.agentId} took longer than {node_reservation_timeout} seconds at time {round(time.time() - self.start_time, 2)}.")
                    return False, None, None

            if elapsed_time > 0.01:
                node_reserved = True
                agent.agents_object.logging.info(f"Agent {agent.agentId} has been waiting {elapsed_time} seconds for the reservation of node {next_node} / {self.graph.nodes[next_node]['pos']} at time {round(time.time() - self.start_time, 2)}.")
            else:
                node_reserved = True

        if node_request.is_set():
            # Node was reserved. Remove the agent from the list of agents that can utilize the node next.
            agent.agents_object.logging.info(f"Agent {agent.agentId} has reserved node {next_node} / {self.graph.nodes[next_node]['pos']} at time {round(time.time() - self.start_time, 2)}.")
            pop = self.graph.nodes[next_node]["utilisation_order"].pop(0)

            if pop[1] > round(time.time() - self.start_time, 1):
                with self.path_planning.reservation_lock:
                    # Make sure this node cannot be reserved by another agent before the reservation time slot of this agent.
                    self.graph.nodes[next_node].setdefault('reservations', []).append((round(time.time() - self.start_time, 1), pop[1], agent.agentId))

            return True, node_resource, node_request

        else:
            # Node was not reserved. Node is the second next node in the path of the agent (node_order_type == "second_next"), no waiting is required yet.
            agent.agents_object.logging.info(f"Agent {agent.agentId} requested the node {next_node} / {self.graph.nodes[next_node]['pos']} at time {round(time.time() - self.start_time, 2)} but it was not reserved yet.")
            return False, None, node_request

    def check_agent_on_dwelling_node(self, agent) -> bool:
        """
        Check if the agent is exactly positioned on a dwelling node.
        
        :param agent: The digital twin agent object.
        :return: True if the agent is exactly positioned on a dwelling node, False otherwise.
        """
        for dwelling_node in self.graph.dwelling_nodes:
            if math.dist(agent.position, self.graph.nodes[dwelling_node]['pos']) == 0:
                return True
        return False

    def check_agent_blocking_dwelling_node(self, agent) -> bool:
        """
        Check if the agent is blocking a dwelling node.
        
        :param agent: The digital twin agent object.
        :return: True if the agent is blocking a dwelling node, False otherwise.
        """
        for dwelling_node in self.graph.dwelling_nodes:
            if agent.lastNodeId == dwelling_node:
                return True
        return False


    def check_agent_errors(self) -> None:
        """
        Check if all agents have errors. In this case a deadlock occurred.
        Handle the deadlock by reinitializing the agents on the dwelling nodes.
        """
        agents_with_errors = []
        for agent in self.agents_object.agents:
            if agent.error:
                agents_with_errors.append(agent)
        
        if len(agents_with_errors) == len(self.agents_object.agents):
            # Agents are in a deadlock. Reinitialize the agents on the dwelling nodes.
            for agent in self.agents_object.agents:
                # Update the agent state.
                agent.update_agent_state(agent_state="INITIALIZING", position=agent.position, theta=agent.theta,
                                        task=agent.task, path=None, orderId=str(self.orderId), order_updateId=agent.order_updateId,
                                        lastNodeId=agent.lastNodeId, lastNodeSequenceId=agent.lastNodeSequenceId, driving=True)
                self.orderId += 1
            
            # Delete all reservations of the agents and release the resources.
            self.path_planning.delete_all_reservations()
            self.graph.reinitialize_resources()

            # Reset the error flags of the agents.
            for agent in self.agents_object.agents:
                agent.error = False
                agent.waiting = False

            # Restart the path planning manager.
            self.agents_initialized = False


class AStarNode():
    """
    Class represents a node in the A* algorithm.
    """
    def __init__(self, parent=None, node=None, position=None, g=0, h=float("inf"), moved_time=0):
        """
        Initialize the AStarNode object.
        
        :param parent: Previous node ID in the path. Default is None.
        :param node: Node ID. Default is None.
        :param position: Position of the node. Default is None.
        :param g: Movement cost from the start node to the current node. Default is 0.
        :param h: Heuristic cost from the current node to the end node. Default is infinity.
        :param moved_time: Total movement time from the start node to the current node. Waiting time is included. Default is 0.
        """
        self.parent = parent
        self.node = node
        self.position = position
        self.g = g
        self.h = h
        self.f = g + h
        self.moved_time = moved_time
    
    def update_node(self, parent, g, moved_time):
        """
        Update the node with new values.

        :param parent: Previous node ID in the path.
        :param g: Movement cost from the start node to the current node.
        :param moved_time: Total movement time from the start node to the current node. Waiting time is included.
        """
        self.parent = parent
        self.g = g
        self.f = self.g + self.h
        self.moved_time = moved_time


class PathPlanning():
    """
    Class for managing the path planning of the agents.
    """
    def __init__(self, config_data, graph):
        """
        Initialize the PathPlanning object.

        :param config_data: The configuration data.
        :param graph: The graph object.
        """
        self.start_time = 0
        self.config_data = config_data
        self.graph = graph
        self.path_planning_time = 1
        self.waiting_move_time = 1
        self.buffer_time = 0.5
        self.path_start_time_delay = 2
        self.reservation_lock = threading.Lock()

    def plan_agent_path(self, agent, start_node, goal_nodes, path_type) -> dict[list, list]:
        """
        Plan the path for the agent to follow along the graph.

        :param agent: The digital twin agent object.
        :param start_node: Start node ID of the path.
        :param goal_nodes: List of goal node IDs of the path. Can be a single goal node or multiple goal nodes.
        :param path_type: Type of the path. Can be INITIALIZATION, CURREN-START-GOAL or START-GOAL.
        :return: Dictionary containing the nodes and edges of the path for the agent to follow. None if no path is found.
        """
        complete_path_found = False
        original_start_node = start_node
        planning_start_time = round(time.time() - self.start_time, 1)

        while not complete_path_found:
            path_nodes_dicts = []
            path_edges_dicts = []
            break_condition = False
            for goal_index, goal_node in enumerate(goal_nodes):
                # Multiple path segments are planned individually.
                # Path to the start node, path to the goal node, path to the dwelling node.
                collision_free_shortest_path = False
                if break_condition:
                    # Replan the whole path.
                    break

                if goal_index != 0:
                    start_node = goal_nodes[goal_index-1]
                    start_rotation = math.atan2(path_positions[-1][1] - path_positions[-2][1], path_positions[-1][0] - path_positions[-2][0])
                                                

                    # Start time of the planned path execution.
                    start_time = round(required_reservations[-1][1], 2)

                    # If goal node is a dwelling node, check if the node can be reserved for the agent infinitely.
                    if self.graph.nodes[goal_node]["type"] == "dwelling":
                        # Get all dwelling nodes.
                        dwelling_nodes = self.graph.dwelling_nodes.copy()

                        # Get the closest dwelling node to the goal node.
                        goal_node = self.graph.get_closest_dwelling_node(self.graph.nodes[agent.task['goal_node']]['pos'],
                                                                         dwelling_nodes)

                        while not self.check_reservation(self.graph.nodes[goal_node].get('reservations', []),
                                                         start_time, float('inf')):
                            # Dwelling node is reserved. Get the next closest dwelling node.
                            dwelling_nodes.remove(goal_node)
                            goal_node = self.graph.get_closest_dwelling_node(self.graph.nodes[agent.task['goal_node']]['pos'],
                                                                             dwelling_nodes)
                
                else:
                    # Start time of the planned path execution.
                    start_time = round(time.time() - self.start_time + self.path_planning_time + agent.path_start_time_delay, 1)
                    start_node = original_start_node
                    start_rotation = agent.theta

                    # The start node is a dwelling node, which is reserved for the agent infinitely. Remove the reservation.
                    self.release_start_node(start_node)

                    if not self.check_reservation(self.graph.nodes[start_node].get('reservations', []),
                                                  planning_start_time, start_time):
                        # Collision with other agents at the start node.
                        agent.agents_object.logging.info(f"Collision of agent {agent.agentId} with other agent at the start node {start_node} / {self.graph.nodes[start_node]['pos']}. Reservation time slot: ({planning_start_time}, {start_time}). Replanning the whole path.")
                        planning_start_time = round(planning_start_time + self.path_start_time_delay, 1)
                        agent.path_start_time_delay += self.path_start_time_delay
                        break_condition = True
                        break

                agent.agents_object.logging.info(f"Agent {agent.agentId} is planning the path segment {goal_index + 1} from node {start_node} to node {goal_node} with start time {start_time} at time {round(time.time() - self.start_time, 2)}.")

                # Check if the agent is picking up or dropping off a load at the station nodes.
                if path_type == 'START-GOAL' and goal_index == 0:
                    pick_drop = True
                elif path_type == 'CURRENT-START-GOAL' and goal_index == 1:
                    pick_drop = True
                else:
                    pick_drop = False
                
                if pick_drop:
                    # Check the reservation of the start node for the time required to pick up the load.
                    if not self.check_reservation(self.graph.nodes[start_node].get('reservations', []),
                                                  start_time, start_time + self.config_data["process_time_loading_unloading"]):
                        # Collision with other agents at the start node. Replan the whole path.
                        agent.agents_object.logging.info(f"Collision of agent {agent.agentId} with other agent at the start node {start_node} / {self.graph.nodes[start_node]['pos']} while pick up at time {round(time.time() - self.start_time, 2)}. Reservation time slot: ({start_time}, {start_time + self.config_data['process_time_loading_unloading']}). Replanning the whole path.")
                        agent.path_start_time_delay += self.path_start_time_delay
                        break_condition = True
                        break
                    else:
                        start_time = round(start_time + self.config_data["process_time_loading_unloading"], 1)

                # Get the k shortest paths from the start node to the end node using Yen's K-Shortest Paths Algorithm.
                paths = self.yen_k_shortest_paths(agent, start_time, start_node, goal_node, pick_drop, start_rotation, k=5)

                if paths is None:
                    # No path found connecting the start and end node.
                    raise ValueError(f"No path segment {goal_index + 1} found connecting the start {start_node} and goal node {goal_node}.")

                # Shortest path length.
                shortest_path_length = paths[0][3][0]

                if not paths:
                    # No path found connecting the start and end node.
                    raise ValueError(f"No path segment {goal_index + 1} found connecting the start node {start_node} and goal node {goal_node}.")

                # Check the paths for collisions with other agents.
                for path in paths:
                    path_nodes, path_positions, required_reservations, path_distances_to_end = path

                    if path_distances_to_end[0] > shortest_path_length * 1.25:
                        # Path is too long. Continue with the next path.
                        continue

                    if pick_drop:
                        # Add the process time to the path.
                        required_reservations, break_condition = self.add_process_time(agent, required_reservations, goal_node)
                        if break_condition:
                            # Replan the whole path.
                            break

                    if self.check_path_reservations(path_nodes, required_reservations):
                        # Collison free path found.
                        collision_free_shortest_path = True
                        agent.agents_object.logging.info(f"Collision free path segment {goal_index + 1} found for agent {agent.agentId} using Yen's K-shortest paths algorithm.")
                        break
                
                if break_condition:
                    # Replan the whole path.
                    break
                
                if not collision_free_shortest_path and path_nodes is not None:
                    # No collision free path found. Select the shortest path and replan the path.
                    path_nodes, path_positions, required_reservations, path_distances_to_end = paths[0]

                    # Plan a new path for the agent.
                    path_nodes_new, path_positions, required_reservations, path_distances_to_end_new \
                        = self.astar_search(agent, start_time, start_node, goal_node, pick_drop, start_rotation, path_nodes, path_distances_to_end)

                    waiting_time = 0
                    while path_nodes_new is None:
                        # No path found connecting the start and end node considering reservations.
                        # Wait for a certain interval before replanning the path.
                        waiting_time += self.waiting_move_time
                        start_time += self.waiting_move_time
                        agent.agents_object.logging.info(f"No path segment {goal_index + 1} found. Agent {agent.agentId} is waiting for {self.waiting_move_time} seconds before replanning the path segment. New start time: {start_time}.")

                        # Check the reservation of the start node.
                        if not self.check_reservation(self.graph.nodes[start_node].get('reservations', []),
                                                      start_time - waiting_time, start_time):
                            # Collision with other agents at the start node. Replan the whole path.
                            agent.agents_object.logging.info(f"Collision of agent {agent.agentId} with other agent at the start node {start_node} / {self.graph.nodes[start_node]['pos']}. Reservation time slot: ({start_time - waiting_time}, {start_time}). Replanning the whole path.")
                            agent.path_start_time_delay += self.path_start_time_delay
                            break_condition = True
                            break

                        path_nodes_new, path_positions, required_reservations, path_distances_to_end_new \
                            = self.astar_search(agent, start_time, start_node, goal_node, pick_drop, start_rotation, path_nodes, path_distances_to_end)

                        if path_nodes_new is None and path_type == 'INITIALIZATION':
                            # When initializing the agents on the dwelling nodes, all paths can be blocked by other agents. Move the other agents first.
                            # Reserve the initial node of the agent again for a infinite time.
                            self.reserve_path(agent, [agent.lastNodeId], [(self.start_time, float('inf'))])
                            agent.path_start_time_delay += self.path_start_time_delay
                            return None

                    if pick_drop and not break_condition:
                        # Add the process time to the path.
                        required_reservations, break_condition = self.add_process_time(agent, required_reservations, goal_node)
                        if break_condition:
                            # Replan the whole path.
                            break

                    if break_condition:
                        # Replan the whole path.
                        break

                    path_nodes = path_nodes_new
                    path_distances_to_end = path_distances_to_end_new

                # Define the sequence IDs for the nodes and edges of the path.
                if path_nodes_dicts != []:
                    sequence_id_node_add = path_nodes_dicts[-1]["sequenceId"]
                    sequence_id_edge_add = path_edges_dicts[-1]["sequenceId"] + 2
                else:
                    sequence_id_node_add = 1
                    sequence_id_edge_add = 2

                # Define the list of nodes of the path.
                for index, node in enumerate(path_nodes):
                    if index == 0 and path_nodes_dicts != []:
                        # The start node of the new path segment is the same as the end node of the previous path segment.
                        # The node is not added to the path list twice, but the reservation time is updated.
                        path_nodes_dicts[-1]["reservationTime"] = (path_nodes_dicts[-1]["reservationTime"][0], required_reservations[0][1])
                        continue
                    path_nodes_dicts.append({"nodeId": node, "sequenceId": index*2 + sequence_id_node_add, "released": False,
                                            "x": path_positions[index][0], "y": path_positions[index][1],
                                            "reservationTime": required_reservations[index], "actions": []})

                if goal_index == 0:
                    # Reserve the start node from planning start time.
                    agent.path_start_time = path_nodes_dicts[0]["reservationTime"][0]
                    path_nodes_dicts[0]["reservationTime"] = (planning_start_time, path_nodes_dicts[0]["reservationTime"][1])

                # Add pick and drop actions at the station nodes.
                if path_type == 'START-GOAL':
                    if self.graph.nodes[path_nodes[0]]["type"] == "station":
                        # Add pick action at the station node.
                        uuid_str = str(uuid.uuid4())
                        agent.pick_action_uuid = uuid_str
                        path_nodes_dicts[0]["actions"].append({"actionType": "pick", "actionId": uuid_str, "blockingType": "HARD"})
                    if self.graph.nodes[path_nodes[-1]]["type"] == "station":
                        # Add drop action at the station node.
                        uuid_str = str(uuid.uuid4())
                        agent.drop_action_uuid = uuid_str
                        path_nodes_dicts[-1]["actions"].append({"actionType": "drop", "actionId": uuid_str, "blockingType": "HARD"})

                elif path_type == 'CURRENT-START-GOAL':
                    if self.graph.nodes[path_nodes[-1]]["type"] == "station" and goal_index == 0:
                        # Add pick action at the station node.
                        uuid_str = str(uuid.uuid4())
                        agent.pick_action_uuid = uuid_str
                        path_nodes_dicts[-1]["actions"].append({"actionType": "pick", "actionId": uuid_str, "blockingType": "HARD"})
                    elif self.graph.nodes[path_nodes[-1]]["type"] == "station" and goal_index != 0:
                        # Add drop action at the station node.
                        uuid_str = str(uuid.uuid4())
                        agent.drop_action_uuid = uuid_str
                        path_nodes_dicts[-1]["actions"].append({"actionType": "drop", "actionId": uuid_str, "blockingType": "HARD"})

                # Get edges of the path and define the list of edges of the path.
                path_edges = [self.graph.get_connected_edge(path_nodes[i], path_nodes[i+1]) for i in range(len(path_nodes)-1)]
                for index, edge in enumerate(path_edges):
                    path_edges_dicts.append({"edgeId": edge, "sequenceId": index*2 + sequence_id_edge_add, "released": False,
                                            "startNodeId":self.graph.edges[edge]['startNodeId'],
                                            "endNodeId": self.graph.edges[edge]['endNodeId']})

                agent.agents_object.logging.info(f"Path segment {goal_index + 1} found for agent {agent.agentId} at time {round(time.time() - self.start_time, 2)}: {path_nodes}, {path_positions}, {required_reservations}.")

            if goal_index == len(goal_nodes) - 1 and not break_condition:
                # All path segments are planned successfully. Define the complete path for the agent.
                agent_path = {'nodes': path_nodes_dicts, 'edges': path_edges_dicts}
                path_nodes = [node["nodeId"] for node in path_nodes_dicts]
                required_reservations = [(node["reservationTime"][0], node["reservationTime"][1]) for node in path_nodes_dicts]
                if len(required_reservations) > 1 and required_reservations[1][0] < time.time() - self.start_time:
                    # Start time of the path is in the past. Replanning the whole path.
                    agent.agents_object.logging.info(f"Start time of the path of agent {agent.agentId} is in the past by {round(time.time() - self.start_time - required_reservations[0][0], 2)} seconds. Replanning the whole path.")
                    agent.path_start_time_delay += (time.time() - self.start_time) - required_reservations[0][0]
                    break_condition = True

                if not break_condition:
                    with self.reservation_lock:
                        # Check the reservations of the complete path again before reserving the complete path.
                        if self.check_path_reservations(path_nodes, required_reservations):
                            # Reserve the complete path for the agent.
                            self.reserve_path(agent, path_nodes, required_reservations)
                            complete_path_found = True
                            agent.agents_object.logging.info(f"Complete path found for agent {agent.agentId} at time {round(time.time() - self.start_time, 2)}: {agent_path['nodes']}.")
                        else:
                            # Collision with other agents. Replan the whole path.
                            agent.agents_object.logging.info(f"Collision of agent {agent.agentId} with other agent along the complete path. Replanning the whole path.")
                            agent.path_start_time_delay += self.path_start_time_delay
                            break_condition = True

        # Reset the path start time delay of the agent.
        agent.path_start_time_delay = 0
    
        return agent_path

    def astar_search(self, agent, start_time, start_node, goal_node, pick_drop, start_rotation, path=None, path_distances_to_end=None) -> tuple[list, list, list, list]:
        """
        Function searches for the fastest path from the start node to the end node for the agent using the A* algorithm.
        The previously planned paths of other agents are considered when planning the path to avoid collisions and deadlocks.

        :param agent: The digital twin agent object.
        :param start_time: Start time of the path planning.
        :param start_node: Start node ID of the agent.
        :param goal_node: Goal node ID of the agent.
        :param pick_drop: True if the agent is picking up or dropping off a load at the station node, False otherwise.
        :param start_rotation: Start rotation of the agent.
        :param path: List of nodes representing the path for the agent. When path is given, timing is used. Default is None.
        :param path_distances_to_end: List of distances to the end of the path (when it is given) for the nodes in the path. Default is None.
        :return: Lists of nodes and positions representing the path, required reservations for the path,
                 distances to the end of the path for the nodes in the path.
        """
        # Movement time to the start node is not considered if the agent is not exactly positioned on the start node.
        # Get the position of the start and goal node.
        start_position = tuple(self.graph.nodes[start_node]["pos"])

        # Initialize the current time.
        current_time = round(start_time, 1)

        # Initialize the node dictionary.
        node_objects = {}
        
        # Create the start node object.
        start_node_obj = AStarNode(None, start_node, start_position, g=0,
                                   h=self.get_h(start_node, goal_node, path, path_distances_to_end))
        node_objects[(start_position, current_time)] = start_node_obj

        # Initialize open and closed list.
        open_list = SortedList(key=lambda node: (node.f, node.h, node.node))
        closed_list = []

        # Get the reserved nodes.
        reserved_nodes = self.get_infinite_reserved_nodes(start_time, self.graph.nodes)
        
        # Add the start node to the open list. A priority queue is used.
        # The f value is the first priority, the h value is the second priority and the node ID is the third priority.
        # The node ID is only used to break ties between nodes with the same f and h values.
        open_list.add(start_node_obj)

        # Search for the shortest path to the end node.
        while open_list:
            # Get the node from the open list with the highest priority.
            current_node_obj = open_list.pop(0)

            # Update the current time. Only when timing is used.
            if path is not None:
                current_time = round(start_time + current_node_obj.moved_time, 1)

            # Check if the current node is in the closed list.
            if (current_node_obj.position, current_time) in closed_list:
                continue
            
            # Add the current node to the closed list.
            closed_list.append((current_node_obj.position, current_time))

            if len(closed_list) % 100 == 0:
                # Pause the path planning for a short time to avoid high CPU usage.
                time.sleep(0.2)

            # Check if the current node is the end node.
            if current_node_obj.node == goal_node:
                # Reconstruct the path from the end node to the start node.
                # agent.agents_object.logging.info(f"Length of the closed list: {len(closed_list)}, start time: {start_time}.")
                path_nodes, path_positions, required_reservations, path_distances_to_end \
                    = self.reconstruct_path(current_node_obj, start_time)

                # Return the reversed path.
                return path_nodes[::-1], path_positions[::-1], required_reservations[::-1], path_distances_to_end[::-1]

            # Get the connected nodes of the current node.
            neighbors = self.graph.get_connected_nodes(current_node_obj.node)
            if path is not None:
                # Waiting move is allowed when timing is used.
                neighbors.append(current_node_obj.node)

            for connected_node in neighbors:
                # Agents can use station nodes only as start and goal nodes.
                if self.graph.nodes[connected_node]["type"] == "station" and connected_node != goal_node:
                    continue

                if connected_node in reserved_nodes:
                    # Node is reserved for infinite time.
                    continue

                # Get the position of the connected node.
                connected_position = self.graph.nodes[connected_node]["pos"]

                # Calculate the movement length and rotation to the connected node.
                move_length = self.get_distance(current_node_obj.position, connected_position)
                if current_node_obj.parent is not None:
                    move_rotation = self.get_rotation(previous_position=current_node_obj.parent.position,
                                                      current_position=current_node_obj.position,
                                                      next_position=connected_position)
                else:
                    move_rotation = self.get_rotation(previous_position=None, current_position=current_node_obj.position,
                                                      next_position=connected_position, start_orientation=start_rotation)

                # Calculate the movement time and cost to the connected node.
                if move_length == 0.0:
                    # Waiting move. Only possible when timing is used. Time in seconds.
                    move_time = self.waiting_move_time
                    move_cost = 0
                else:
                    # Move to the connected node. Time in seconds.
                    move_time = round(move_length / agent.velocity + move_rotation / agent.rotation_velocity, 1)
                    move_cost = move_length / agent.velocity + move_rotation / agent.rotation_velocity

                # When timing is used, several checks are performed.
                if path is not None:
                    # Check the reservation of the current node and the connected node.
                    # The current node must be reserved as long the agent uses the connected edge to move to the connected node.
                    if self.graph.nodes[connected_node]["type"] == "station" and connected_node == goal_node:
                        # The current node is the goal node, which is a station node.
                        # The agent must be able to reserve the station node long enough to leave if again without collision.
                        max_time_to_leave_node = self.calculate_max_time_to_leave_node(agent, current_node_obj, connected_node)
                    else:
                        max_time_to_leave_node = 0

                    if pick_drop and self.graph.nodes[connected_node]["type"] == "station" and connected_node == goal_node:
                        # Consider the process time for drop off at the station node.
                        max_time_to_leave_node += self.config_data["process_time_loading_unloading"]

                    if not self.check_reservation(self.graph.nodes[connected_node].get('reservations', []),
                                                  current_time - self.buffer_time, current_time + move_time + max_time_to_leave_node):
                        # Connected node is reserved.
                        continue

                    if not self.check_reservation(self.graph.nodes[current_node_obj.node].get('reservations', []),
                                                  current_time, current_time + move_time + self.buffer_time):
                        # Current node is reserved.
                        continue

                    # Calculate the time the agent arrives at the connected node.
                    connected_time = round(current_time + move_time, 1)
                    
                    # Cost for waiting move.
                    if connected_node == current_node_obj.node:
                        # Waiting move is prioritized compared to moving around.
                        # Cost for waiting is smaller than moving around for the same time.
                        move_cost = self.calculate_waiting_move_cost(current_node_obj, connected_node)
                    
                    # Cost for returning to the previous node.
                    elif current_node_obj.parent is not None and current_node_obj.parent.node == connected_node:
                        # Cost is increased to prevent the agent from moving back and forth between two nodes.
                        move_cost = move_cost * 2

                else:
                    connected_time = current_time

                # Check if the connected node is in the closed list.
                if (connected_position, connected_time) in closed_list:
                    continue

                if (connected_position, connected_time) not in node_objects.keys():
                    # Create a new node object.
                    new_node_obj = AStarNode(current_node_obj, connected_node, connected_position,
                                             current_node_obj.g + move_cost,
                                             self.get_h(connected_node, goal_node, path, path_distances_to_end),
                                             current_node_obj.moved_time + move_time)
                    node_objects[(connected_position, connected_time)] = new_node_obj

                else:
                    if node_objects[(connected_position, connected_time)] in open_list:
                        if current_node_obj.g + move_cost >= node_objects[(connected_position, connected_time)].g:
                            # The current path to the connected node is not better than the previously found path.
                            continue
                        else:
                            # The current path to the connected node is better than the previously found path. Update the node.
                            open_list.remove(node_objects[(connected_position, connected_time)])
                            node_objects[(connected_position,
                                          connected_time)].update_node(current_node_obj,
                                                                       current_node_obj.g + move_cost,
                                                                       current_node_obj.moved_time + move_time)

                # Add the connected node to the open list.
                open_list.add(node_objects[(connected_position, connected_time)])

        # No path found.
        return None, None, None, None
    
    def yen_k_shortest_paths(self, agent, start_time, start_node, goal_node, pick_drop, start_rotation, k=5):
        """
        Yen's K-Shortest Paths Algorithm to find the k shortest paths from start_node to goal_node.

        :param agent: The digital twin agent object.
        :param start_time: Start time of the path planning.
        :param start_node: Start node ID of the agent.
        :param goal_node: Goal node ID of the agent.
        :param pick_drop: True if the agent is picking up or dropping off a load at the station node, False otherwise.
        :param start_rotation: Start rotation of the agent.
        :param k: Number of shortest paths to find. Default is 5.
        :return: List of k shortest paths, each path is a tuple of (path_nodes, path_positions, required_reservations, path_distances_to_end).
        """
        initial_start_time = round(start_time, 1)
        initial_start_rotation = start_rotation
        # Find the shortest path using A* algorithm.
        path_nodes, path_positions, required_reservations, path_distances_to_end \
                = self.astar_search(agent, start_time, start_node, goal_node, pick_drop, start_rotation)
        if path_nodes is None:
            return None

        # Initialize the list of k shortest paths.
        k_shortest_paths = [(path_nodes, path_positions, required_reservations, path_distances_to_end)]
        potential_paths = []

        for _ in range(1, k):
            for j in range(len(k_shortest_paths[-1][0]) - 1):
                # Spur node is the j-th node in the last found path.
                spur_node = k_shortest_paths[-1][0][j]
                root_path_nodes = k_shortest_paths[-1][0][:j + 1]

                if j > 0:
                    spur_start_time = k_shortest_paths[-1][2][j-1][1]
                    start_rotation = math.atan2(k_shortest_paths[-1][1][j][1] - k_shortest_paths[-1][1][j-1][1],
                                                k_shortest_paths[-1][1][j][0] - k_shortest_paths[-1][1][j-1][0])
                else:
                    spur_start_time = initial_start_time
                    start_rotation = initial_start_rotation

                # Remove edges that are part of the previous shortest paths.
                removed_edges = []
                for path in k_shortest_paths:
                    if len(path[0]) > j and path[0][:j + 1] == root_path_nodes:
                        edge = self.graph.get_edge(path[0][j], path[0][j + 1])
                        if edge is not None:
                            self.graph.remove_edge(edge)
                            removed_edges.append(edge)

                # Calculate the spur path from the spur node to the goal node.
                spur_path_nodes, spur_path_positions, spur_required_reservations, spur_path_distances_to_end \
                    = self.astar_search(agent, spur_start_time, spur_node, goal_node, pick_drop, start_rotation)
                if spur_path_nodes is not None:
                    # Combine the root path and the spur path.
                    total_path_nodes = k_shortest_paths[-1][0][:j] + spur_path_nodes
                    total_path_positions = k_shortest_paths[-1][1][:j] + spur_path_positions
                    
                    if j > 1:
                        spur_required_reservations[0] = (k_shortest_paths[-1][2][j-2][1], spur_required_reservations[0][1])
                    else:
                        spur_required_reservations[0] = (initial_start_time, spur_required_reservations[0][1])
                    total_required_reservations = k_shortest_paths[-1][2][:j] + spur_required_reservations

                    total_path_distances_to_end = []
                    for index in range(len(total_path_nodes)):
                        distance_to_end = sum(self.get_distance(total_path_positions[i], total_path_positions[i + 1]) for i in range(index, len(total_path_positions) - 1))
                        total_path_distances_to_end.append(distance_to_end)
                    
                    # Check if nodes are visited twice.
                    nodes_visited_twice = False
                    visited_nodes = set()
                    for node in total_path_nodes:
                        if node in visited_nodes:
                            nodes_visited_twice = True
                            break
                        visited_nodes.add(node)

                    # Check if the total path is already in the list of k shortest paths or potential paths. Also check that nodes are not visited twice.
                    if total_path_nodes not in [path[0] for path in k_shortest_paths] and total_path_nodes not in [path[0] for path in potential_paths] \
                            and not nodes_visited_twice:
                        potential_paths.append((total_path_nodes, total_path_positions, total_required_reservations, total_path_distances_to_end))

                # Add the removed edges back to the graph.
                for edge in removed_edges:
                    self.graph.add_edge(edge)

            if not potential_paths:
                break

            # Sort the potential paths by their total cost and add the best one to the list of k shortest paths.
            potential_paths.sort(key=lambda x: sum(self.get_distance(x[1][i], x[1][i + 1]) for i in range(len(x[1]) - 1)))
            k_shortest_paths.append(potential_paths.pop(0))

        return k_shortest_paths

    def get_h(self, current_node, goal_node, path, path_distances_to_end) -> float:
        """
        Calculates the heuristic cost from the current node to the end node.

        :param current_node: The current node.
        :param goal_node: The end node.
        :param path: List of nodes representing the path.
        :param path_distances_to_end: List of distances to the end of the path for the nodes in the path.
        :return: Heuristic cost from the current node to the end node.
        """
        if path is None:
            # No path is given. Use the Euclidean distance as heuristic.
            h_value = self.get_distance(self.graph.nodes[current_node]["pos"],
                                        self.graph.nodes[goal_node]["pos"])

        else:
            # Path is given. Use the distance to the end of the path as heuristic.
            if current_node in path:
                # Node is in the path. Use the distance to the end of the path as heuristic.
                h_value = path_distances_to_end[path.index(current_node)]

            else:
                # Node is not in the path. Calculate heuristic based on closest node of path and distance to path.
                closest_path_node = self.get_closest_path_node(self.graph.nodes[current_node]["pos"], path)
                closest_path_node_index = path.index(closest_path_node)
                path_distance_to_end = path_distances_to_end[closest_path_node_index]
                distance_to_path = self.get_distance(self.graph.nodes[current_node]["pos"],
                                                     self.graph.nodes[closest_path_node]["pos"])
                
                # Use a factor to adjust the heuristic when the agent is not on the path.
                factor = 0.25
                h_value = path_distance_to_end + distance_to_path * factor

        return h_value

    def get_distance(self, start_node, goal_node) -> float:
        """
        Calculates the distance between two nodes.

        :param start_node: Position of the start node.
        :param goal_node: Position of the end node.
        :return: Distance between the two nodes.
        """
        distance = math.dist(start_node, goal_node)
        return distance

    def get_rotation(self, previous_position, current_position, next_position, start_orientation=0) -> float:
        """
        Calculates the rotation between the current edge and the connected edge.

        :param previous_position: Position of the previous node.
        :param current_position: Position of the current node.
        :param next_position: Position of the next node.
        :param start_orientation: Start orientation of the agent. Is required when no previous node exists. Default is 0.
        :return: Rotation in radians between the current orientation and the orientation of the connected edge.
        """
        if previous_position is None:
            # No previous node. Take the start orientation as the orientation of the current edge.
            current_edge_orientation = start_orientation
        else:
            # Calculate the orientation of the current edge.
            current_edge_orientation = math.atan2(current_position[1] - previous_position[1],
                                                  current_position[0] - previous_position[0])
        
        if current_edge_orientation > math.pi or current_edge_orientation < -math.pi:
            # Normalize the orientation to the range [-pi, pi].
            current_edge_orientation = (current_edge_orientation + math.pi) % (2 * math.pi) - math.pi
        
        # Calculate the orientation of the next edge.
        next_edge_orientation = math.atan2(next_position[1] - current_position[1],
                                           next_position[0] - current_position[0])
        
        # Calculate the rotation between the current edge and the connected edge.
        rotation = abs(next_edge_orientation - current_edge_orientation)

        # Normalize the rotation to the range [0, pi].
        if rotation > math.pi:
            rotation = 2 * math.pi - rotation
        
        return rotation

    def get_closest_path_node(self, position, path) -> int:
        """
        Get the closest node in the path to the position.

        :param position: Position of the node.
        :param path: List of nodes representing the path.
        :return: Node ID of the closest node in the path.
        """
        distances = [self.get_distance(position, self.graph.nodes[path_node]['pos']) for path_node in path]
        closest_node = path[distances.index(min(distances))]
        return closest_node

    def calculate_waiting_move_cost(self, current_node_obj, connected_node) -> float:
        """
        Calculate the cost of the waiting move. Reduction factors are used to reduce the waiting move cost.

        :param current_node_obj: Current node object.
        :param connected_node: Node ID of the connected node.
        :return: Cost of the waiting move.
        """
        # Adjust the reduction factor. Agents should not move back and forth between two nodes but prioritize waiting.
        reduction_factors = [0.95, 0.9, 0.8, 0.1, 0.01]
        first_reduction_factor = reduction_factors.pop(0)
        move_length = self.waiting_move_time * first_reduction_factor
        parent = current_node_obj.parent

        for i in range(len(reduction_factors)):
            if parent is not None and parent.node == connected_node:
                move_length = self.waiting_move_time * reduction_factors[i]
                parent = parent.parent
            else:
                break

        return move_length
    
    def calculate_max_time_to_leave_node(self, agent, current_node_obj, connected_node) -> float:
        """
        Function calculates the maximum time to leave the connected node based on the maximum movement time to its neighbor nodes.

        :param agent: The digital twin agent object.
        :param current_node_obj: Current node object.
        :param connected_node: Node ID of the connected node.
        :return: Maximum time to leave the node.
        """
        connected_position = self.graph.nodes[connected_node]["pos"]
        neighbors = self.graph.get_connected_nodes(connected_node)
        max_time_to_leave_node = 0
        for neighbor in neighbors:
            neighbor_position = self.graph.nodes[neighbor]["pos"]
            move_length = self.get_distance(connected_position, neighbor_position)
            move_rotation = 3.14  # Worst case rotation in radians.
            move_time = move_length / agent.velocity + move_rotation / agent.rotation_velocity
            if move_time > max_time_to_leave_node:
                max_time_to_leave_node = move_time
        return round(max_time_to_leave_node, 1)

    def check_connection_to_station_node(self, node) -> bool:
        """
        Check if the node is connected to a station node.

        :param node: Node ID of the node to check.
        :return: Boolean value indicating if the node is connected to a station node.
        """
        neighbors = self.graph.get_connected_nodes(node)
        for neighbor in neighbors:
            if self.graph.nodes[neighbor]["type"] == "station":
                return True
        return False

    def add_process_time(self, agent, required_reservations, goal_node) -> tuple[list, bool]:
        """
        Add the process time to the path when the agent picks up or drops off a load at a station node.
        
        :param agent: Agent object.
        :param required_reservations: List of time slots the agent is using the nodes and connected edges.
        :param goal_node: Goal node ID of the agent.
        :return: Updated list of required reservations, boolean value indicating if the path should be replanned.
        """
        # Check the reservation of the goal node for the time required to drop off the load.
        if not self.check_reservation(self.graph.nodes[goal_node].get('reservations', []),
                                        required_reservations[-1][1], required_reservations[-1][1] + self.config_data["process_time_loading_unloading"]):
            # Collision with other agents at the goal node. Replan the whole path.
            agent.agents_object.logging.info(f"Collision of agent {agent.agentId} with other agent at the goal node {goal_node} / {self.graph.nodes[goal_node]['pos']} while drop off at time {round(time.time() - self.start_time, 2)}. Reservation time slot: ({required_reservations[-1][1]}, {required_reservations[-1][1] + self.config_data['process_time_loading_unloading']}). Replanning the whole path.")
            agent.path_start_time_delay += self.path_start_time_delay
            break_condition = True
        else:
            required_reservations[0] = (round(required_reservations[0][0] - self.config_data["process_time_loading_unloading"], 1),
                                        required_reservations[0][1])
            required_reservations[-1] = (required_reservations[-1][0],
                                            round(required_reservations[-1][1] + self.config_data["process_time_loading_unloading"], 1))
            break_condition = False
        return required_reservations, break_condition

    def reconstruct_path(self, current_node_obj, start_time) -> tuple[list, list, list, list]:
        """
        Reconstruct the path from the goal node to the start node.

        :param current_node_obj: Current node object.
        :param start_time: Start time of the path planning.
        :return: Lists of nodes and positions representing the path, required reservations for the path,
                 distances to the end of the path for the nodes in the path.
        """
        path_nodes = []
        path_positions = []
        path_distances_to_end = []
        required_reservations = []

        # Initialize the current node and the distance to the end.
        current = current_node_obj
        goal_node = current_node_obj.node
        distance_to_end = 0

        while current is not None:
            # Add the node to the path list.
            path_nodes.append(current.node)
            path_positions.append(current.position)

            if len(path_nodes) > 1:
                if current.node == path_nodes[-2]:
                    # Waiting move. The same node should not be added to the path list twice.
                    path_nodes.pop(-1)
                    path_positions.pop(-1)

                    # Adapt the required reservations for the waiting move.
                    if current.parent is not None:
                        reservation = (round(start_time + current.parent.moved_time, 1), round(required_reservations[-1][1], 1))
                        required_reservations.pop(-1)
                        required_reservations.append(reservation)

                        next_move_time = current.moved_time - current.parent.moved_time
                    
                    else:
                        reservation = (start_time, round(required_reservations[-1][1], 1))
                        required_reservations.pop(-1)
                        required_reservations.append(reservation)
                
                else:
                    # Movement to the next node.
                    if current.parent is not None:
                        required_reservations.append((round(start_time + current.parent.moved_time, 1), round(start_time + current.moved_time + next_move_time, 1)))
                        path_distances_to_end.append(distance_to_end)

                        next_move_time = current.moved_time - current.parent.moved_time
                        move_length = self.get_distance(current.parent.position, current.position)
                        distance_to_end += move_length

                    else:
                        required_reservations.append((start_time, round(start_time + next_move_time, 1)))
                        path_distances_to_end.append(distance_to_end)

            else:
                # Calculate the reservation time of the node and the distance to the end of the path.
                if current.node == goal_node:
                    # No next move is planned.
                    if self.graph.nodes[current.node]["type"] == "station":
                        next_move_time = 0
                    elif self.graph.nodes[current.node]["type"] == "dwelling":
                        next_move_time = math.inf

                if current.parent is not None:
                    required_reservations.append((round(start_time + current.parent.moved_time, 1), round(start_time + current.moved_time + next_move_time, 1)))
                    path_distances_to_end.append(distance_to_end)

                    next_move_time = current.moved_time - current.parent.moved_time
                    move_length = self.get_distance(current.parent.position, current.position)
                    distance_to_end += move_length

                else:
                    required_reservations.append((start_time, round(start_time + next_move_time, 1)))
                    path_distances_to_end.append(distance_to_end)

            # Update the current node.
            current = current.parent

        return path_nodes, path_positions, required_reservations, path_distances_to_end

    def reserve_path(self, agent, path_nodes, required_reservations) -> None:
        """
        Reserve the path for the agent to follow.
        Nodes must be reserved for the time slots the agent is using them to avoid collisions.
        
        :param agent: Agent object.
        :param path_nodes: List of nodes representing the path for the agent.
        :param required_reservations: List of time slots the agent is using the nodes and connected edges.
        """
        for index, (path_node, required_reservation) in enumerate(zip(path_nodes, required_reservations)):
            # Get the time slot the node is reserved.
            start_time = required_reservation[0]
            end_time = required_reservation[1]

            # Check if the node is reserved for the given time slot.
            if not self.check_reservation(self.graph.nodes[path_node].get('reservations', []), start_time, end_time):
                agent.agents_object.logging.error(f"Node {path_node} / {self.graph.nodes[path_node]['pos']} is already reserved for the time {start_time} - {end_time}. Reservations: {self.graph.nodes[path_node].get('reservations', [])}.")
                raise ValueError(f"Node {path_node} / {self.graph.nodes[path_node]['pos']} is already reserved for that time slot.")

            # Reserve the node. Edges are reserved implicitly when the nodes are reserved.
            self.graph.nodes[path_node].setdefault('reservations', []).append((start_time, end_time, agent.agentId))

            if index == 0:
                # First node of the path is already utilized by the agent.
                continue

            # Define the order in which the nodes are used by the agents. It is used to avoid deadlocks when agents are delayed.
            self.graph.nodes[path_node].setdefault('utilisation_order', []).append((agent.agentId, start_time))
            self.graph.nodes[path_node]['utilisation_order'].sort(key=lambda x: x[1])

    def release_start_node(self, node) -> None:
        """
        Start node can be reserved for a infinite time.
        Remove the reservation of the node and reset the utilisation order.

        :param node: The node to be released.
        """
        for reservation in self.graph.nodes[node].get('reservations', []):
            if reservation[1] == float('inf'):
                self.graph.nodes[node]['reservations'].remove(reservation)
                break

    def delete_agent_path_reservations(self, agent, agent_path) -> None:
        """
        Delete the reservations of the agent along the path. Infinitely reserved nodes are not considered.
        
        :param agent: Agent object.
        :param agent_path: Path nodes of the agent.
        """
        for node in agent_path:
            for reservation in self.graph.nodes[node['nodeId']].get('reservations', []):
                if reservation[2] == agent.agentId and reservation[1] != float('inf'):
                    self.graph.nodes[node['nodeId']]['reservations'].remove(reservation)

    def check_path_reservations(self, path_nodes, required_reservations) -> bool:
        """
        Check if collisions with other agents occur along the path.

        :param path_nodes: List of nodes representing the path for the agent.
        :param required_reservations: List of time slots the agent is using the nodes and connected edges.
        :return: Boolean value indicating if collisions occur. True if no collisions occur, False otherwise.
        """
        for path_node, required_reservation in zip(path_nodes, required_reservations):
            start_time = required_reservation[0]
            end_time = required_reservation[1]
            # Check if the node is reserved for the given time slot.
            if not self.check_reservation(self.graph.nodes[path_node].get('reservations', []), start_time, end_time):
                return False
        return True

    def check_reservation(self, node_reservations, reservation_start_time, reservation_end_time) -> bool:
        """
        Check if the node is reserved for the given time slot.
        Remove in addition old reservations.

        :param node_reservations: List of time slots the node is reserved.
        :param reservation_start_time: Start time of the reservation.
        :param reservation_end_time: End time of the reservation.
        :return: Boolean value indicating if the node is reserved at the given time.
                 True if the node is not reserved, False otherwise.
        """
        # Make a copy of the node reservations to avoid changing the list while iterating over it.
        node_reservations_copy = node_reservations.copy()
        for node_reservation in node_reservations_copy:
            # Remove old reservations.
            reserved_start = node_reservation[0]
            reserved_end = node_reservation[1]
            if reserved_end < time.time() - self.start_time - 1000:
                node_reservations.remove((reserved_start, reserved_end, node_reservation[2]))
                continue
            # Check if the node is reserved for the given time slot.
            if reservation_start_time < reserved_end and reservation_end_time > reserved_start:
                return False
        return True

    def get_infinite_reserved_nodes(self, current_time, nodes) -> list:
        """
        Get the nodes that are reserved for an infinite time.

        :param current_time: Current time.
        :param nodes: Dictionary of nodes.
        :return: List of nodes that are reserved for an infinite time.
        """
        infinite_reserved_nodes = []
        for node in nodes:
            for reservation in nodes[node].get('reservations', []):
                if reservation[0] - 5 < current_time and reservation[1] == float('inf'):
                    infinite_reserved_nodes.append(node)
                    break
        return infinite_reserved_nodes

    def delete_all_reservations(self) -> None:
        """
        Delete all reservations of the nodes.
        """
        for node in self.graph.nodes:
            self.graph.nodes[node]['reservations'] = []
            self.graph.nodes[node]['utilisation_order'] = []
