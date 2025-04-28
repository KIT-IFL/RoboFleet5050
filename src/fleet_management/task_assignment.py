import time
import math
import threading

class TaskAssignment:
    """
    Class for assigning tasks to the agents.
    """
    def __init__(self, graph, agents_object, task_management) -> None:
        """
        Initialize the task assignment object.

        :param graph: The graph object.
        :param agents: The agents object managing the digital twin agents.
        :param task_management: The task management object.
        """
        self.start_time = 0
        self.graph = graph
        self.agents_object = agents_object
        self.task_management = task_management
        self.assignment_thread = threading.Thread(target=self.task_assignment_manager)
    
    def task_assignment_manager(self) -> None:
        """
        Tasks are assigned to the digital twin agents that are in the IDLE state.
        """
        running = True
        while running:
            next_task = None
            available_agents = []

            # Get the next unassigned task.
            for task in self.task_management.task_list:
                if not task['task_assigned']:
                    next_task = task
                    break
            
            if next_task is None:
                # No unassigned tasks available. Wait for a while.
                time.sleep(0.5)
                continue
            
            # Get the available agents.
            for agent in self.agents_object.agents:
                if agent.agent_state == "IDLE" and agent.task is None:
                    available_agents.append(agent)
            
            # Assign the task to the agent that is closest to the start node of the task.
            if next_task is not None and available_agents:
                closest_agent = self.get_clostest_available_agent(available_agents, next_task)
                if closest_agent is not None:
                    self.assign_task(closest_agent, next_task)
                    available_agents.remove(closest_agent)

            if available_agents != [] and next_task is not None:
                # No sleep time if there are available agents and unassigned tasks.
                continue

            # Sleep for a while if no tasks are available or no agents are available.
            time.sleep(0.5)

    def get_clostest_available_agent(self, available_agents, task) -> object:
        """
        Get the agent that is closest to the start node of the task.

        :param available_agents: The list of available agents.
        :param task: The task to be assigned.
        :return: The agent that is closest to the start node of the task.
        """
        closest_agent = None
        closest_distance = float('inf')
        for agent in available_agents:
            distance = math.dist(agent.position, self.graph.nodes[task['start_node']]['pos'])
            if distance < closest_distance:
                closest_distance = distance
                closest_agent = agent
        return closest_agent
    
    def assign_task(self, agent, task) -> None:
        """
        Assign a task to the agent.

        :param agent: The digital twin agent object.
        :param task: The task to be assigned to the agent.
        """
        task['task_assigned'] = True
        self.task_management.unassigned_tasks.remove(task)

        # Update the agent state.
        agent.update_agent_state(agent_state="PLANNING", position=agent.position, theta=agent.theta,
                                 task=task, path=None, orderId=agent.orderId, order_updateId=agent.order_updateId,
                                 lastNodeId=agent.lastNodeId, lastNodeSequenceId=agent.lastNodeSequenceId, driving=False)

        agent.agents_object.logging.info(f"Task {agent.task['task_id']} assigned to agent {agent.agentId} at time {round(time.time() - self.start_time, 2)}. Start node: {agent.task['start_node']}. Goal node: {agent.task['goal_node']}.")
