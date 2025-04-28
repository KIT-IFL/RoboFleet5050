import time
import random
import threading

random.seed(10)


class TaskManagement:
    """
    Class for managing the tasks for the agents.
    """
    def __init__(self, graph, agents_object, agent_simulation) -> None:
        """
        Initialize the task management object.

        :param graph: The graph object.
        :param agents: The agents object managing the digital twin agents.
        :param agent_simulation: The agent simulation object managing the simulated agents.
        """
        self.start_time = 0
        self.graph = graph
        self.agents_object = agents_object
        self.agent_simulation = agent_simulation
        self.task_list = []
        self.tasks_generated = []
        self.unassigned_tasks = []
        self.pickup_nodes = self.get_pickup_nodes()
        self.dropoff_nodes = self.get_dropoff_nodes()
        self.task_generation_thread = threading.Thread(target=self.generate_random_tasks)

    def generate_random_tasks(self) -> None:
        """
        Generate tasks between the station nodes and add them to the task list.
        Tasks are generated at random at a defined frequency.
        """
        running = True
        while running:
            if len(self.unassigned_tasks) > 5:
                # If there are more than 5 unassigned tasks, wait for a while.
                time.sleep(1)
                continue

            # Generate a random pickup and dropoff node.
            pickup_node = random.choice(self.pickup_nodes)
            dropoff_node = random.choice(self.dropoff_nodes)

            # Check if the pickup and dropoff nodes correspond to the same station.
            if self.check_same_station(pickup_node, dropoff_node):
                same_station = True
            else:
                same_station = False

            # Ensure that the pickup and dropoff nodes are different.
            while pickup_node == dropoff_node or same_station:
                dropoff_node = random.choice(self.dropoff_nodes)
                if self.check_same_station(pickup_node, dropoff_node):
                    same_station = True
                else:
                    same_station = False

            # Create a task between the pickup and dropoff nodes.
            task = {'task_id': len(self.tasks_generated),
                    'start_node': pickup_node,
                    'goal_node': dropoff_node,
                    'task_assigned': False, 'task_completed': False}

            # Add the task to the list of open tasks the list of generated tasks.
            self.task_list.append(task)
            self.tasks_generated.append(task)
            self.unassigned_tasks.append(task)

            if len(self.tasks_generated) >= 1000:
                running = False

            # Wait for a certain interval before generating the next task.
            time.sleep(0.2)

    def get_pickup_nodes(self) -> list:
        """
        Get the pickup nodes from the graph.

        :return: A list of pickup nodes.
        """
        pickup_nodes = []
        for station in self.graph.stations.values():
            interactionNodeIds = station['interactionNodeIds']
            station_types = station['stationDescription'].split(';')
            for index, interactionNodeId in enumerate(interactionNodeIds):
                if station_types[index] == 'OUTPUT' or station_types[index] == 'INPUT_OUTPUT':
                    pickup_nodes.append(interactionNodeId)
        return pickup_nodes

    def get_dropoff_nodes(self) -> list:
        """
        Get the dropoff nodes from the graph.

        :return: A list of dropoff nodes.
        """
        dropoff_nodes = []
        for station in self.graph.stations.values():
            interactionNodeIds = station['interactionNodeIds']
            station_types = station['stationDescription'].split(';')
            for index, interactionNodeId in enumerate(interactionNodeIds):
                if station_types[index] == 'INPUT' or station_types[index] == 'INPUT_OUTPUT':
                    dropoff_nodes.append(interactionNodeId)
        return dropoff_nodes

    def check_same_station(self, pickup_node, dropoff_node) -> bool:
        """
        Check if the pickup and dropoff nodes correspond to the same station.

        :param pickup_node: The pickup node.
        :param dropoff_node: The dropoff node.
        :return: True if the pickup and dropoff nodes correspond to the same station, False otherwise.
        """
        for station in self.graph.stations.values():
            interactionNodeIds = station['interactionNodeIds']
            if pickup_node in interactionNodeIds and dropoff_node in interactionNodeIds:
                return True
        return False
