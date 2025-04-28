import math
import threading


class Graph:
    """
    Class representing the graph, based on which the agents are controlled by the fleet manager.
    """
    def __init__(self, lif_data):
        """
        Initialize the graph based on the data from the LIF file.
        
        :param lif_data: The data from the LIF file.
        """
        self.nodes = self.get_nodes(lif_data)
        self.edges = self.get_edges(lif_data)
        self.stations = self.get_stations(lif_data)
        self.dwelling_nodes = self.get_dwelling_nodes(lif_data)
        self.node_resources, self.edge_resources = self.initialize_resources()
    
    def get_nodes(self, lif_data) -> dict:
        """
        Get the nodes from lif_data.

        :param lif_data: The data from the LIF file.
        :return: A dictionary containing the nodes. The keys are the node IDs.
        """
        nodes = {}
        for node in lif_data['layouts'][0]['nodes']:
            nodeId = node['nodeId']
            pos = (node['nodePosition']['x'], node['nodePosition']['y'])
            if 'theta' in node['vehicleTypeNodeProperties'][0]:
                theta = node['vehicleTypeNodeProperties'][0]['theta']
            else:
                theta = None

            nodes[nodeId] = {'nodeId': nodeId, 'pos': pos, 'theta': theta, 'type': 'basic'}

        return nodes
    
    def get_edges(self, lif_data) -> dict:
        """
        Get edges from lif_data.

        :param lif_data: The data from the LIF file.
        :return: A dictionary containing the edges. The keys are the edge
        """
        edges = {}
        for edge in lif_data['layouts'][0]['edges']:
            edgeId = edge['edgeId']
            startNodeId = edge['startNodeId']
            endNodeId = edge['endNodeId']
            startNodePos = self.nodes[startNodeId]['pos']
            endNodePos = self.nodes[endNodeId]['pos']

            edges[edgeId] = {'edgeId': edgeId, 'startNodeId': startNodeId, 'endNodeId': endNodeId,
                              'startNodePos': startNodePos, 'endNodePos': endNodePos}

        return edges
    
    def get_edge(self, nodeId_1, nodeId_2) -> dict:
        """
        Get an edge based on the start and end node IDs.

        :param nodeId_1: The ID of node number 1.
        :param nodeId_2: The ID of node number 2.
        :return: The edge.
        """
        reversed_edge = None
        for edge in self.edges:
            if (self.edges[edge]['startNodeId'] == nodeId_1 and
                    self.edges[edge]['endNodeId'] == nodeId_2):
                return self.edges[edge]
            elif (self.edges[edge]['startNodeId'] == nodeId_2 and
                  self.edges[edge]['endNodeId'] == nodeId_1):
                reversed_edge = self.edges[edge]
        if reversed_edge is not None:
            return reversed_edge
        return None

    def remove_edge(self, edge) -> None:
        """
        Remove an edge from the graph.

        :param edge: The edge to remove.
        """
        if edge['edgeId'] in self.edges:
            del self.edges[edge['edgeId']]

    def add_edge(self, edge) -> None:
        """
        Add an edge to the graph.

        :param edge: The edge to add.
        """
        self.edges[edge['edgeId']] = edge

    def get_stations(self, lif_data) -> dict:
        """
        Get the stations from lif_data.
        Stations are nodes where the agents can interact with the environment.

        :param lif_data: The data from the LIF file.
        :return: A dictionary containing the stations. The keys are the station IDs.
        """
        stations = {}
        for station in lif_data['layouts'][0]['stations']:
            station_id = station['stationId']
            if station['stationType'] == 'PRODUCTION':
                interactionNodeIds = station['interactionNodeIds']
                interactionNodesPos = [self.nodes[nodeId]['pos'] for nodeId in interactionNodeIds]
                interactionNodesTheta = [self.nodes[nodeId]['theta'] for nodeId in interactionNodeIds]
                
                stations[station_id] = {'interactionNodeIds': interactionNodeIds, 'interactionNodesPos': interactionNodesPos,
                                        'interactionNodesTheta': interactionNodesTheta, 'stationDescription': station['stationDescription'],
                                        'station_position': station['stationPosition']}
                
                for nodeId in interactionNodeIds:
                    self.nodes[nodeId]['type'] = 'station'

        return stations

    def get_dwelling_nodes(self, lif_data) -> list:
        """
        Get the dwelling nodes.
        Dwelling nodes are nodes, where the agents can stop and wait for the next task.
        Number of dwelling nodes must be equal or greater than the number of agents.

        :param lif_data: The data from the LIF file.
        :return: A list of node IDs of the dwelling nodes.
        """
        dwelling_nodes = []
        for station in lif_data['layouts'][0]['stations']:
            if station['stationType'] == 'CHARGING':
                for nodeId in station['interactionNodeIds']:
                    dwelling_nodes.append(nodeId)
                    self.nodes[nodeId]['type'] = 'dwelling'

        return dwelling_nodes

    def get_closest_dwelling_node(self, position, dwelling_nodes) -> str:
        """
        Get the closest dwelling node to the position of the agent.
        
        :param position: Position of the agent.
        :param dwelling_nodes: List of dwelling nodes to consider. Default is all dwelling nodes.
        :return: The ID of the closest dwelling node.
        """
        closest_distance = float('inf')
        closest_dwelling_node = None
        for dwelling_node in dwelling_nodes:
            distance = math.dist(position, self.nodes[dwelling_node]['pos'])
            if distance < closest_distance:
                closest_distance = distance
                closest_dwelling_node = dwelling_node
        return closest_dwelling_node

    def get_connected_nodes(self, node) -> list:
        """
        Get a list of nodes connected to the node position.
        
        :param node: The node ID.
        :return: A list of the nodes connected to the current node of the agent.
        """
        connected_nodes = []
        for edge in self.edges:
            # Get the outgoing edges from the node (unidirectional graph).
            if node == self.edges[edge]['startNodeId']:
                connected_nodes.append(self.edges[edge]['endNodeId'])
            
            # Get the incoming edges from the node (unidirectional graph).
            # Condition is required if the edges of the graph are bidirectional.
            elif node == self.edges[edge]['endNodeId']:
                connected_nodes.append(self.edges[edge]['startNodeId'])
        return connected_nodes

    def get_connected_edge(self, startNodeId, endNodeId) -> str:
        """
        Get the edge ID based on the start and end node IDs.

        :param startNodeId: The start node ID.
        :param endNodeId: The end node ID.
        :return: The edge ID.
        """
        reversed_edge = None
        for edge in self.edges:
            if (self.edges[edge]['startNodeId'] == startNodeId and
                    self.edges[edge]['endNodeId'] == endNodeId):
                return edge
            elif (self.edges[edge]['startNodeId'] == endNodeId and
                  self.edges[edge]['endNodeId'] == startNodeId):
                reversed_edge = edge
        if reversed_edge is not None:
            return reversed_edge
        return None

    def initialize_resources(self) -> tuple[dict, dict]:
        """
        Initialize the node and edge resources of the graph with a capacity of 1.

        :return: A tuple containing the node and edge resources dictionaries of the graph.
        """
        # All nodes and edges have a capacity of 1.
        node_resources = {node: Resource(capacity=1) for node in self.nodes}
        edge_resources = {edge: Resource(capacity=1) for edge in self.edges}

        return node_resources, edge_resources

    def reinitialize_resources(self) -> None:
        """
        Reinitialize the node and edge resources of the graph.
        """
        self.node_resources, self.edge_resources = self.initialize_resources()


class Resource:
    """
    Class representing a resource with a certain capacity. The resource can be requested and released.
    """
    def __init__(self, capacity, count=0):
        self.capacity = capacity
        self.count = count
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)

    def request(self):
        event = threading.Event()
        with self.condition:
            if self.count < self.capacity:
                self.count += 1
                event.set()
            else:
                threading.Thread(target=self._wait_for_resource, args=(event,)).start()
        return event

    def _wait_for_resource(self, event):
        with self.condition:
            while self.count >= self.capacity:
                self.condition.wait()
            self.count += 1
            event.set()

    def release(self):
        with self.condition:
            self.count -= 1
            self.condition.notify()
