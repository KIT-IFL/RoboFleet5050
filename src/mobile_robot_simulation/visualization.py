import os
import math
import pygame


class Visualization:
    """
    Class for visualizing the simulated agents.
    """
    def __init__(self, config_data, graph) -> None:
        """
        Initialize the visualization object.

        :param config_data: The data from the configuration file.
        :param graph: The graph object based on which the agents are controlled by the fleet manager.
        """
        self.config_data = config_data
        self.graph = graph
        self.scaling_factor = config_data['pygame_scaling_factor']
        self.agent_rotation_diameter = self.config_data['agent_rotation_diameter']
        self.distance_to_boundary = config_data['agent_rotation_diameter'] * (3/4) * 3
        self.x_offset = 0
        self.y_offset = 0
        self.max_height = 0
        self.max_width = 0
        self.screen = self.initialize_screen()
        self.offscreen_surface = pygame.Surface(self.screen.get_size())
        self.clock = pygame.time.Clock()
        self.frames = []
        self.current_collisions = set()
        self.agent_image_path = os.path.join("data", "input_files", "mobileRobot_image.png")
        self.agent_image = pygame.image.load(self.agent_image_path)
        self.agent_image = pygame.transform.scale(self.agent_image,
                                                  (int(self.agent_rotation_diameter*self.scaling_factor)*0.83,
                                                   int(self.agent_rotation_diameter*(2/3)*self.scaling_factor)*0.83))
        
    def initialize_screen(self) -> pygame.Surface:
        """
        Initialize the Pygame screen.
        The screen size is based on the size of the graph. Meters are converted to pixels by multiplying by the scaling factor.
        """
        pygame.init()

        min_x = 1000000
        min_y = 1000000
        max_x = -1000000
        max_y = -1000000
        
        # Find the minimum and maximum x and y coordinates of the nodes.
        for node in self.graph.nodes:
            min_x = min(min_x, self.graph.nodes[node]['pos'][0])
            min_y = min(min_y, self.graph.nodes[node]['pos'][1])
            max_x = max(max_x, self.graph.nodes[node]['pos'][0])
            max_y = max(max_y, self.graph.nodes[node]['pos'][1])
        
        # Shift the graph to the right and down if the minimum x or y coordinate is negative.
        if min_x < 0:
            self.x_offset = (abs(min_x) + self.distance_to_boundary) * self.scaling_factor
        else:
            self.x_offset = (self.distance_to_boundary - min_x) * self.scaling_factor
        if min_y < 0:
            self.y_offset = (abs(min_y) + self.distance_to_boundary) * self.scaling_factor
        else:
            self.y_offset = (self.distance_to_boundary - min_y) * self.scaling_factor
        
        # Calculate the width and height of the screen.
        if max_x >= 0:
            self.max_width = (abs(max_x) + self.distance_to_boundary) * self.scaling_factor + self.x_offset
        else:
            self.max_width = self.x_offset - (abs(min_x) - self.distance_to_boundary) * self.scaling_factor
        
        if max_y >= 0:
            self.max_height = (abs(max_y) + self.distance_to_boundary) * self.scaling_factor + self.y_offset
        else:
            self.max_height = self.y_offset - (abs(max_y) - self.distance_to_boundary) * self.scaling_factor

        screen = pygame.display.set_mode((self.max_width, self.max_height))
        pygame.display.set_caption("Fleet Management Simulation")
        return screen

    def draw_graph(self, agents) -> None:
        """
        Draw the nodes and edges of the graph on the screen.

        :param agents: The simuled agents objects.
        """
        font = pygame.font.SysFont(None, int(0.36*self.scaling_factor))

        # Draw edges.
        for edge in self.graph.edges:
            start_pos = self.graph.edges[edge]['startNodePos']
            end_pos = self.graph.edges[edge]['endNodePos']
            pygame.draw.line(self.offscreen_surface, (0, 0, 0), 
                             (start_pos[0]*self.scaling_factor + self.x_offset, self.max_height - (start_pos[1]*self.scaling_factor + self.y_offset)),
                             (end_pos[0]*self.scaling_factor + self.x_offset, self.max_height - (end_pos[1]*self.scaling_factor + self.y_offset)),
                             int(0.08*self.scaling_factor))

        # Draw nodes.
        for node in self.graph.nodes:
            pos = self.graph.nodes[node]['pos']
            pygame.draw.circle(self.offscreen_surface, (150, 200, 255), 
                               (pos[0]*self.scaling_factor + self.x_offset, self.max_height - (pos[1]*self.scaling_factor + self.y_offset)),
                               int(0.30*self.scaling_factor))
            # Draw node ID
            text = font.render(node, True, (0, 0, 0))
            text_rect = text.get_rect(center=(int(pos[0]*self.scaling_factor + self.x_offset), int(self.max_height - (pos[1]*self.scaling_factor + self.y_offset))))
            self.offscreen_surface.blit(text, text_rect.topleft)
        
        for agent in agents:
            # Highlight released edges. 
            for edge in agent.edgeStates:
                if edge['edgeId'] == 'egde_to_start_node':
                    continue
                if edge['released']:
                    start_pos = self.graph.edges[edge['edgeId']]['startNodePos']
                    end_pos = self.graph.edges[edge['edgeId']]['endNodePos']
                    pygame.draw.line(self.offscreen_surface, (100, 200, 100), 
                                    (start_pos[0]*self.scaling_factor + self.x_offset, self.max_height - (start_pos[1]*self.scaling_factor + self.y_offset)),
                                    (end_pos[0]*self.scaling_factor + self.x_offset, self.max_height - (end_pos[1]*self.scaling_factor + self.y_offset)),
                                    int(0.08*self.scaling_factor))

            # Highlight released nodes.
            for node in agent.nodeStates:
                if node['released']:
                    pos = self.graph.nodes[node['nodeId']]['pos']
                    pygame.draw.circle(self.offscreen_surface, (100, 200, 100), 
                                       (pos[0]*self.scaling_factor + self.x_offset, self.max_height - (pos[1]*self.scaling_factor + self.y_offset)),
                                       int(0.30*self.scaling_factor))

                    # Draw node ID
                    text = font.render(node['nodeId'], True, (0, 0, 0))
                    text_rect = text.get_rect(center=(int(pos[0]*self.scaling_factor + self.x_offset), int(self.max_height - (pos[1]*self.scaling_factor + self.y_offset))))
                    self.offscreen_surface.blit(text, text_rect.topleft)

        # Draw dwelling nodes.
        self.draw_dwelling_nodes()

        # Draw the stations.
        self.draw_stations()

    def draw_dwelling_nodes(self) -> None:
        """
        Draw the dwelling nodes on the screen.
        """
        for dwelling_node in self.graph.dwelling_nodes:
            pos = self.graph.nodes[dwelling_node]['pos']
            # Draw the outline around the node.
            pygame.draw.circle(self.offscreen_surface, (255, 255, 0),
                               (pos[0]*self.scaling_factor + self.x_offset, self.max_height - (pos[1]*self.scaling_factor + self.y_offset)),
                               int(0.40*self.scaling_factor), int(0.12*self.scaling_factor))

    def draw_stations(self) -> None:
        """
        Draw the stations on the screen.
        """
        for station in self.graph.stations:
            for interaction_node_pos in self.graph.stations[station]['interactionNodesPos']:
                # Draw the outline around the node.
                pygame.draw.circle(self.offscreen_surface, (255, 0, 0),
                                   (interaction_node_pos[0]*self.scaling_factor + self.x_offset, self.max_height - (interaction_node_pos[1]*self.scaling_factor + self.y_offset)),
                                   int(0.40*self.scaling_factor), int(0.12*self.scaling_factor))
            # Draw the stations as a rectangle.
            station_pos = self.graph.stations[station]['station_position']
            pygame.draw.rect(self.offscreen_surface, (255, 0, 0),
                             (station_pos['x']*self.scaling_factor + self.x_offset - int(0.50*self.scaling_factor),
                              self.max_height - (station_pos['y']*self.scaling_factor + self.y_offset) - int(0.50*self.scaling_factor),
                              int(1.0*self.scaling_factor), int(1.0*self.scaling_factor)), 0)
            # Draw the station ID.
            text = pygame.font.SysFont(None, int(0.36*self.scaling_factor)).render(station, True, (0, 0, 0))
            text_rect = text.get_rect(center=(int(station_pos['x']*self.scaling_factor + self.x_offset), int(self.max_height - (station_pos['y']*self.scaling_factor + self.y_offset))))
            self.offscreen_surface.blit(text, text_rect.topleft)

    def draw_agents(self, agents) -> None:
        """
        Draw the simulated agents on the screen. The agvPosition of the agents is used to draw the agents on the screen.

        :param agents: The simuled agents objects.
        """
        font = pygame.font.SysFont(None, int(0.3*self.scaling_factor))

        for agent in agents:
            # Draw the agent.
            pos = agent.position
            rotated_image = pygame.transform.rotate(self.agent_image, math.degrees(agent.theta))
            rect = rotated_image.get_rect(center=(int(pos[0]*self.scaling_factor + self.x_offset), int(self.max_height - (pos[1]*self.scaling_factor + self.y_offset))))
            self.offscreen_surface.blit(rotated_image, rect.topleft)

            # Draw the load of the agent.
            if agent.loaded:
                pygame.draw.circle(self.offscreen_surface, (255, 165, 0),
                                   (int(pos[0]*self.scaling_factor + self.x_offset), int(self.max_height - (pos[1]*self.scaling_factor + self.y_offset))),
                                   int(self.agent_rotation_diameter*self.scaling_factor*0.2))

            # Draw the agent ID on the agent.
            text = font.render(str(agent.agentId), True, (0, 0, 0))
            text_rect = text.get_rect(center=(int(pos[0]*self.scaling_factor + self.x_offset), int(self.max_height - (pos[1]*self.scaling_factor + self.y_offset))))
            self.offscreen_surface.blit(text, text_rect.topleft)

    def display_collision_message(self) -> None:
        """
        Display a collision message on the screen.
        """
        font = pygame.font.SysFont(None, int(0.75*self.scaling_factor))
        text = font.render('Collision Detected!', True, (255, 0, 0))
        self.offscreen_surface.blit(text, (self.offscreen_surface.get_width() // 2 - text.get_width() // 2, self.offscreen_surface.get_height() // 2 - text.get_height() // 2))
