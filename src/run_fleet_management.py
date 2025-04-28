# Import the necessary libraries and modules.
import os
import sys
import time
import json
import logging
import threading
from typing import Dict

# Get the current working directory.
current_dir = os.getcwd()

# Define the folder names.
fleet_management_folder = 'RoboFleet5050'
src_folder = 'src'

if fleet_management_folder in current_dir.split('\\')[-1]:
    # Do nothing if the current working directory is already 'RoboFleet5050'.
    pass
elif src_folder in current_dir.split('\\')[-1]:
    # Change the current working directory to the 'RoboFleet5050' directory.
    os.chdir(os.pardir)
else:
    raise Exception("Current working directory is neither'.../RoboFleet5050', nor '.../src'.")

# Add the path to the src directory to the system path.
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

# Import the necessary classes.
from fleet_management.graph import Graph
from fleet_management.agents import Agents
from fleet_management.task_management import TaskManagement
from fleet_management.task_assignment import TaskAssignment
from fleet_management.fleet_management import FleetManagement
from fleet_management.agents_initialization import AgentsInitialization
from mobile_robot_simulation.agent_simulation import AgentSimulation


def setup_logging(log_file_path: str) -> logging.Logger:
    """
    Configure the logging setup.

    :param log_file_path: The path to the logging file.
    :return: The logging object.
    """
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(message)s',
        filename=log_file_path,
        filemode='w'
    )
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(formatter)
    logging.getLogger('').addHandler(console_handler)
    logging.getLogger('matplotlib').setLevel(logging.WARNING)
    return logging.getLogger(__name__)

class ConfigManager:
    """
    Class handles loading and managing input configuration files for the fleet management.
    """
    def __init__(self, config_paths: Dict[str, str]):
        self.config_data = self._load_json(config_paths['config'])
        self.lif_data = self._load_json(config_paths['lif'])
        self.agents_initialization_data = self._load_json(config_paths['agents_initialization'])

    @staticmethod
    def _load_json(path: str) -> dict:
        """
        Loads a JSON file from the given path.
        
        :param path: The path to the JSON file.
        :return: The JSON data.
        """
        try:
            with open(path, 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            logging.error(f"File not found: {path}")
            sys.exit(1)
        except json.JSONDecodeError:
            logging.error(f"Invalid JSON format in file: {path}")
            sys.exit(1)

def run_fleet_management(config_manager: ConfigManager, logging: logging.Logger):
    """
    Run the fleet manager.
    
    :param config_manager: The configuration manager object.
    :param logger: The logging object.
    """
    # Create the graph.
    graph = Graph(lif_data=config_manager.lif_data)

    if config_manager.config_data['fleet_management_mode'] == "SIMULATED_AGENTS":
        # Create the Agents object. For each simulated agent, a digital twin agent object is created.
        agents_object = Agents(config_data=config_manager.config_data, graph=graph,
                               agents_initialization_data=config_manager.agents_initialization_data, logging=logging)

        # Create the AgentSimulation objects. For each agent, a simulated agent object is created.
        agent_simulation = AgentSimulation(config_data=config_manager.config_data, lif_data=config_manager.lif_data,
                                           agents_initialization_data=config_manager.agents_initialization_data, logging=logging)

    elif config_manager.config_data['fleet_management_mode'] == "REAL_AGENTS":
        # Generate the AgentsInitialization file based on the initial state message of the agents. Wait untill all agents have sent their initial state message.
        initialization_event = threading.Event()
        agents_initialization = AgentsInitialization(config_data=config_manager.config_data, agents_initialization_data=config_manager.agents_initialization_data,
                                                    logging=logging, initialization_event=initialization_event)
        initialization_event.wait()
        config_manager.agents_initialization_data = agents_initialization.update_agents_initialization_file()

        # Create the Agents object. For each real agent, a digital twin agent object is created.
        agents_object = Agents(config_data=config_manager.config_data, graph=graph,
                               agents_initialization_data=config_manager.agents_initialization_data, logging=logging)

        # Create the AgentSimulation objects. For each agent, a simulated agent object is created. Only for visulaization purposes.
        agent_simulation = AgentSimulation(config_data=config_manager.config_data, lif_data=config_manager.lif_data,
                                           agents_initialization_data=config_manager.agents_initialization_data, logging=logging)
    
    else:
        logging.error("Invalid fleet_management_mode. Please select either 'SIMULATED_AGENTS' or 'REAL_AGENTS' in the config_file.")
        sys.exit(1)

    # Crate the task generation and task management and fleet management objects.
    task_management = TaskManagement(graph=graph, agents_object=agents_object, agent_simulation=agent_simulation)
    task_assignment = TaskAssignment(graph=graph, agents_object=agents_object, task_management=task_management)
    fleet_management = FleetManagement(config_data=config_manager.config_data, graph=graph, agents_object=agents_object,
                                       task_management=task_management)

    # Run the simulation.
    start_time = time.time()
    agent_simulation.run(start_time=start_time, run_time=config_manager.config_data['run_time'],
                         task_management=task_management, task_assignment=task_assignment, fleet_management=fleet_management)

def main():
    """
    Main function to set up and start the simulation.
    """
    # Logging setup.
    logging = setup_logging("data/output_files/logging_file.log")

    # Configuration paths.
    config_paths = {
        "config": "data/input_files/config_file.json",
        "lif": "data/input_files/lif_file.json",
        "agents_initialization": "data/input_files/agentsInitialization_file.json",
    }

    # Load configurations.
    config_manager = ConfigManager(config_paths)

    # Run the fleet management.
    run_fleet_management(config_manager, logging)

if __name__ == "__main__":
    main()
