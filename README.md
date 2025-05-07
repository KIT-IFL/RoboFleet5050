# RoboFleet5050

This repository provides a **lightweight, modular and scalable fleet management system** for coordinating both **simulated and real mobile robots** using the [VDA 5050 communication standard](https://www.vda.de/de/themen/automobilindustrie/vda-5050). Layouts are defined using the [Layout Interchange Format (LIF)](https://vdma.org/documents/34570/3317035/FuI_Guideline_LIF_GB.pdf/779bc75c-9525-8d13-412e-fff82bc6ab39?t=1710513623026), which includes a node-edge graph and station definitions.

The system feautures **prioritized multi-robot path planning** inspired by [Cooperative A*](https://dl.acm.org/doi/10.5555/3022473.3022494), and ensures **deadlock prevention** using a concept inspired by [Logical Time](https://ieeexplore.ieee.org/abstract/document/9261470), regardless of deviations between planned and actual path execution of the mobile robots. A **2D simulation environment**, built with **PyGame**, is integrated for rapid testing and visualization.

## Features
- Supports both **simulated and real mobile robots**
- Communication via **VDA 5050**, including:
    - `order` topic
    - `state` topic
    - Expandable to other topics (`instantAction`, `factsheet`, etc.)
- Path planning for **lifelong tranportation processes**:
    - Inpired by [Cooperative A*](https://dl.acm.org/doi/10.5555/3022473.3022494)
    - Uses **dwelling nodes** for collision and deadlock-free planning (number of dweeling nodes must be equal or larger than number of mobile robots)
    - Path structure: `Dwelling → Pickup → Dropoff → Dwelling`
- **Prevents resource conflicts and deadlocks** using prioritized access to resources (nodes, edges, stations) inpired by the concept of [Logical Time](https://ieeexplore.ieee.org/abstract/document/9261470)
- Integrated **PyGame simulation**
- Configurable via **JSON files**

## Visualization

The following visualizations demonstrate how the simulated mobile robots perform transportation tasks:
- The mobile robots are initialized at random positions within the environment.
- Each mobile robot first moves to the nearest dwelling node (framed in yellow).
- Nodes and edges released for the mobile robots (VDA 5050 order message) are highlighted in green.
- At the dwelling node, a transportation task is assigned to the mobile robots.
- The mobile robots move to the corresponding pickup station (framed in red).
- At the pickup station, they are loaded with a transport unit (indicated by an orange circle).
- The mobile robots transports the load to the designated dropoff station (framed in red).
- The processing (loading and unloading) of the transport unit takes a defined time.
- After unloading, the mobile robots return to a dwelling node to await their next task.

<p align="left">
    <img src="./data/output_files/demo_videos/FM_2Agents_TestArea.gif" width="500" alt="Fleet Management Simulation - Test Area" style="margin-left: 40px;">
    <img src="./data/output_files/demo_videos/FM_6Agents_ArenaArea.gif" width="500" alt="Fleet Management Simulation - Arena Area" style="margin-left: 40px;">
</p> 
</p> <p align="left"> <b>Top:</b> Two mobile robots are controlled in a simple test layout with four stations.<br> <b>Bottom:</b> Six mobile robots are controlled in a more complex layout with five stations. </p>

## Project Setup Guide

### Requirements
- [Anaconda](https://www.anaconda.com/) or [Miniconda](https://docs.anaconda.com/miniconda/)
- A running **MQTT Broker** (e.g., [Mosquitto](https://mosquitto.org/))

### Clone the repository
```
git clone https://github.com/KIT-IFL/RoboFleet5050.git
```

### Create a Conda Environment using the `environment.yml`
```
conda env create -n fleet_management_env -f environment.yml
```

## Run the Simulation
1. Set the following in `config_file.json`:
    ```
    "mqtt_broker_ip": "localhost",
    "fleet_management_mode": "SIMULATED_AGENTS"
    ```

2. Ensure your MQTT broker is running, the start the fleet manager:
    ```
    python run_fleet_management.py
    ```
    This will launch a PyGame window and start the simulation.

## Run with Real Mobile Robots
1. Set the following in `config_file.json`:
    ```
    "mqtt_broker_ip": "your-broker-address",
    "fleet_management_mode": "REAL_AGENTS"
    ```
2. Set up `agentsInitialization_file.json` to match your real mobile robots:
    
    Each `agentId` must match the mobile robot’s `serialNumber` in the VDA 5050 state messages.

3. Ensure your MQTT broker is running, the start the fleet manager:
    ```
    python run_fleet_management.py
    ```
4. The system will:
    - Wait for initial state messages from all mobile robots defined in `agentsInitialization_file.json`.
    - Assign transportation tasks and send order messages once all mobile robots have send an initial state message.

## Configuration Files
All configuration files are located in the `data/input_files/` directory:

- `lif_file.json`: Defines the layout including nodes, edges, and stations.
- `agentsInitialization_file.json`: Contains initialization information for each mobile robot and their communication topics.
- `config_file.json`: General configuration settings including:
    - MQTT broker
    - Simulation and visulaization parameters
    - Simulation mode (control of simulated or real mobile robots)

## Acknowledgement

This project has been created for the work in the research project [SDM4FZI](https://www.sdm4fzi.de/) to enable flexible material transportation in changing production environments using mobile robots.

We extend our sincere thanks to the German Federal Ministry for Economic Affairs and Climate Action (BMWK) for supporting this research project 13IK001ZF “Software-Defined Manufacturing for the automotive and supplying industry”.

## License Notice

This project is licensed under the MIT License (see `LICENSE` for details) and uses the following third-party packages:

- `jsonschema` – MIT License
- `sortedcontainers` – Apache License 2.0
- `paho-mqtt` – Eclipse Public License 2.0
- `pygame` – GNU Lesser General Public License v2.1 or later (LGPL-2.1+)
- `pygame-screen-record` – MIT License

All libraries are used as-is via pip or conda and not modified.
Please refer to each package’s repository for full license texts.