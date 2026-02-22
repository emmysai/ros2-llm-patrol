# ROS2 LLM Waypoint Patrol

## Overview

This project implements a **ROS2-based mobile robot patrol system**
combined with a **Large Language Model (LLM) agent**.

The robot autonomously navigates between four predefined waypoints in a
loop.\
An LLM-powered chatbot provides natural language summaries of the
robot's internal state and sensor data.

------------------------------------------------------------------------


### Robot Environment

-   ROS2 Humble
-   TurtleBot3 (Burger)
-   Gazebo Simulation
-   SLAM Toolbox
-   Navigation2

### Waypoint Patrol

The robot continuously cycles through the following waypoints:

1.  **Home Position**\
    `x: 0.00  y: 0.00  theta: 0.00`

2.  **Pose A**\
    `x: -2.00  y: 1.00  theta: 0.00`

3.  **Pose B**\
    `x: -0.50  y: -2.00  theta: 0.00`

4.  **Goal**\
    `x: -2.00  y: -1.50  theta: 0.00`

After reaching the final waypoint, the robot restarts from the first.

------------------------------------------------------------------------

## LLM Agent

The system integrates a **Google Gemini LLM (gemini-2.5-flash)**.

The LLM acts as a chatbot that:

-   Summarizes robot pose and motion
-   Interprets sensor values (LaserScan, Odometry)
-   Detects potential obstacles
-   Determines the nearest waypoint
-   Provides human-readable analysis

The LLM retrieves live robot data via ROS2 service tools:

-   `/llm_tools/get_robot_state`
-   `/llm_tools/get_nearest_waypoint`

------------------------------------------------------------------------

## Running with Docker

## Docker set up
**Set a new environment variable:**

Set your Google API Key inside the container:

``` bash
cd docker
export GOOGLE_API_KEY="your_key_here"
source ~/.bashrc
```

**Container build:**
   ```bash
   docker compose build
   ```
   
### Start container

``` bash
docker compose up -d
docker compose up -d --build
docker exec -it ros2_turtlebot3 bash
```

## Build the ROS2 workspace
```bash
colcon build
```

### Source ROS

``` bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Launch Simulation

``` bash
ros2 launch turtlebot3_full_bringup full_bringup.launch.py
```

### Launch Waypoint Patrol

``` bash
ros2 launch waypoint_patrol patrol.launch.py
```

### Start LLM Tools

``` bash
ros2 launch llm_agent llm_agent.launch.py
```

### Start Chatbot

``` bash
export GEMINI_MODEL="gemini-2.5-flash"
ros2 run llm_agent chat_cli
```

## UI

``` bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

python3 -m pip install --user streamlit
python3 -m streamlit run app_neu.py --server.address 0.0.0.0 --server.port 8501
```
Open in Browser\

http://localhost:8501\

------------------------------------------------------------------------

## Example Questions

You can ask:

-   *Wie ist der Zustand des Roboters?*
-   *Welcher Punkt ist am nächsten?*
-   *Gibt es Hindernisse in der Nähe?*
-   *Bewegt sich der Roboter?*

------------------------------------------------------------------------

## Architecture

ROS Topics → ROS2 Services → LLM Tools → Gemini Model → Natural Language
Output

The LLM does **not hallucinate sensor data**.\
It only interprets structured JSON data retrieved via ROS services.


------------------------------------------------------------------------

# 3. ROS Tools

## Call Robot State

``` bash
ros2 service call /llm_tools/get_robot_state std_srvs/srv/Trigger {}
```

## Call Next Waypoint

``` bash
ros2 service call /llm_tools/get_nearest_waypoint std_srvs/srv/Trigger {}
```

------------------------------------------------------------------------

## Tested With

-   Ubuntu 22.04
-   ROS2 Humble
-   Docker
-   Python 3.10
-   Google Gemini API

------------------------------------------------------------------------


# Impact of Language (German vs. English)

The system prompt is formulated in German and the interaction is primarily conducted in German. Therefore it will work in German Language only.

**Recommendation:**\
For industrial applications, the system logic should primarily use English.\
Multilingual support can be added at the UI level.
Furthermore a translation pipeline can be added.

------------------------------------------------------------------------

## Recommendation

A chatbot for the direct control of an industrial robot is not recommended if it\ is used as the sole decision-making authority. Risks such as hallucinaitions might occur.\ Misinterpretations might also pose a risk factor.\

An LLM is however suitable as:\

-   Assistance system
-   Diagnostic interface
-   Monitoring tool
-   Knowledge access to documentation
-   Training support
------------------------------------------------------------------------


# ros2-llm-patrol

docker container basis from:
https://github.com/nils93/chat2robot
