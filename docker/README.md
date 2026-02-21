# ROS2 TurtleBot3

   ```bash
   cd chat2robot/docker
   ./setup_and_run.sh   //only for Ubuntu 22.04  

   ```

# ROS2 TurtleBot3 -  Docker Setup
1. **Set a new environment variable:**
   ```bash
   export GOOGLE_API_KEY="your_key_here"
   source ~/.bashrc
   ```

2. **Change project directory:**
   ```bash
   cd ~/chat2robot/docker
   ```
3. **Container build:**
   ```bash
   docker compose build
   ```
4. **Container start:**
   ```bash
   GOOGLE_API_KEY="$GOOGLE_API_KEY" docker compose up --detach --gpus all
   ```
   *without GPU:*
   ```bash
   GOOGLE_API_KEY="$GOOGLE_API_KEY" docker compose up --detach
   ```
5. **Container Access:**
   ```bash
   docker exec -it ros2_turtlebot3 bash
   ```
6. **Container Stop:**
   ```bash
   docker compose down
   ```


