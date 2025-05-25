# Thor-ROS Docker Environment

This repository provides a ready-to-use Docker environment with **ROS 2 Humble**, **MoveIt 2**, **Gazebo**, and a graphical desktop accessible via your web browser using noVNC.

## 🪟 Setup on Windows

> This setup uses **WSL 2** and **Docker Desktop** to run a complete ROS 2 development environment with GUI access.

### 1. Install Prerequisites

- **Install [WSL 2](https://learn.microsoft.com/en-us/windows/wsl/install)** (Windows Subsystem for Linux)
- **Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)** and enable WSL integration

### 2. Launch WSL

Open your WSL terminal (e.g. Ubuntu).

### 3. Clone the Repository

```bash
git clone https://github.com/AngelLM/Thor-ROS/ --depth 1
```

### 4. Build the Docker Image

```bash
cd Thor-ROS/docker
docker build -t ros2-thor .
```

### 5. Run the Container

```bash
docker run -it -p 6080:6080 --name ros2_thor -v ./Thor-ROS/ws_thor:/home/ros/ws_thor --device=/dev/ttyUSB0 --privileged ros2-thor:latest
```

> ✅ Replace `/dev/ttyUSB0` with the actual serial device of your Arduino if needed. This grants USB access to the container.

### 6. Access the GUI

Open your browser and navigate to:

```
http://localhost:6080/vnc.html
```

Click **Connect** to access the XFCE desktop environment.

### 7. Build and Launch ROS 2 Programs

Inside the container terminal:

```bash
cd /home/ros/ws_thor
colcon build
source install/setup.bash
ros2 launch thor_bringup simulated_robot.launch.py
```

---

## 🛠️ Optional Customization

### 🖥️ Change Desktop Resolution

To change the resolution, modify the `supervisord.conf`:

```ini
[program:xvfb]
command=/usr/bin/Xvfb :1 -screen 0 1920x1080x16
```
And rebuild the docker image.
