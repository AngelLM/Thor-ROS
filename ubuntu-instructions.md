# Thor-ROS & Asgard Installation Guide (Ubuntu 22.04)

Ready to run Thor and Asgard on Ubuntu?
Just follow these steps, grab your terminal and let the penguin do the heavy lifting!

---

## 1. Set up Ubuntu 22.04

Start with a fresh install of Ubuntu 22.04.  
If you’re already running it, you’re good to go!


## 2. Install ROS2 Humble

We recommend following the official ROS2 documentation:  
👉 [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Here’s a quick summary of the steps:

**Set locale**
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

**Setup sources**
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

**Install ROS2 packages**
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

**Source the ROS2 setup script**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## 3. Install Thor-ROS dependencies

Let’s get all the extra goodies for Thor-ROS and Asgard:

**ros2-control**
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

**Gazebo**
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

**MoveIt2**
```bash
sudo apt install ros-humble-moveit-ros-control-interface ros-humble-moveit ros-humble-moveit-resources
```

**rosbridge**
```bash
sudo apt install ros-humble-rosbridge-suite
```

**Serial and JSON libraries**
```bash
sudo apt install libserial-dev nlohmann-json3-dev
```


## 4. Clone the Thor-ROS repository

Let’s get the code!

```bash
cd ~/
git clone https://github.com/AngelLM/Thor-ROS
```
---

## 5. Build the ROS2 packages

Time to compile:

```bash
cd ~/Thor-ROS/ws_thor
colcon build
```


## 6. Install npm and Asgard dependencies

Let’s get Asgard ready to shine:

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo bash -
sudo apt install -y nodejs
cd ~/Thor-ROS/asgard
npm install
```


## 7. Launch the Thor ROS2 system (Terminal #1)

Start the simulated robot or connect to your real one!

```bash
cd ~/Thor-ROS/ws_thor
source install/setup.bash
ros2 launch thor_bringup simulated_robot.launch.py
```


## 8. Launch Asgard (Terminal #2)

Open a second terminal and start the web interface:

```bash
cd ~/Thor-ROS/asgard/
npm run dev
```


## 9. Open Asgard in your browser

Copy the URL shown in Terminal #2 after running `npm run dev` (usually something like `http://localhost:5173`) and enjoy controlling Thor from your browser!



**You’re all set!**  
If you run into trouble or want to join the community, don’t forget to visit our [Discord](https://discord.com/invite/a5dSVqSUK5).