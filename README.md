# Thor-ROS
This repository holds the current progress on the development of the software implementation for the [Thor Robotic Arm](https://github.com/angellm/thor) based on the Dockerize [ROS2](https://www.ros.org/) and [MoveIt](https://moveit.ai/) implementation.


## Hardware
[Thor](https://github.com/AngelLM/Thor) is an Open Source and printable robot arm with six degrees of freedom designed and built by [ÁngelLM](https://x.com/_angellm). Its configuration (yaw-roll-roll-yaw-roll-yaw) is the same used by most of the manipulator robots on the market. In its extended position, Thor is about 625mm high and can lift loads up to 750 grams. You can find more information about the THOR project at the [THOR project](http://thor.angel-lm.com/) website.

## ROS Implementation
In order to give THOR the most versatile and futureproof implementation possible, ROS2 Humble LTS was chosen as our distribution of choice. Additionally, with the objective of making the implementation as flexible as possible, another abstraction layer was added by Dockerizing ROS, which allows it to run on any machine regardless of its operating system.


![ROS_Humble](/Docs/Media/humble_composed.png)
### Repository Structure
The THOR-ROS repository is based around the [ROS Humble](https://hub.docker.com/r/osrf/ros/tags) Docker image from [OSFR](https://github.com/ropensci/osfr). Its main branch provides the most stable version of the implementation to date.


### Set up and Use
**Docker set up**: For making use of the Thor ROS2 implementation, we have to make sure we have Docker installed and running so we can build the image [here](/Docs/windows_docker_installation.pdf) we have provided a step-by-step guide to natively run Docker on Windows machines. The process of building and launching the image can be done in many ways, but you can consult [this](/Docs/vscode_docker_integration.pdf) provided guide for our recommended method, which seamlessly integrates VsCode and Docker.

Traditionally, when working with Dockerized images, users interact with them via the system terminal. While this approach is typically sufficient for most applications, it falls short for robotics use cases. In robotics, we need access to simulation software like RViz and Gazebo, which require graphical interfaces in order to be visualize and operated. For this reason, we have taken the time to implement access to the existing GUI by forwarding it to a system local port.

**Access the GUI**: In order to access the GUI first, we have to make sure we have built and initialized the container. Once we have done this first step, we can access our browser of choice and access `localhost:6080` in the navigation bar. Then we can use `vscode` as our login once we are prompted to introduce a password. VOILA! All ready for use.

![THOR](/Docs/Media/moveit_thor_build.png)

---
## Join the community.
If you want to contribute, feel free to contact us or join the [Thor Robot Discord Community](https://discord.gg/g3uFPnt5) so we can together improve, develop and maintain the THOR-ROS project.

## Last added changes
- THOR URDF collisions + visual meshes

- Created Moveit 2.0 configuration package.

- Motion planning within RViz

