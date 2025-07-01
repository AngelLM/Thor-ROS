# Dockerfile
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install MoveIt2 and useful dependencies
RUN apt-get update && apt-get install -y \
    wget curl git build-essential lsb-release gnupg2 locales sudo vim tmux net-tools iputils-ping \
    python3-colcon-common-extensions libserial-dev nlohmann-json3-dev\
    ros-humble-moveit ros-humble-moveit-resources \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-rviz2 ros-humble-ros2-control ros-humble-ros2-controllers\
    ros-humble-rosbridge-suite ros-humble-tf-transformations ros-humble-moveit-ros-control-interface\
    novnc websockify supervisor xfce4 x11vnc xvfb xterm

# Install node and npm ---
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && apt-get install -y nodejs

# Configure locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    echo "export LANG=en_US.UTF-8" >> /etc/bash.bashrc

COPY thor_logo.png /usr/share/backgrounds/thor_logo.png
COPY xfce4-desktop.xml /root/.config/xfce4/xfconf/xfce-perchannel-xml/xfce4-desktop.xml

# Add config supervisor
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Copy gazebo cache files
COPY gazebo_cache/. /root/.gazebo/

# Start script
COPY start.sh /start.sh
RUN chmod +x /start.sh

CMD ["/usr/bin/supervisord"]
