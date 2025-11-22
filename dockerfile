# Basis: Offizielles ROS2 Jazzy Image
FROM ros:jazzy

# Installiere Git und die benötigte RMW-Implementation
RUN apt-get update && apt-get install -y \
    git \
    ros-jazzy-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Setze das Arbeitsverzeichnis im Container
WORKDIR /ros2_ws/src

# Kopiere dein Paket ins Image
COPY . /ros2_ws/src/ros2-quac-inverse-kinematics

# Setze die Shell auf bash, um Setup-Skripte korrekt zu sourcen
SHELL ["/bin/bash", "-c"]

# Baue den ROS2-Workspace
WORKDIR /ros2_ws
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

# Starte den gewünschten Node
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run quac_ik quac_ik_node"]