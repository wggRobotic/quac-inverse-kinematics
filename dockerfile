FROM ros:humble
SHELL ["/bin/bash", "-c"]
WORKDIR /quac

RUN apt update
RUN apt install -y ros-humble-rmw-cyclonedds-cpp

COPY ./quac_inverse_kinematics /quac/src/quac_inverse_kinematics
RUN . /opt/ros/humble/setup.bash && colcon build