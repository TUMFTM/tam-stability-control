# ROS Distro
ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO

# Set working directory
RUN mkdir /ws
WORKDIR /ws

RUN sudo apt update && sudo apt install -y -q --no-install-recommends libboost-all-dev ros-jazzy-geographic-msgs

# Copy repo into the container
COPY . /ws/src

# Source ROS and build package
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --packages-up-to stability_controller_node_cpp tracking_controller_node_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Set entrypoint by sourcing overlay workspace
RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. \
    /ws/install/setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]