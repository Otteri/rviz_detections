###################################################################
# Author: Joni Airaksinen (Otteri)
# Build: $ docker build -t rviz-detections .
# Run:   $ docker run --rm -it --network=host rviz-detections
###################################################################

# Path to binary release. Assumes it to be in repo root by default.

FROM ros:melodic AS build-stage

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Copy whole package as is
COPY . app/src/

# Build ROS detector
RUN /bin/bash -c "source /opt/ros/melodic/setup.sh && \
    cd /app && \
    catkin config --init --install && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release"

################################################################################
FROM ros:melodic-ros-core as production-stage

# Copy installation (build output)
COPY --from=build-stage /app/install/ /app/install/

ENTRYPOINT bash -c "cd /app/ && source install/setup.bash && roslaunch rviz_detections rviz_detections.launch"
