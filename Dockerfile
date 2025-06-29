ARG BASE_IMAGE=lcas.lincoln.ac.uk/lcas/ros-docker-images:humble-2

FROM ${BASE_IMAGE} as base

USER root

ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy source code for dependency analysis
FROM base as sourcefilter
COPY ./src /tmp/src
# Keep only package.xml files for dependency resolution
RUN find /tmp/src -type f \! -name "package.xml" -delete

# Install ROS dependencies
FROM base as depbuilder
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update --rosdistro ${ROS_DISTRO} && apt-get update
RUN cd /tmp/src && rosdep install --from-paths . --ignore-src -r -y \
    && cd && rm -rf /tmp/src \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Final stage
FROM depbuilder as final

# Copy the full source code
COPY ./src /opt/ros_ws/src

# Set up workspace
WORKDIR /opt/ros_ws

# Build the ROS2 package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set up environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "source /opt/ros_ws/install/setup.bash" >> /etc/bash.bashrc

# Set default environment variables for MCP server
ENV MCP_HOST=0.0.0.0
ENV MCP_PORT=8000

# Expose the SSE port
EXPOSE 8000

# Switch to ros user
USER ros

# Set default command
CMD ["/bin/bash"]
