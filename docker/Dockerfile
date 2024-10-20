ARG DISTRO
FROM ros:${DISTRO}

ARG DISTRO

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/noether_roscon_2024
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/noether_roscon_2024 \
  apt update -y -qq \
  && source /opt/ros/${DISTRO}/setup.bash \
  && vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/noether_roscon_2024/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
    --skip-keys libvtk

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/noether_roscon_2024 \
  source /opt/ros/${DISTRO}/setup.bash \
  && cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args \
  && rm -rf build log

# Set the entrypoint to source the workspace
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
