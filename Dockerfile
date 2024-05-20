# Get the base image from the docker hub (osrf/ros:noetic-desktop-full)
FROM osrf/ros:noetic-desktop-full
# To have a non-interactive build
ARG DEBIAN_FRONTEND=noninteractive
# Install the necessary packages
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros \
    ros-noetic-ros-controllers \
    terminator \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-velodyne-simulator \
    git \
    wget \
    bash-completion \
    && apt-get upgrade -y

# enable auto bash completion
# Comment out all the lines in /etc/apt/apt.conf.d/docker-clean
RUN sed -i 's/^/#/g' /etc/apt/apt.conf.d/docker-clean && \
    # Uncomment the last 3 lines in /root/.bashrc
    tac /root/.bashrc | sed '1,3s/^#//g' | tac > temp && mv temp /root/.bashrc && \
    # Update the bashrc file
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    # source the bashrc file
    /bin/bash -c "source /root/.bashrc"

# Create a workspace for the gem robot
RUN mkdir -p /home/ubuntu/gem_ws/src && cd /home/ubuntu/gem_ws/src && \
    git config --global user.email "you@example.com" && \
    git config --global user.name "Your Name" && \
    # Clone the package: POLARIS_GEM_e2
    git clone --depth 1 https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git
# Apply the patch to the POLARIS_GEM_e2 package
RUN mkdir -p /home/ubuntu/RequiredPatches && \
    wget "https://www.dropbox.com/scl/fo/hr304092kolzn4g051y6q/AAeHs1VD27GRIaYRoNm_i5w?e=1&preview=simu_update.patch&rlkey=nvmbw6is5c0pe0633ynlqhh2k&st=3mpnqr8i&dl=1" -O /home/ubuntu/RequiredPatches/gem_update.zip && \
    cd /home/ubuntu/RequiredPatches && \
    unzip /home/ubuntu/RequiredPatches/gem_update.zip -x / -d gem_update && \
    cd /home/ubuntu/gem_ws/src/POLARIS_GEM_e2 && \
    git am /home/ubuntu/RequiredPatches/gem_update/simu_update.patch
# Build the POLARIS_GEM_e2 package
RUN cd /home/ubuntu/gem_ws && \
    # Build the package
    /bin/bash -c '. /opt/ros/noetic/setup.bash ; catkin_make -j1 '
# Manage DISPLAY access
RUN mkdir -m 0700 /tmp/runtime-root && \
    apt-get install -y \
    dbus-x11 \
    libgl1-mesa-dri \
    libgl1-mesa-glx
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
ENV DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
ENV LIBGL_ALWAYS_SOFTWARE=1

# Create a workspace for the LIO-SAM-Localization
RUN mkdir -p /home/ubuntu/lio_sam_localization_ws/src && cd /home/ubuntu/lio_sam_localization_ws/src && \
    # Clone the package: LIO_SAM
    git clone --depth 1 https://github.com/harshalkataria/LIO-SAM-Localization.git
# Build the POLARIS_GEM_e2 package
RUN cd /home/ubuntu/lio_sam_localization_ws && \
    apt install software-properties-common -y && \
    add-apt-repository ppa:borglab/gtsam-release-4.0 && \
    apt install libgtsam-dev libgtsam-unstable-dev -y && \
    # Build the package
    /bin/bash -c '. /opt/ros/noetic/setup.bash ; catkin_make -j1 '

# Keep the container running
CMD tail -f /dev/null

WORKDIR /
