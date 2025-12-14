FROM stereolabs/zed:4.0-runtime-cuda12.1-ubuntu22.04

CMD ["/bin/bash"]

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y --no-install-recommends tzdata && \
    ln -sf /usr/share/zoneinfo/Asia/Kuala_Lumpur /etc/localtime && \
    echo "Asia/Kuala_Lumpur" > /etc/timezone && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends ca-certificates curl dirmngr gnupg2 && \
    rm -rf /var/lib/apt/lists/*

RUN curl -L -s -o /tmp/ros2-apt-source.deb https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.jammy_all.deb && \
    echo "1600cb8cc28258a39bffc1736a75bcbf52d1f2db371a4d020c1b187d2a5a083b /tmp/ros2-apt-source.deb" | sha256sum --strict --check && \
    apt-get update && \
    apt-get install /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-ros-core=0.10.0-1* && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

RUN apt-get update && \
    apt-get install --no-install-recommends -y build-essential git python3-colcon-common-extensions python3-colcon-mixin python3-rosdep python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-ros-base=0.10.0-1* && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-perception=0.10.0-1* && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends libcublas-12-0 libcusolver-12-0 libcurand-12-0 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-rviz2 ros-humble-xacro && rm -rf /var/lib/apt/lists/*
