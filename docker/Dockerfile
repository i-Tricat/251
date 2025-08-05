FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    build-essential \
    ros-noetic-tf \
    ros-noetic-pcl-conversions \
    ros-noetic-rtcm-msgs \
    ros-noetic-nmea-msgs\
    && rm -rf /var/lib/apt/lists/*

# rosdep update만 수행 (이미 init 되어 있음)
RUN rosdep fix-permissions && rosdep update

COPY ../catkin_ws /root/catkin_ws
RUN rm -rf /root/catkin_ws/build /root/catkin_ws/devel

# ROS 환경 설정 및 빌드
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make"

# 편의 설정
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "alias cm='cd ~/catkin_ws && catkin_make'" >> /root/.bashrc && \
    echo "alias sb='source ~/.bashrc'" >> /root/.bashrc

CMD ["bash"]

