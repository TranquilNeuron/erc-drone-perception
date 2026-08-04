FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-depthai-ros \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install "numpy<2" "opencv-contrib-python==4.9.0.80" "depthai<3"

ENV ROS_DOMAIN_ID=42

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/erc_ws/install/setup.bash ] && source /root/erc_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/erc_ws
CMD ["bash"]

