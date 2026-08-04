#!/bin/bash
docker run -d --rm \
  --network host --ipc=host --privileged \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/erc_ws/fastdds_udp.xml \
  -v /dev/bus/usb:/dev/bus/usb \
  -v ~/erc/erc_ws:/root/erc_ws \
  --name erc erc_perception \
  bash -c "source /opt/ros/humble/setup.bash && source /root/erc_ws/install/setup.bash && ros2 launch erc_perception perception.launch.py"
echo "ERC perception starting. Logs: docker logs -f erc   Stop: docker stop erc"
