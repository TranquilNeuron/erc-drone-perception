#!/bin/bash
docker run -it --rm \
  --network host --ipc=host --privileged \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/erc_ws/fastdds_udp.xml \
  -v /dev/bus/usb:/dev/bus/usb \
  -v ~/erc/erc_ws:/root/erc_ws \
  --name erc erc_perception
