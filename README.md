# ERC Drone Perception — OAK-D Pro probe detection (ROS2 Humble, Docker)
Detects yellow-green probes + ArUco (5x5, IDs 51-64), tracks with persistent IDs,
outputs 3D positions on /erc/probe/poses (camera optical frame, ROS_DOMAIN_ID=42).

## Fresh-machine setup (Ubuntu 22.04 / Jetson)
1. sudo apt install docker.io && sudo usermod -aG docker $USER  (re-login)
2. If dockerd fails on Jetson (iptables raw table):
   echo '{ "iptables": false, "bridge": "none" }' | sudo tee /etc/docker/daemon.json
   sudo systemctl restart docker      # note: containers must use --network host
3. Camera udev rule (host):
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger   # then replug camera
4. git clone <this repo> ~/erc && cd ~/erc
5. docker build --network host -t erc_perception .
6. ~/erc/run_erc.sh  → inside: colcon build --symlink-install  → exit

## Daily use
START: ~/erc/start.sh      LOGS: docker logs -f erc      STOP: docker stop erc
SHELL: docker exec -it erc bash
Host ROS: export ROS_DOMAIN_ID=42; ros2 daemon stop (first time)
Headless check: ros2 topic hz /erc/probe/image/compressed ; ros2 topic echo /erc/probe/poses --once

## Calibration notes (CAMERA-SPECIFIC)
probe_node.py contains intrinsics + DISP_OFFSET for OUR OAK-D Pro at 720p.
New camera or resolution => re-read /oak/rgb/camera_info + redo depth calibration.
probe_template_*.npy = captured probe silhouettes (Hu matching) — part of the system.
