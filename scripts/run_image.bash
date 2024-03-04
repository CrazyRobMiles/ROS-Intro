docker run -it \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/$USER/ROS-Intro/rob_ws:/home/$USER/rob_ws" \
  --network=host \
  --ipc=host \
  --name=ros_demo \
  --privileged \
  --user="$(id -u):$(id -g)" \
  --device=/dev/ttyACM0:/dev/ttyACM0 \
  --device=/dev/ttyACM1:/dev/ttyACM1 \
  --device=/dev/ttyACM2:/dev/ttyACM2 \
  ros2-foxy-gazebo:arm64
