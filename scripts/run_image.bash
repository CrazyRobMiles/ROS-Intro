docker run -it \
  --privileged \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/$USER/ROS-Intro/rob_ws:/home/$USER/rob_ws" \
  --network=host \
  --ipc=host \
  --name=ros_demo \
  --user="$USER" \
  ros2-foxy-gazebo:arm64
