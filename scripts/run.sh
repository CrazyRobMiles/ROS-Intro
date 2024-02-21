docker run -it \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/$USER/ROS-Intro:/home/$USER/ROS-Intro" \
  --network=host \
  --ipc=host \
  --name=panel_demo \
  --device /dev/gpiomem0:/dev/gpiomem0 \
  --device /dev/mem \
  --user="$(id -u):$(id -g)" \
  ros2-foxy-gazebo:arm64
