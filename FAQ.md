# FAQ
#### Why do I get an error when I try to run my Docker image?
This might be because there is already an image with that name on your machine. The **run** command creates a new container and starts it running. If the container already exists the run command will fail. Docker can create a brand new name for successive containers but the run command in this exercise always creates a container called **ros_demo**. Once a container has been created you can use the Docker start command to resetart it:
```
docker start -i ros_demo
```
Note that if this command fails you will need to use the run command to run the Docker image and make a container. This command is a bit complex so I've created a script for you to do it. The scripts are in the *scripts* folder in the *rob_ws* folder.
 
```
source scripts/run_image.bash
``` 
This script always creates a container called *ros_demo*.

### Why have things got weird with ROS?
If you use the start command above in more than one terminal session on your desktop you get a bunch of terminal sessions all sharing a single connection to a Docker container. This does not go well. If you already have a container running you can execute a new shell connected to the container by using the exec command as shown below. 
```
docker exec -it ros_demo /bin/bash
```
You can use this command to connect multiple terminal sessions to docker. 
### How do I stop my Docker container?
You can use the command exit to stop the container:
```
exit
```
If use this command in a session created by running or starting a container the container will be stopped and any exec sessions will also close. If you use this command in a session started using the exec command it will just close that session. 

You can use the start command to restart a container. Note that when you exit a container it is "frozen in time" and you can resume that container by starting it.
### Why are some commands missing from my container?
The Docker image that runs in a container starts with a very "bare bones" version of Linux and then adds to it. This means that not all commands are available. You can install them inside the image, but you need to remember that these will vanish when the container is deleted. 
### What happens to files I create in my container?
The run command you can see above maps the rob_ws folder into the Docker container. This is the folder that contains the sample application. Changes that you make to the contents of this folder in the container will be reflected in the rob_ws folder on your machine. 

Any other files you create in the container will persist in that container. If you remove the container (perhaps by pruning it) you will lost all those changes. 
### What is the difference between run, start and exec in Docker?
The run command takes an image and runs a new container. If you have specified a name for the container, Docker will check to see if there is already a container with that name. If there is 
### Can I run Visual Studio Code in the container?
You would have to install Visual Studio Code in the container to be able to use it there. However, there is no need. You can use Visual Studio Code in your Pi to work on files in the ROS-Intro folder and these changes will refect into the container.
### Why won't graphical ROS applications work in the container?
Make sure that you add local to xhost before you start the container. Issue this command to do this. 
```
xhost +local:
```
### How do I stop a Docker container?
If you want to stop a container you can use the stop command:
```
docker stop ros_demo
```
This stops the container. You can restart it later using the docker start command. 
### How do I get rid of inactive Docker containers?
They don't take up much memory, but if you want to get rid of all your stopped containers you can use the container prune command:
```
docker container prune
```
Note that this will not remove running containers.
### How do I find out what Docker containers are running?
```
docker container ls
```
This lists all the active containers on your machine.
### How do I find out what Docker containers I have?
```
docker container ls -a
```
This lists all the containers on the machine and tells you when they were last run. It includes containers that are not presently running.
### Why don't my ROS commands work in my package?
Remember that before you can run ROS commands on your package you need to run a script to set up your shell:
```
source install/setup.bash
```
The setup.bash file is created automatically when the package is built. If you don't run it to configure your shell you will find that none of the nodes will be picked up.
### Why have I run out of file space?
Docker creates a cache of files it uses when it builds a new image. This is nice if it saves reloading them next time you build the image but nasty if it means that you run out of file space.
``` 
docker system prune -f
```
The command above will ask docker to remove all cached files. 
### Why won't any of my graphical applications work?
You might get errors when you try to run a program such as rviz2 in the ROS docker environment. This is frequently (at least for me) when I've forgotten to allow remote processes access to the windowing environment. Use this command before you start the container running. 

```
xhost +local:
```
### Why do I get an error when I try to drive the motors?
You might get an error when you try to move the motors becuase the ROS program controlling the motors will try to access the i2c device which is usually restricted. You can solve this with the command:
```
sudo chmod 666 /dev/i2c-1
```
This makes the i2c-1 device accessible by all. You will have to issue this command after your have opened your docker image.
### Why won't my Docker image work on my computer?
Docker is great at hiding the particular version of operating system that you are using on your machine. But even docker can't deal with the differences between different type of processor chips. For example, you can't run the ROS image above on a PC compatible device. However, you should be able to customise the docker file to build a version of ROS for you which would run on most any PC. 
