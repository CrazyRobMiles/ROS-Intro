# Messaging Demo
This exercise shows you the fundamentals of ROS messaging and how two different nodes can communicate by use of the publish-subscribe messaging provided by ROS.

**Before you perform this exercise you need to make sure that you have installed ROS on your Pi according to the steps [here](Installation.md).**

## Start the ROS Docker image
1. Start your Pi and open the Pi desktop. Now open a command prompt window on the desktop. 

1. We might want to use the *rviz2* tool to visualise the topics published by our ROS apps. To do this we need to enable the windowing environment. Enter the command below to do this.
    ```
    xhost +local:
    ```
1. Now change into the ROS-Intro folder which was created during the installation process.
    ```
    cd ROS-Intro
    ```
1. The ROS-Intro folder contains a folder called *scripts* which contains a set of scripts you can use to start and stop ROS in Docker. We want to start our ROS image running inside a container, so we use the run_image.bash script to do this. Type in the command below. It tells your terminal shell (the thing you are presently interacting with) to open the script and run it.
    ```
    source scripts/run_image.bash
    ``` 
1. You should see your prompt change to the prompt for the Docker image which is now in control. 

## Build the application source files
You're not on your Pi any more. Now you're connected to Docker which is running the image that contains ROS. This is a bit like stepping into a parallel universe, but without the special effects. If this command fails take a look in the [faq](FAQ.md).

Our "parallel universe" contains ROS. I've also arranged for some parts of your Pi universe to be projected into into the ROS universe; specifically a folder called *rob_ws*. This contains the robot workspace that we are going to use. 

1.  Change to the rob_ws folder:

    ```
    cd rob_ws
    ```
1. The first thing we need to do is build the application from the source files in the package. Type in the following command
    ```
    colcon build --packages-select front_panel
    ```
    This finds the front_panel nodes in the src folder and uses them to build the code for them. The front_panel package contains nodes for buttons and a display. After a while you will get a summary message telling you that the build has finished.
## Run the display application
Now that we have our code built we can run it. We are going to start by running the button display application. In a real robot this could change the colour of a light or print a message on a screen. For our demo we are just going to display incoming messages on the terminal.
1.  First we setup the command prompt so that ROS commands can find the elements in the ROS package that we are using. Enter the following command to do this.
    ```
    source install/setup.bash
    ```
1.  Now we can start one of our robot processes. We have two processes, one sends messages (buttons) and the other displays them. We can start the display application first using the following command
    ```
    ros2 run front_panel display
    ```
    The program will start up but you won't see any messages yet. That's because the button process isn't running. To do this you will need to open another command prompt and connect it to the Docker container running ROS.
## Run the button application
1.  Start by opening another command prompt on the Raspberry Pi desktop.
1.  We need to connected this command prompt to the Docker container running ROS. Change to the example folder:
    ```
    cd ROS-Intro
    ```
1. Now we need to execute another shell environment inside the Docker container. Use the fullowing comand to do this. 
    ```
    docker exec -it ros_demo /bin/bash
    ```
1.  Change to the rob_ws folder:
    ```
    cd rob_ws
    ```
1.  Now setup the command prompt so that ROS commands can find the elements in the ROS package that we are using.
    ```
    source install/setup.bash
    ```
    The buttons program presses a bunch of random buttons - in a finished application the button presses messages would be triggered by hardware. To start the buttons process do the following command:
    ```
    ros2 run front_panel buttons
    ```
    The buttons node will start running and publishing random button messages.  You will see these appearing in the button application terminal and also in the display terminal.

    You've done it! You've got a ROS application running on a Raspberry Pi and using publish/subscribe to communicate. 
## Investigate the application
Now we can use some ROS tools to take a look at the running application. To do this you will need to open up a third terminal on your desktop.
1.  Start by opening another command prompt on the Raspberry Pi desktop.
1.  We need to connected this command prompt to the Docker container running ROS. Change to the example folder:
    ```
    cd ROS-Intro
    ```
1. Now we need to execute another shell environment inside the Docker container. Use the fullowing comand to do this. 
    ```
    docker exec -it ros_demo /bin/bash
    ```
1.  Change to the rob_ws folder:
    ```
    cd rob_ws
    ```
1.  Now setup the command prompt so that ROS commands can find the elements in the ROS package that we are using.
    ```
    source install/setup.bash
    ```
1.  You can now use some ROS commands to investigate the running system. Try these:
    ```
    ros2 node list
    ros2 topic list
    ros2 service list
    ```
## Use RQT to investigate topics and messages
The rqt application is highly configurable and lets you create a dashboard for your robot application. You can view messages and even inject messages of your own. You'll be using the same terminal window that you used in the previous exercise. 
1.  Start the rqt program:
    ```
    rqt
    ```
    Now you can use the **Plugins>Topics** menu to investigate the messaging. 
## Tidying up
You can stop any executing node by holding down CTRL and pressing C. You can exit any Docker terminal by using the command **exit**. If you exit the terminal you use to start or run the container all the other command sessions will stop automatically. Any files you created in the Docker container will remain. Read the [FAQ](FAQ.md) for more details about how Docker works. 






