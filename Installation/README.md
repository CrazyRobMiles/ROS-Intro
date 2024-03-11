# ROS Installation
You can use these steps to take an "empty" Raspberry Pi and get it running ROS. The process has been tested on a Raspberry Pi 4 with 4Gbytes of memory and on a Raspberry Pi 5 with 8Gbyes of memory. 
## Installing Raspberry Pi OS
A particular version of ROS is tied to a specific operating system, but we are going to be running ROS via a system called Docker which hides the underlying operating system from software running on it. You can use the latest Raspberry Pi OS. Just make sure to install the 64 bit version. 
### Make an sd card for your Pi
The first thing we are going to do is create a microSD card containing the operating system. A 32Gbyte card works OK, although if you want to have multiple ROS images on your machine you might find 64Gbyte more flexible.
1. Start the Raspberry Imager program on your machine and put the microSD card into the drive on your computer. 
1. Select the OS to be installed. Make sure you install a 64 bit operating system. 
1. Set the advanced options in the Imager program. Now make the folling settings. 
    1. Hostname (I called mine "bigpi")
    1. Enable SSH (password authentication)
    1. User: xxxx Password: xxxxxx (don't use x, put in your own words) I created a user called rob (it's used in the ssh command a few steps down)
    1. Set Wi-Fi credentials and locale
    1. Save the options so you can use them again if you need to.
1. Make the image.
### Update your Pi Operating System
1. Remove the SD card from your computer, plug it into your Raspberry Pi and turn the Pi on. Wait a few minutes until the green led on the Pi seems to settle down a bit.
1.	Now log in from your computer. You'll need a command prompt to do this. The commands given are from Microsoft PowerShell. You will be setting up a secure shell, so use the command ssh:
    ```
    ssh bigpi.local -l rob
    ```
1.	Use the name of the computer and the username that you set in the steps above. Say yes to set up the connection if it is the first time you've connected.
1.	Give your password when it is requested.
1.  Once you've logged in you should update your computer:
    ```
    sudo apt update
    sudo apt upgrade
    ```
1.  Now reboot to get all the new software goodness.
    ```
    sudo reboot
    ```
### Setup secure sockets
You might want to connect to your new Pi from another machine. This saves you connecting a keyboard, screen and mouse to the it. To do this you need to set up "secure sockets" which implement a network connection between the Pi and your machine. 

1.	Now set up SSL to allow your remote machine to connect to the PI.
    1. **On the Pi**
        1. Check for existing SSH
            ```
            ls ~/.ssh
            ```
        1. If this comes back with some files they are already there. Otherwise:
            ```
            ssh-keygen
            ```

        1.	Use the defaults
    1.	**On your remote machine**
        1.	Copy your keys over:
            ```
            cat ~/.ssh/id_rsa.pub | ssh rob@bigpi.local 'mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys'
            ```

        1. I used the username and the address of the pi I’m setting up. Enter the password for the user (in my case rob)
    1. Now you can test the connection. Open another terminal window on your machine and connect directly to the pi with no password:
        ```
        ssh bigpi.local -l rob
        ```
    1. This should open a remote terminal session with no password required. If this works you will be able to use remote development with Visual Studio Code.
### Enable remote desktop
Remote desktop makes it very easy to work with your Pi. You'll need to download and install VNC Viewer onto your computer, but it means that you can get full graphical control over your computer remotely. The Pi display appears in a window on your desktop and you can use your mouse and keyboard to interact with your Pi.
1.  Start raspi-config:
    ```
     sudo raspi-config
    ```
    You can move the "cursor" arount the screen by using the arrow keys on your keyboard. The tab key will step you between option. Press the enter key to select something. 
1.  Enable VNC on your Pi:

    1. Select option 3 - Interface Options
    1. Select the I2 VNC option
    1. Select "Yes" in the menu. This will return you to the main raspi-config menu for the next step, setting the resolution of the VNC screen.

1.  Set the resolution of the VNC screen
    
    You can set the size of the screen that is displayed on your destop. A large screen is nice, but a smaller screen won't load up the Pi quite so much.

    1.  Select option 2 - Display Options
    1.  Select option D3 VNC Resolution
    1.  Select a resolution that works for you. I like 1600x1200. If you pick a lower resolution you may have problems using Visual Studio Code to open files. For some reason file open dialogue box is too big to fit on a lower resolution screen.  
    1.  Use Select to select the resolution and return to the top screen. Once you've set up VNC and configured the resolution you can select Finish on the main sudo-config screen to apply the settings. You may be asked to reboot the Pi. 

1.  Get the VNC Viewer program for your device. You can download it from [here](https://www.realvnc.com/en/connect/download/viewer/) 

1.   When you run the VNC Viewer you just need to enter the address of your Pi into the top of the program and press enter. The address of the pi will be "yourPiName".local. I called my pi bigpi, so the address I use is "bigpi.local". 
## Install Software
Now that we have the Raspberry Pi running we need to install some software to make it useful. You might find that some of these tools are already installed - depending on which operating system you installed. You can do all this installation from the remote terminal session you opened using ssh in the previous step.
### git
1. First we need to install git on your machine:
    ```
    sudo apt-get install git -y
    ```
1. Next we need to configure the username and email values for the Git installation on the machine. Use your email and username as appropriate. 
    ```
    git config --global user.name "Rob Miles"
    git config --global user.email rob@robmiles.com
    ```
### Visual Studio Code
A lot of what we are going to do will require the Visual Studio Code editor. Let's install it.
```
sudo apt-get install code
```
This will take a little while. Note that you can't run Visual Studio Code from the command line, you'll need to be using a windowed environment to run it. You can plug your monitor, keyboard and mouse into your Pi and do this directly, but you might find it more convenient to use remote desktop and link to your Pi from another machine (which already ahs a keyboard, mouse and screen) over the network. So let's set that up.
### Docker
Docker is a fantastic way to deploy programs. Without Docker these instructions would be a lot longer and take you through all the steps required to install ROS. And then you might find it doens't work on your particular Raspberry Pi operating system. Docker makes it possible for any Pi running a 64 bit operating system to be able to run ROS.

We are going to build a Docker image which has been setup with ROS and lots of other things too. We have to go through a series of steps to install Docker, but it is totally worth it. Perform all these steps from a terminal on your Pi. 

1.  First we get the docker install script:

    ```
    curl -fsSL https://get.docker.com -o get-docker.sh
    ```

    This fetches a script from docker which will perform the installation for us. The script is stored in the file **get-docker.sh** in our home directory.
1. Next we run the script:

    ```
    sudo sh get-docker.sh
    ```
    After a while the install will complete. 
1. Now we need to join the docker group. A group is a security mechanism. Things in particlar groups can be granted particular permissions. Components in Docker are in the docker group, so we need to join this group. We do this once for the machine. 
    ```
    sudo usermod -aG docker rob
    ```
    I did this for the user **rob** (which is the user I created when I built the Pi). If you did'nt call yourself rob you will need to use your name instead.
1.  We need to log out and log back in again so that the new group membership is recognised:
    ```
    exit
    ssh bigpi.local -l rob
    ```
    The exit commmand will send us back to the terminal, then use the ssh command to reconnect.
1.  We can now give a docker command to make sure that all is well.
    ```
    docker ps
    ```
    This lists all the active docker containers
    ```
    CONTAINER ID   IMAGE     COMMAND   CREATED   STATUS    PORTS     NAMES
    ```
    The list is empty becuase nothing is running under docker yet.

1.  We can prove that Docker is working properly by downloading a docker image and running it. 
    ```
    docker run hello-world
    ```
    Docker will try to run the hello-world image on your machine. If it doesn't find this image it is automatically "pulled" from the Docker library. This should print a "Hello from Docker!" message along with some other text.

1.  If you do the docker ps command we saw earlier you might think you'd see the hello-world docker, but you don't because it has already ended. But we can get a list of all the things that docker has run by adding an -a to the end of the command:
    ```
    docker ps -a
    ```
This will show the process that you were running. There is a lot more to Docker, but we are just going to focus on those parts which make it possible to run ROS on your Pi.
## Build a Docker image for ROS
Now we need to use Docker to build an which contains ROS. Docker can build images under the control of a Dockerfile which specifies the starting image and then describes what needs to be done to make the image useful. In our case we are going to start with an empty Linux image and add ROS and a few other useful programs to it. The Dockerfile we are going to use can be found on GitHub (it's in this repository). So the next thing we do is clone this repository onto your Pi.  

```
git clone https://github.com/CrazyRobMiles/ROS-Intro
```
This will create a folder called ROS-Intro containing this README file, a Dockerfile, some useful scripts and the ROS exercise described in the HackSpace article. Change your working directory to this folder.
```
cd ROS-Intro
```
The folder contains a Dockerfile which we can use to build a Docker image that contains ROS and all the graphical tools that it uses. 

Perform the following command: (change the **USERNAME** from **rob** to the username you created when you built your system )
```
docker build --build-arg USERNAME=rob --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) -t ros2-foxy-gazebo:arm64 .

```
This will generate a Docker image called **ros2-foxy-gazebo:arm64**. It will take a while to do this, so grab a cup of coffee. Note that you only have to do this at the start. 
## Run ROS in Docker
Once the image has been built you will be able to run it using Docker. Docker will create a "container" which is a process running the image we have just created. The command to run an image is the docker run command. It can be supplied with options to control the machine resources that code in the container has access to. We want our ROS programs to be able to access files on the hard disk, use network connetions and access hardware on the Raspberry Pi. The command you can see below does all this. Copy it out of this page and run it in a terminal window on your Pi. 

### ROS Graphics
Some of the ROS components (for example rviz2, gazebo and rqt) are graphical applications. If you want to use these you should enter the command below in a command prompt running on the Raspberry Pi desktop. 
```
xhost +local:
```
This allows the container to connect to the X server on your Pi. You need to do this once, at the start of your ROS session - although you will have to do it again next time you start your Pi. You don't need to use the graphical interface if you just want to use ROS to control your robot. In that case you could look for a Docker image which just contains ROS without the graphical elements. 

### ROS Hardware
If you want your ROS application to be able to interact with hardware on your Pi you will need to give it access to the relevant devices. The motor example for the HiWonder robot used in the article needs to have access to i2c, so you need to run the command below to enable this. 
```
sudo chmod 666 /dev/i2c-1
```
## 

Let's start by working with the project created in the magazine article. This implements a simulation of a robot control panel with buttons and lights along with a simple motor controller. You can find this exercise [here](/MessageDemo.md).