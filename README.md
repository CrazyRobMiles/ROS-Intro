# ROS
An introduction to running ROS on a Raspberry Pi.

You can use these steps to take an "empty" Raspberry Pi and get it running ROS. The process has been tested on a Raspberry Pi 5 with 4Gbytes of memory.
## Install Raspberry Pi OS
You can use the latest Raspberry Pi OS. Just make sure to install the 64 bit version.
### Make an sd card for your Pi
The first thing we are going to do is create a microSD card containing the operating system. A 32Gbyte card works OK.
1. Start the Raspberry Imager program on your machine and put the microSD card into the drive on your computer. 
1. Select the OS to be installed. Make sure you install a 64 bit operating system. 
1. Set the advanced options in the Imager program. Now make the folling settings. 
    1. Hostname (I called mine "bigpi")
    1. Enable SSH (password authentication)
    1. User: xxxx Password: xxxxxx (don't use x, put in your own words) I created a user called rob (it's used in the ssh command a few steps down)
    1. Set Wi-Fi credentials and locale
    1. Save the options so you can use them again if you need to
1. Make the image.
### Configure your Pi Operating System
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
1.	Now set up SSL to allow your remote machine to connect to the PI.
    1. On the Pi
        1. Check for existing SSH
            ```
            ls ~/.ssh
            ```
        1. If this comes back with some files they are already there. Otherwise:
            ```
            ssh-keygen
            ```

        1.	Use the defaults
    1.	On your remote machine
        1.	Copy your keys over:
            ```
            cat ~/.ssh/id_rsa.pub | ssh rob@bigpi.local 'mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys'
            ```

        1. I used the username and the address of the pi I’m setting up. Enter the password for the user (in my case rob)
    1. Now you can test the connection. Open another terminal window on your remote machine and connect directly to the pi with no password:
        ```
        ssh bigpi.local -l rob
        ```
    1. This should open a remote terminal session with no password required. If this works you will be able to use remote development with Visual Studio Code.
## Install software
Now that we have the Raspberry Pi running we need to install some software to make it useful. You might find that some of these tools are already installed - depending on which operating system you installed. You can do all this installation from the remote terminal session you opened using ssh in the previous step.
### Install git
1. Now we need to install git on your machine:
    ```
    sudo apt-get install git -y
    ```
1. Next we need to configure the username and email values for the Git installation on the machine. Use your email and username as appropriate. 
    ```
    git config --global user.name "Rob Miles"
    git config --global user.email rob@robmiles.com
    ```
### Install Visual Studio Code
A lot of what we are going to do will require the Visual Studio Code editor. Let's install it.
```
sudo apt-get install code
```
This will take a little while. Note that you can't run code from the command line, you'll need to be using a windowed environment. You can plug your monitor, keyboard and mouse into your Pi and do this directly, but you might find it more convenient to use remote desktop and link to your Pi over the network. So let's set that up.
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
    1.  Select a resolution that works for you. I like 1280x720. Note that you can zoom the screen on your PC so it won't appear too small, you just won't be able to get quite as much text on it. 
    1.  Use Select to select the resolution and return to the top screen. Once you've set up VNC and configured the resolution you can select Finish on the main sudo-config screen to apply the settings. You may be asked to reboot the Pi. 

1.  Get the VNC Viewer program for your device. You can download it from [here](https://www.realvnc.com/en/connect/download/viewer/) 

1.   When you run the VNC Viewer you just need to enter the address of your Pi into the top of the program and press enter. The address of the pi will be "yourPiName".local. I called my pi bigpi, so the address I use is "bigpi.local". 
### Install Docker
We are going to run ROS inside Docker. This makes the configuration a lot easier.
1.  First we get the docker install script:

    ```
    curl -fsSL https://get.docker.com -o get-docker.sh
    ```

    This fetches a script from docker which will perform the installation for us. The script is stored in the file get-docker.sh in our home directory.
1. Next we run the script:

    ```
    sudo sh get-docker.sh
    ```
    After a while the install will complete. 
1. Now we need to join the docker group (which is where the docker daemons run).
    ```
    sudo usermod -aG docker rob
    ```
    I did this for the user rob (which is the user I created when I built the Pi)
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
    This should print a "Hello from Docker!" message along with some other text.

1.  If you do the docker ps command we saw earlier you might think you'd see the hello-world docker, but you don't because it has already ended. But we can get a list of all the things that docker has run by adding an -a to the end of the command:
    ```
    docker ps -a
    ```
    This will show the process that you were running. 
### Build a Docker image for ROS
Now that you have Git running on your machine you can copy this repository onto it so that you can use the resources in the file to set up ROS. 

```
git clone https://github.com/CrazyRobMiles/ROS-Intro
```
This will create a folder called ROS-Intro containing this README file and a Dockerfile. Change your working directory to this folder:
```
cd ROS-Intro
```
We are now going to build a Docker image that contains ROS and all the graphical tools that it uses. Perform the following command
```
docker build -t ros2-foxy-gazebo:arm64 .
```
This will generate a Docker image called ros2-foxy-gazebo:arm64. It will take a while to do this, so grab a cup of coffee. Note that you only have to do this at the start. Once the image has been built you will be able to just run it. 
When the image has been built you can run it with the following command:

```
docker run -it \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/$USER:/home/$USER" \
  --network=host \
  --ipc=host \
  --device /dev/gpiomem0:/dev/gpiomem0 \
  --device /dev/mem \
  --user="$(id -u):$(id -g)" \
  ros2-foxy-gazebo:arm64
```
This will start ROS running and you will find that your command prompt changes. Now we can issue ROS commands. Let's start by creating a simple project.


