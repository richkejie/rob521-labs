# rob521-labs

This repo is for ROB521 Labs.

# Using ROS-Noetic with Docker (for Windows users)

1.  Install Docker: https://docs.docker.com/get-started/get-docker/ (also install WSL)
2.  Install the Docker VSCode extension: https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker
3.  After installing the extension, go to the root of this repo (where the Dockerfile is located)
4.  Right click on the Dockerfile in VSCode and click 'Build Image'. Optionally, run: `docker build -t rob521-ros_noetic .` (the `.` is important, specifies current directory) (`rob521-ros_noetic` is the image name, can be whatever you want)
5.  `docker run --name ros_container --user=user --env=DISPLAY=host.docker.internal:0 --volume="$(pwd):/mnt" --restart=no --runtime=runc --volume="C:\\:/mnt/c" --network=host -t -d rob521-ros_noetic
`
6.  At this point, you can open Docker Desktop (just search for Docker in the start menu), and you will see your image and also your container running.
7.  You can run `docker ps` to see what containers are running.

## Getting a GUI with Docker using Xming

1.  Download Xming: https://sourceforge.net/projects/xming/
2.  Open XLaunch and select: Multiple windows, Display number 0, Start no client, Clipboard, finish!
3.  With the container running, run terminator: `terminator -u&` (run this in the terminal/exec tab of the container in Docker Desktop)

## Setup ROS

1.  In the terminator window, run: `source /opt/ros/noetic/setup.bash` (can add this to .bashrc)
2.  Install rviz: `sudo apt-get install ros-noetic-rviz`
