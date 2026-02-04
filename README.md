# rob521-labs

This repo is for ROB521 Labs.

# Docker Setup (ROB521 Instructions) --- For Windows Users

1.  Install Docker: https://docs.docker.com/get-started/get-docker/ (also install WSL)
2.  Install the Docker VSCode extension: https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker
3.  Run the command in the `build_docker_file.sh` file. This builds the image (uses bash).
4.  Run the command in the `run_docker_container.sh` file. This runs the container and opens a terminal session in that environment. You should see the repo's files mounted in the workspace.

Note: For Intel/AMD graphics, replace `--gpus all` with `--device=/dev/dri:/dev/dri`

5.  If you stopped the container, run `docker start -ai rob521_container` to get it running in your terminal again.

## Getting a GUI with Docker using Xming

1.  Download Xming: https://sourceforge.net/projects/xming/
2.  Open XLaunch and select: Multiple windows, Display number 0, Start no client, Clipboard and No Access Control, finish!
3.  Note, in `run_docker_container.sh` I changed the env argument for this to work (just a Windows thing...)

## Setup ROS

1.  `cd /workspace`
2.  `catkin_make`
3.  `source devel/setup.bash`

If everything is working (make sure XLaunch is running), run gazebo: `roslaunch turtlebot3_gazebo turtlebo3_empty_world.launch`

## Other Notes

- In the container environment, you should see your Windows files included under

## Python Virtual Environment (optional, for keeping packages organized)

Assumes you are in the container environment.

1.  Install pip: `sudo apt update`, `sudo apt install python3-pip`
2.  Install venv: `sudo apt install python3.8-venv`
3.  Provided Dockerfile builds image running Python 3.8.10 (check by running `python3 --version`)
4.  In the location where you want to start a Python Virtual Environment, run `python3 -m venv ./venv`
5.  Activate the venv: `source ./venv/bin/activate`
6.  You should see `(venv)` at the start of your terminal prompt.
7.  Now you can use pip to install python packages without adding them to your system globally.
8.  Run `deactivate` to exit the venv.
