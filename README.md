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
