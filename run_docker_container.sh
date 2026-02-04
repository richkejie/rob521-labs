docker run -it --net=host --env="DISPLAY=host.docker.internal:0.0" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd)src:/workspace/src/rob521_labs_mount" --gpus all --name rob521_container rob521-labs-img


# --env="DISPLAY"
    # --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    # --volume="$HOME/rob521_ws/src:/workspace/src/rob521_labs_mount" \