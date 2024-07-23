### Using the stack:
#### Automatically starting the software stack on boot AKA hands-free operation:
#### Note: When running/testing the autonomy mode, please make sure to connect the battery to the VESC
1. The setup you performed during installation using `mushr_install.bash` creates a `systemd` service that can automatically start the docker on boot. However, this service must be enabled by you by executing:
```bash
sudo systemctl enable mushr_start.service
```

2. To have the hound-core stack and the perception system start on boot, you will need to edit the [mushr_noetic](https://github.com/prl-mushr/mushr/blob/noetic-HOUND/mushr_utils/install/mushr_noetic) docker file by commenting out the bash starting lines and uncommenting the line that starts docker in a detached mode.
In essence, you need to change this:
```bash
xhost +local:docker
docker-compose -f $MUSHR_INSTALL_PATH/$MUSHR_COMPOSE_FILE run -p 9090:9090 --rm mushr_noetic bash
xhost -local:docker

# docker-compose -f $MUSHR_INSTALL_PATH/$MUSHR_COMPOSE_FILE run -p 9090:9090 -d --rm mushr_noetic /root/catkin_ws/src/hound_core/entry_command.sh
```
To this:
```bash
# xhost +local:docker
# docker-compose -f $MUSHR_INSTALL_PATH/$MUSHR_COMPOSE_FILE run -p 9090:9090 --rm mushr_noetic bash
# xhost -local:docker

docker-compose -f $MUSHR_INSTALL_PATH/$MUSHR_COMPOSE_FILE run -p 9090:9090 -d --rm mushr_noetic /root/catkin_ws/src/hound_core/entry_command.sh
```

Now, when you reboot the system, it should automatically start the docker, as well as the sensor nodes and the elevation mapping stack. 