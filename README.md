![](content/hound.png)
# HOUND
This repository serves as the cornerstone of the HOUND project. Note that as the HOUND project depends on several other repositories, this repository only contains instructions on installing the HOUND stack and instructions for running the minimal examples.

### Note: 
1) The installation requires a few reboots, please save and close any work before starting this process.
2) These instructions have only been tested on the Nvidia Jetson Orin series hardware running Jetpack 5.1.2 and on an Ubuntu x86_64 machine running ubuntu 20.04.
3) If you use the docker and find important packages useful for research please make a PR/Github issue! We'll update the docker ASAP, assuming the required package does not break existing stuff.
4) If you need something that breaks the rest of our stack in the docker, you'd modify the [docker generation](https://github.com/prl-mushr/mushr/tree/noetic-HOUND/mushr_utils/install) files as well as the [jetson-container's](https://github.com/prl-mushr/jetson-containers) related files and then build the docker from scratch (which takes a while).

### Hardware Requirements:
#### Setting up just the autonomy stack:
1) If installing on the HOUND, then [HOUND hardware](https://github.com/prl-mushr/hound_hardware). Otherwise, no additional hardware is required.
2) An internet connection

#### Requirements for running the BeamNG simulator:
1) The BeamNG simulator is run on a separate Windows computer. See [here](https://documentation.beamng.com/support/troubleshooting/requirements/) for the system requirements to run BeamNG.
2) Full duplex ethernet cable.
3) BeamNG also has experimental support for running the simulator on an Ubuntu host, however, this at the moment does not support the camera and LiDAR sensors.

### Environment setup for Running Autonomy stack on Ubuntu x86_64 (Autonomy stack installation not supported on MacOS or Windows. WSL instructions coming soon!):
Step 1: [Install docker](https://docs.docker.com/desktop/install/debian/#install-docker-desktop)

Step 2: [Install Nvidia Docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian)

### Environment setup for Jetson:
Follow the instructions [here](https://github.com/prl-mushr/hound_hardware) to set up your Nvidia Orin NX.

### Installing HOUND stack:
Step 1:
```bash
mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
git clone -b noetic-HOUND https://github.com/prl-mushr/mushr
```

Step 2: Executing the following command will prompt you to answer three questions
1) "Are you installing on robot and need all the sensor drivers? (y/n) " -- say yes, this should not produce a conflict even if you're on a desktop.
2) "download HOUND/field-robotics related repositories? (y/n) " -- say yes
3) "Build from scratch? (Not recommended, takes much longer than pulling ready-made image) (y/n) " -- say no.
Building the docker from scratch is possible but not recommended for now.
```bash
cd ~/catkin_ws/src/mushr/mushr_utils/install/
./mushr_install.bash
sudo reboot
```

Step 3: The following command is used to start the docker. On the first run, this will pull the docker image, which is approximately 20 GB in size, and so can take between 10 minutes and an hour depending on your internet connection speed.
```bash
mushr_noetic
```

Step 4: Once inside the docker, build the catkin workspace:
```bash
cd ~/catkin_ws/ && catkin build
```

This concludes the installation of the hound's autonomy stack.

### Installing the simulator integration:
The docker install pulls the BeamNG integration code, however, there are some additional setup steps. Follow the instructions on [BeamNGRL](https://github.com/prl-mushr/BeamNGRL) to set up the BeamNG simulator integration.
Note that you will need to git clone BeamNGRL on the Windows host as well to start the simulator on it.

### Testing the stack in HITL mode:
The HITL unit test is useful for testing your entire stack before you go for a real-world field test. It runs the autonomy stack in ROS as if it was running with real hardware data, communicating the control outputs to the simulator.

1) Make sure the computer running the autonomy stack and the one running the simulator are connected via the ethernet cable. On the computer running the autonomy stack:
```bash
ping <IP_ADDRESS_OF_WINDOWS_COMPUTER>
```
You should see a valid ping, with a latency of less than or equal to 1 millisecond

2) On the Windows computer, go to the directory containing BeamNGRL:
```bash
cd PATH/TO/BeamNGRL/examples
python boot_beamng.py
```

3) On the computer running the autonomy stack, start the docker: 
```bash
mushr_noetic
```

Now, run the following to start the HITL unit test:
```bash
cd ~/catkin_ws/src/hound_core
./unit_test.bash <IP_ADDRESS_OF_WINDOWS_COMPUTER>
```

You should see the following on your autonomy computer's screen:

![](content/hound_hitl.gif)

Note: The MPPI may not follow the desired trajectory exactly, and the car may spin out. This may happen because of the car being used by the HITL simulation has different tire parameters than the ones being assumed by the MPPI model.

#### In case the map looks weird/caved, you need to reset the map:
```bash
rosservice call /elevation_mapping/clear_map
```
This concludes running the HITL unit test.

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

### High level description of the components involved in the auto-start process: 

 - When you make the aforementioned change to the mushr_noetic file, The [entry_command.sh](https://github.com/prl-mushr/hound_core/blob/main/entry_command.sh) is executed upon entering the docker. It takes care of sourcing the workspace and launching [sensors.launch](https://github.com/prl-mushr/hound_core/blob/main/launch/sensors.launch) which starts all of the peripherals.
 
 - `sensors.launch` first starts [HAL_9000.py](https://github.com/prl-mushr/hound_core/blob/main/src/HAL_9000.py), which is the Hardware Abstraction Layer that manages publishing of transforms, setting/resetting sensor rates, managing data recording and a bunch of other miscellaneous tasks. This node publishes System-on-Chip diagnostics, such as CPU/GPU temperatures, frequencies etc. on `/SOC_diagnostics`. It uses the [HAL.yaml](https://github.com/prl-mushr/hound_core/blob/main/config/HAL.yaml) config file, which describes the sensor orientations, update rates, recovery actions and so on.
  
 - `sensors.launch` then uses [timed-roslaunch](https://github.com/prl-mushr/timed_roslaunch) to launch the sensor nodes and the elevation mapping system to make sure that peripherals don't run into conflicts over access to the USB port driver, and that the elevation mapping stack doesn't start too early. The peripheral driver nodes are:
   - `apm.launch` which starts the ardupilot interface to ROS
   - `X4.launch` which starts the YDLiDAR
   - `vesc_driver_node.launch` which starts the VESC driver
   - `realsense_d455_HOUND.launch` which starts the realsense camera
 - All of the above are started in auto respawn mode, which means that if the peripheral is physically disconnected and reconnected during operation, the node will automatically reboot the peripheral drivers. This can be particularly useful for hands-free operation in off-road environments where the car may crash and have a USB momentarily disconnect due to the impulse.
  
 - [hound_ll_control](https://github.com/prl-mushr/hound_core/blob/main/src/hound_ll_control.cpp) is the low-level controller that runs the speed control loop and the rollover prevention system, and is responsible for multiplexing the inputs from the operator and the autonomy stack. It publishes a diagnostic message on the topic `/low_level_diagnostics` that lets you know the wheel-speed targets, errors as well as when the system intervened to prevent a rollover. The last piece of information can be useful for debugging your own high-level controller.
   - It uses parameters from this [low_level_config_real.yaml](https://github.com/prl-mushr/hound_core/blob/main/config/low_level_Config_real.yaml)

 - [elevation_mapping_cupy_no_vis.launch](https://github.com/prl-mushr/elevation_mapping_cupy/blob/main/elevation_mapping_cupy/launch/elevation_mapping_cupy_no_vis.launch) launches the elevation mapping stack, which includes the raw elevation mapping node, map cropping node, and the inpainting node. Note that this node depends on pose estimates that come from the ardupilot board. [D455_parameters.yaml](https://github.com/prl-mushr/elevation_mapping_cupy/blob/main/elevation_mapping_cupy/config/D455_parameters.yaml) is the parameter file for the raw elevation mapping, and [this](https://github.com/prl-mushr/grid_map_occlusion_inpainting/blob/master/grid_map_occlusion_inpainting_ros/config/default.yaml) is the parameter file used by the inpainting node.

 - [vesc_to_odom.py](https://github.com/prl-mushr/hound_core/blob/main/src/vesc_to_odom.py) is a node that may be used to send wheelspeed measurements to the ardupilot board to improve accuracy on high-traction surfaces.

 - [osm_router.py](https://github.com/prl-mushr/hound_core/blob/main/src/osm_router.py) is the node that converts GPS waypoints into a path in the local reference frame. It can optionally use Open street maps to generate the paths (assuming an internet connection is available), but this option is generally not useful as the vehicle usually operates in off-road environments. One could write a high-level planner to replace this in the future.

 - [ntrip_client.launch](https://github.com/prl-mushr/ntrip_ros/blob/main/launch/ntripclient.launch) starts the NTRIP node and publishes RTCM3 corrections, which are subscribed to by the ardupilot node and sent to the GPS for obtaining RTK accuracy. Note that you will need to provide your own username, password and NTRIP stream name.

3. Running the default high-level controller: the default high-level controller uses Model Predictive Path Integral Controller. Although not currently demonstrated, a different controller could also be used, as the high-level control file is "split" into two parts, the [hound_hl_control.py](https://github.com/prl-mushr/hound_core/blob/main/src/hound_hl_control.py) and the [hound_mppi.py](https://github.com/prl-mushr/hound_core/blob/main/src/hound_mppi.py).
 
  - The `hound_hl_control.py` is effectively providing a "state" and "auxiliary information" as input and expecting "actions" as output. Here, the current goal position and speed limit are considered as the "auxiliary information". All of this happens effectively in this [code block](https://github.com/prl-mushr/hound_core/blob/main/src/hound_hl_control.py#L98C1-L111C37):
    ```python
    self.process_grid_map() ## get the elevation map
    speed_limit = np.sqrt((self.Dynamics_config["D"]*9.8)/self.path_poses[self.current_wp_index, 3]) ## get speed limit based on friction coefficients
    speed_limit = min(speed_limit, self.speed_limit) ## get speed limit based on user input
    lookahead = np.clip(speed_limit, self.lookahead/2, self.lookahead) ## speed dependent lookahead
    self.goal, terminate, self.current_wp_index = self.update_goal(self.goal, np.copy(self.state[:3]), self.path_poses, self.current_wp_index, lookahead, wp_radius=self.wp_radius, looping=self.looping) ## get current goal based on trajectory and lookahead
    self.controller.set_hard_limit(self.hard_limit) ## set hard limits on control for safety
    if(terminate):
        self.goal_init = False
        ctrl = np.zeros(2)
    else:
        # calculate control commands. 
        ctrl = self.controller.update(self.state, self.goal, self.map_elev*self.elevation_multiplier, self.map_norm, self.map_cost, self.map_cent, speed_limit)
    self.state[15:17] = ctrl
    self.send_ctrl(ctrl)
    ```
  
  - The high-level controller also publishes a diagnostics message for debugging purposes on `/high_level_diagnostics`
  
  - It uses the [hound_mppi_real.yaml](https://github.com/prl-mushr/hound_core/blob/main/config/hound_mppi_real.yaml) config file.

4. In general, any config files that end with `_real.yaml` are meant to be used on the real system.

## Adapting the HOUND stack to your platform (separate README for each is a WIP! If you need help immediately, please create a Github issue and I can help you out!):
   If you're interested in adapting the HOUND stack to your platform, here are the things you will likely need to tune (please read the usage section first), assuming you are using the same set of sensors, but are just changing their orientations and size of the platform:
  - `HAL.yaml` -- this defines the sensor configurations
  - `D455_parameters.yaml` -- this defines the raw elevation mapping parameters (this is where you'd have to change the sensor noise, min-max elevation thresholds, and so on).
  - `low_level_config_real.yaml` -- this defines the parameters for low-level control
  - `hound_mppi_real.yaml` -- this defines the high-level control parameters


## Datasets:
Several datasets were collected during the testing of the HOUND hardware.
### Note:
1) Some of the rosbags may not contain "all" the messages. Usually, this will be the case for rosbags that are too small (only a few seconds long). This mostly happens because rosbag record takes a few seconds to start recording all the messages.
2) Some of the rosbags may contain odometry and IMU data at 12.5 Hz instead of 50 Hz. The standard is 50 Hz, but sometimes the ardupilot-mavros node does not increase the update rate to 50 Hz. This is caused by a bug that has been fixed now, but had not been addressed when the bags were collected (it was a rare event).

### Visualizing the data in rviz:
1) Start roscore:
```bash
roscore
```
2) Set use_sim_time parameter:
```bash
rosparam set use_sim_time true
```
3) Start a rosbag with clock:
```bash
roslaunch play /path/to/bag_folder/hound_x.launch --clock
```
4) Start rviz as follows:
```bash
rviz -d /root/catkin_ws/src/hound_core/rviz/mppi_rviz.rviz
```
First few datasets collected using the HOUND hardware: 
1) [new_bags_1](https://drive.google.com/drive/folders/1sS3eayuNPIDPXBG4Ejq77Rzc-OpJaYks?usp=sharing)
2) [new_bags_2](https://drive.google.com/drive/folders/1CcIw9SoD6V5kblfzP96k5vfq1-5Uju_v?usp=sharing)

Datasets collected using the d455 instead of the d435 camera:
1) [new_bags_3](https://drive.google.com/drive/folders/1dut0QQscM1zGSM1hDTDna4EqMtbCVdnB?usp=sharing) -- This dataset is not particularly useful, as we were experimenting with region of interest settings on the camera, and ended up saturating the camera sensor in most of the frames. The dataset is only included in case someone wants to tally the kilometers operated by the system.
2) [new_bags_4](https://drive.google.com/drive/folders/12D8AFurK10BF6UKG0dReFFSA7qQtGr2w?usp=sharing)
3) [new_bags_5](https://drive.google.com/drive/folders/1w7_21CfkHCaU1jUW_0Nw0ZIm3knlXst0?usp=sharing)

Datasets collected for evaluating simulator accuracy:
[new_bags_6](https://drive.google.com/drive/folders/1mX6f5TXzX6WHbrv4zwV3bbimhbbg5moI?usp=sharing)






