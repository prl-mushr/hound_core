


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
