# DFKI Quadruped
<p align="center">
<img src="header.JPG" alt="both quadrupeds controlled by this software stack on top of VULCANO" width="80%"/>
</p>

This repo contains the *ADAPTED* quadruped controller used at DFKI's underactuated lab.
It mainly contains a *simulation*, dynamic walking controller using *MPC*, *WBC* and different *Gait Sequencers* and hardware *drivers* to be used with different quadrupeds.
- To install the whole software stack please refer to [Installation](#installation)
- To run the simulated experiments from our paper [**Benchmarking Different QP Formulations and Solvers for Dynamic Quadrupedal Walking**](https://arxiv.org/abs/2502.01329) please refer to section [Run the solver comparison experiments](#run-the-solver-comparison-experiments)
- To run the model-adaptation from out paper **Adaptive Model-Based Control of Quadrupeds via Online System
Identification using Kalman Filter** (under review) please refer to section [Run adaptive MPC experiments](#run-adaptive-mpc-experiments)
- To run the controller in simulation or on your own hardware *(EXPERIMENTAL)*, please refer to [Run the software stack](#run-the-software-stack) — start with the [Quick start (simulation)](#quick-start-simulation) section for copy-paste commands.

## Authors
- [Shubham Vyas](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/person/shvy01) (Project Leader)
- [Rohit Kumar](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/person/roku02) (Project Leader)
- [Franek Stark](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/person/frst03) (Software Maintainer)
- [Hannah Isermann](mailto:hannah.isermann@dfki.de)
- [Jakob Middelberg](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/person/jami03)
- [Jonas Haack](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/person/joha08)
- [Mihaela Popescu](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/person/mipo02)
- Lasse Shala

## Citation
If you use this code in your scientific work, please cite our paper:
```

@misc{stark_benchmarking_2025,
title = {Benchmarking {Different} {QP} {Formulations} and {Solvers} for {Dynamic} {Quadrupedal} {Walking}},
url = {http://arxiv.org/abs/2502.01329},
doi = {10.48550/arXiv.2502.01329},
author = {Stark, Franek and Middelberg, Jakob and Mronga, Dennis and Vyas, Shubham and Kirchner, Frank},
month = feb,
year = {2025},
note = {Accepted for the International Conference on Robotics and Automation (ICRA) 2025},
}
```

## Acknowledgements
This work has been performed in the AAPLE project funded by the German Federal Ministry for Economic Affairs and Climate Action (BMWK) (Grant Number: 50WK2275).
And it is additionally supported by the M-RoCK project funded by the German Aerospace Center (DLR) with federal funds (Grant Number: FKZ 01IW21002) from the Federal Ministry of Education and Research (BMBF).
Further, it is additionally supported with project funds from the federal state of Bremen for setting up the Underactuated Robotics Lab (Grant Number: 201-001-10-3/2023-3-2).

## Contributing
Please use the [issue tracker](https://github.com/dfki-ric-underactuated-lab/dfki-quad/issues), to submit bug reports and feature requests.
lease use merge requests as described [here](CONTRIBUTING.md) to add/adapt functionality.


## Requirements / Dependencies
* The requirements are all external and will be **automatically installed** when using the docker image, a full list can be found under [DEPENDENCIES](DEPENDENCIES.md).

---

<p align="center">
<img src="robot_fall.gif" width="360" height="202" />
</p>


## Installation

**1. Build the docker image:**

* If you want to build the image yourself, use the `build_new_image.sh` script.

    ```bash
    ./build_new_image.sh
    ```

    > **Note:** If you already started a container before and saved important data to it, make a backup, since this script will delete all previously created containers with the "dfki_quad" label.

**2. Starting and accessing containers**

* You can start a new container by executing the `run_docker.sh` script.

    ```bash
    ./run_docker.sh
    ```

* If you need more than one terminal, you can use the `new_docker_shell.sh` script.
This will launch a new shell for the already running container.

    ```bash
    ./new_docker_shell.sh
    ```

* Containers are managed via **Docker Compose** (`docker-compose.yml`). The `run_docker.sh` script starts or reattaches the `dfki_quad` container and automatically creates the local `ws/build`, `ws/install`, `ws/log`, and `ws/data` directories. Build artifacts and logs persist on the host between container restarts.
* Architecture-specific settings (e.g. power monitoring, external storage on Jetson) are applied automatically via `docker-compose.aarch64.yml` or `docker-compose.x86_64.yml`.

    > **Note:** If you want to create a new container (e.g. after changing Docker settings or rebuilding the image), use the `reset_docker_container.sh` script. This removes all containers with the "dfki_quad" label. Then run `./run_docker.sh` again to create a fresh container.

* **Gamepad:** USB gamepads are passed into the container via `/dev/input`. Connect the gamepad **before** starting the container, or restart the container after plugging it in. See [Gamepad control](#gamepad-control) for supported devices and button mappings.

**3. Building the software stack**

* Inside the container, build the packages and run a simulation by running the following commands.

    > **Note:** The `--symlink-install` option enables you to change *yaml* and *python* files without rebuilding. If you want to use it, you have to use it consistently, as it cannot overwrite compilations without this option. Use `$ rm -r build/ install/ log/` to remove former compilations if you want to compile with this option the first time.

  a) For the ulab quad:
     ```bash
     colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DROBOT_NAME=ulab
     source install/setup.bash
     source ~/setup_ulab_workspace.bash
     ```
    b) For the unitree quad:
     ```bash
     colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DROBOT_NAME=go2
     source install/setup.bash
     source ~/setup_go2_workspace.bash
     ```
    > **Note:** The unitree software stack requires cyclone dds, which will be setup by the last command. This means that your network interface need to be configured in this script.
If you just want to use the simulation, independent from the build command, please just source `setup_ulab_workspace.bash`.

    **Container shortcuts** (defined in the Docker image):
    | Shortcut | Command |
    |----------|---------|
    | `cbg` | Build for Go2 |
    | `cbr` | Build for ULab |
    | `sg` | Source Go2 real-hardware environment (CycloneDDS) |
    | `sr` | Source simulation environment (FastRTPS) |

## Run the solver comparison experiments
If you want to repoduce the results from __Benchmarking Different QP Formulations and Solvers for Dynamic
Quadrupedal Walking__ follow the following instruction:
- Nake sure that you checkout git tag **ICRA25**

> Note: If you just want to recreate the plots, you can use our [Dataset](https://zenodo.org/records/13767157?token=eyJhbGciOiJIUzUxMiJ9.eyJpZCI6ImM2YmI4M2E3LTFjYTEtNDQxYS1iYzZmLTNjMzg2ZGFmZmYzYSIsImRhdGEiOnt9LCJyYW5kb20iOiJhNzZmMGE3ZmUzZTYxNGUyODg5YjJkMWI0N2RiMDkyZCJ9.6vIqkFOnB-MqHcPBpA3v8jU4oUmoOM4eOaKRF3BtRhVLIcOoaQ3HVdNrFoN2r6kkmCvU09NPV8mPprelmFgK-A) and jump directly to step (5).

**1. Installation**
* If not yet done, build the docker image and install the software stack using the _GO2_ quadruped option:
    ```bash
    https://github.com/dfki-ric-underactuated-lab/dfki-quad
    cd dfki-quad
    ./build_new_image.sh
    ./run_docker.sh
    cbg # This is a shorthand to build the software stack using the -DROBOT_NAME=go2 option
    sr # This is a shorthand to ~/setup_ulab_workspace.bash, which is nececarry since the simulation is used and not a real system
    ```
  > **Note**: If you want to run the simulation and the controller on two different machines, like in the paper, the installation has to be done on both.


**2. _Simulation Machine:_ Start the simulation**
* On the machine that should run the simulation run the container and start the simulation
    ```bash
    sr # This sources the ros environment for the simulation
    export ROS_DOMAIN_ID=123 # Make sure both machines run on the same ROS 2 Domain ID
    ros2 launch simulator simulator.launch.py sim:=go2
    ```

* Visualization will be rendered (if configured) and can be seen in the browser (URL will be shown in the terminal, often [localhost:7000](http://localhost:7000/)).

**3. _Target computer:_ Run the experiments**
* Run the automatic script which runs the set of experiments from the simulation automatically for both prediction horizons:
    ```bash
    # Outside of docker run:
    ./automatic_solver_experiments.sh <pc_name> <test_preset> 123 ICRA25_N20 20 ICRA25_N10 10

    # <pc_name> specfies the name of the target computer which is used to label the experiments (in the paper this was out of [arm_orin, desktop_pc, latte_panda])

    # <test_preset> specifies which test preset should be run for the paper this is both [mpc, wbc]

    # 123 is the ROS_DOMAIN_ID on which the simulation runs

    # ICRA25_N20 is the branch and the corresponding prediction horizon 20 on that branch
    # ICRA25_N10 is the branch and the corresponding prediction horizon 10 on that branch
    ```
    > **Note:** The test script will spin a docker container, build the project and run all experiments (i.e. combinations of MPC solvers, WBC solvers, condensing levels, etc.). For each experiment it first runs the standing and then the trot scenario.
    If a single experiment fails (*the terminal might show errors etc.*), the script will after a while automatically restart the experiment and mark the old one as failed, it will repeat this as often as specified above.
    The full script might take depending on your system at least one hour. Please wait until the script tells you that it is finished.

* The automatic script saves the experiment sets to folders under [ws/src/solver_experiments/](ws/src/solver_experiments/)The folders are named automatically in the following way, such that it can be later imported into the plotting script: `<target_platform>-N<precition_horizon>-<experiment_name_tag>`
  * `<target_platform>` indicates on which target computer the controller was executed, e.g. `arm_orin`.
  * `<prediciton_horizon>` indicates whith which MPC prediction horizon the experiments were executed, e.g. `10`.
  * `<experiment_name_tag>` contains the test preset and a timestamp

* **Optional** If you with to run your own set of experiments you can change the underlying python script which is runned by the [automatic_solver_experiments.sh](automatic_solver_experiments.sh):
    * The experiments are run from the script [ws/src/solver_experiments/solver_experiments/solver_experiments.py](ws/src/solver_experiments/solver_experiments/solver_experiments.py)
    * The experimemts can be configured the following way:
        - [Line 626](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L609) sets which MPC solvers are included in the tests
        - [Line 637](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L637) sets which condensed levels are tested for MPC solvers with a sparse interface
        - [Line 633](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L633) sets which modes are included into the tests for the HPIPM MPC solver
        - [Line 638](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L638) sets which WBC solvers are included in the tests
        - [Line 639](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L639) sets which WBC scenes/formulations are included in the tests
        - [Line 645](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L645) sets how often an experiment is repeated until it is considered as 'failed'
        - [Line 634](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L634) sets which OSQP backends are included in the tests
        - [Line 640](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L640) sets which solver tolerances are included in the tests
        - [Lines 649ff](ws/src/solver_experiments/solver_experiments/solver_experiments.py#L649) define the test presets as used by the automatic experiment script


**5. Generate the plots**
* Assuming all different experiment result folders (named in the scheme defined above) containing the single experiments have been moved to one empty root folder, the plots can be generated.
* The jupyter notebook [ws/src/solver_experiments/evaluation_scripts/comparison_paper.ipynb](ws/src/solver_experiments/evaluation_scripts/comparison_paper.ipynb) defines at the beginning in the variable `result_path` the location of that root folder:
    ```bash
    sr # shorthand to source the types needed to load the ros bags
    jupyter notebook --allow-root src/solver_experiments/evaluaion_scripts/comparison_paper.ipynb
    ```
    > **Note:** It is recommended to run the python scripts and Jupyter notebook from the docker image as all dependencies are installed there.
* After the location has been set, run the script and the plots are being generated. This might take a while.

## Run adaptive MPC experiments
If you want to test the algorithm described in __Adaptive Model-Based Control of Quadrupeds via Online System
Identification using Kalman Filter__(under review), make sure that you checkout git tag **IROS25** and run the software stack as described in [Run the software stack](#run-the-software-stack) either in simulation or on real hardware. Once the controller is running, follow these instructions:

**1. Activate Model Adaptation** In a new terminal: access the docker container
```bash
./new_docker_shell.sh
```
If you are using real hardware, make sure you call ```sg```. Then run:
```bash
ros2 param set /mit_controller_node use_model_adaptation true
```

**2. (Optional:) Vizualize the Model Updates** The model updates are published on /quad_model_update. You can use any plotting tool you like. We recommend plotjuggler which is installed in the docker container:

In a new terminal run:
```bash
xhost +local:docker
```
This enables docker to open external windows. Access the docker container and make sure you call ```sg```, if you use real hardware. Then run plotjuggler:
```bash
ros2 run plotjuggler plotjuggler
```
Start the listener and select /quad_model_update and/or /quad_model_debug.

**Note 1:** Always make sure you set the right ROS_DOMAIN_ID when you start a new terminal.

**Note 2:** The way it is tuned, the adaptation only sends updates to the controller when the estimation covariance is low enough. This is for the GO2 robot the case during stepping motion. Therefore, data will only then appear on /quad_model_update. The thresholds and algorithm properties can be accessed in /ws/src/controllers/config/mit_controller_`<`real/sim`>`_`<`go2/ulab`>`.yaml

## Run the software stack
You can run the software stack using the simulation or using a real hardware (for example the Unitree GO2 in the education version).
Currently simulation supports either the GO2 or the dfki in house quadruped.

Following components have to be launched to run the software stack:
- Hardware driver or Simulation
- Leg Driver
- Dynamic walking controller
- *When on the real system:* State estimation

Please find instructions for all components below.
> **Note:** Almost all commands in this section need to run in separate terminals.

### Quick start (simulation)

Use these commands to get a simulated robot walking. Replace `go2` with `ulab` for the DFKI quadruped, and use `cbr` instead of `cbg` when building for ULab.

**On the host (once):**
```bash
./build_new_image.sh   # first time, or after pulling dev-branch changes
./run_docker.sh
```

**Inside the container — build (once per robot variant):**
```bash
cbg                    # build for Go2  (use cbr for ULab)
sr                     # source sim environment (FastRTPS)
```

**Terminal 1 — simulation:**
```bash
sr
ros2 launch simulator simulator.launch.py sim:=go2
```

**Terminal 2 — leg driver** (open a new host shell with `./new_docker_shell.sh`, then):
```bash
sr
ros2 launch drivers leg_driver_launch.py sim:=go2
```

**Terminal 3 — stand up:**
```bash
sr
ros2 launch controllers quad_stand_up.launch.py sim:=go2
```

**Terminal 4 — controller and gamepad:**
```bash
sr
ros2 launch controllers mit_controller.launch.py sim:=go2
```

Connect a USB gamepad before starting the controller. Use the left stick to walk and the face buttons to switch gaits (see [Gamepad control](#gamepad-control)).

### Gamepad control

The controller launch file starts the standard ROS 2 `joy` node (`ros-humble-joy`) together with a `joy_to_target` node that converts joystick input into velocity and gait commands.

**Setup**

1. Connect a USB gamepad to the host machine.
2. Start (or restart) the Docker container with `./run_docker.sh` so `/dev/input` is available inside the container.
3. Launch the controller as shown above — the gamepad driver starts automatically.
4. Verify the gamepad is detected:
   ```bash
   ros2 topic echo /joy
   ```
   Move a stick; you should see axis values change.

> **Note:** If you upgraded from the `main` branch, rebuild the Docker image (`./build_new_image.sh`) so the container includes `ros-humble-joy` instead of the former `joy_linux` package.

**Supported gamepads**

| Gamepad | Notes |
|---------|-------|
| Logitech F310 (Mode **X**) | Recommended. Flip the switch on the back to **X** mode. |
| Logitech F310 (Mode **D**) | Also supported; fewer axes are exposed. |
| Xbox 360 controller | Supported; button mapping may differ slightly. |
| Unitree Go2 wireless controller | Used automatically on real Go2 hardware via the `/wirelesscontroller` topic. |

**Movement**

| Input | Action |
|-------|--------|
| Left stick ↑/↓ | Forward / backward velocity |
| Left stick ←/→ | Lateral velocity |
| Right stick ←/→ | Yaw rate |
| Right stick ↑/↓ | Body pitch |
| RB + right stick ←/→ | Body roll |
| D-pad ↑/↓ | Raise / lower body height |
| LT / RT (triggers) | Reduce / increase velocity scaling |
| Press left or right stick (L / R) | **Emergency damping** — robot goes limp |

**Gaits and sequencer**

| Input | Action |
|-------|--------|
| A | Stand (all feet in contact) |
| X | Walking trot |
| B | Static walk |
| Y | Switch to **Adaptive** gait sequencer |
| LB + Y | Switch to **Simple** gait sequencer |
| LB + X | Pace |
| LB + B | Bound |
| LB + A | Pronk |
| D-pad ←/→ (or RT + D-pad ↑/↓) | Decrease / increase swing-leg height |
| Start | Reset state-estimation covariances (real hardware) |

On the real Go2, the built-in wireless controller provides the same mappings. When both a USB gamepad and the wireless controller are active, whichever was used most recently takes priority.

### Robot selection
* When running a launch file, the used robot and whether it is on the real system or in simulation has to be specified.
For this, it is required to set either the parameter `sim` or the parameter `real` to `ulab` for the dfki dog or to `go2` for the unitree.
For this, you have to append either `sim:=ulab`, `sim:=go2`, `real:=ulab` or `real:=go2` to the launch commands.
Eg. for the MPC controller:
    ```bash
    ros2 launch controllers mit_controller.launch.py sim:=ulab
    # or
    ros2 launch controllers mit_controller.launch.py real:=go2
    # ...
    ```

### Running the Simulation
* In order to run the simulation, start the simulation node:
    ```bash
    ros2 launch simulator simulator.launch.py sim:=go2   # or sim:=ulab
    ```
* All parameters can be set in the `src/simulator/config/simulator_params_*.yaml` file (there is one configuration file per robot). The parameter `visualisation` specifies if the simulator also shows a Meshcat visualisation.

* Visualization will be rendered (if configured) and can be seen in the browser (URL will be shown in the terminal, often [localhost:7000](http://localhost:7000/))
    #### Simulation clock
    The simulator can run faster or slower than realtime. This can be specified via the parameter `simulator_realtime_rate` in the config file.
A value of 0.0 means that the simulator will run as fast as possible. Note that also for other values (1.0 refers to realtime, 0.5 to half of realtime, 2.0 to double of realtime)
this is only an upper bound, meaning that the simulator might run slower.

* In order to synchronize the rest of the software stack the simulator publishes the simulation time on the ` /clock` topic (following the ROS 2 convention).
**To synchronize other nodes, instead of following the computer time (in ROS also known as wall time), they have to listen to this clock topic.**
This can be archived by setting the `use_sim_time` parameter to `True` when launching the respective node.
**This parameter is automatically set to correctly when using the provided launch files.**

    > **Note on developing nodes that use a timer or follow a clock:** \
    Depending on the `use_sim_time` the call node->get_clock() will return a clock that follows the system's time (wall time)  or the `/clock` topic.
    To create a timer based on this setting use the following call:
    [`rclcpp::create_timer(node, node->get_clock(),...)`](https://docs.ros.org/en/ros2_packages/rolling/api/rclcpp/generated/function_namespacerclcpp_1a480aa4d6e0efd8063c211749706e3019.html#)
    Using the call [`node->create_wall_timer(...)`](https://docs.ros.org/en/ros2_packages/rolling/api/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4I000EN6rclcpp4Node17create_wall_timerEN6rclcpp9WallTimerI9CallbackTE9SharedPtrENSt6chrono8durationI12DurationRepT9DurationTEE9CallbackTN6rclcpp13CallbackGroup9SharedPtrEb) will create a timer that follows the system time, ignoring the `use_sim_time` setting.
    **It is very important to consider for each timer if it should follow the simulation or real-time during simulation! Making the wrong decision may result in undesired behaviour during simulation.**

    #### Reset simulation
* The simulation offers a service `/reset_sim` of type [ResetSimulation.srv](ws/src/interfaces/srv/ResetSimulation.srv)
which will reset the simulation and place the robot to a requested pose. This call might take a few seconds.
The joint positions and PID gains will be set to the original initial value.
It returns on a successful reset.
    ```bash
    ros2 service call /reset_sim interfaces/srv/ResetSimulation "{pose: {position: {z: 0.4}, orientation: {w: 1.0, x: 0, y: 0,  z: 0.0}}, joint_positions: [-0.03287414616578515, 0.7382670992579555, -1.6656333683908857, 0.022376335027853064, 0.7301659339175386, -1.6657410165323536, -0.03343962015375848, 0.7507916433141004, -1.6985474687285194, 0.02383046084651509, 0.7426327340796195, -1.6989789512741746]}"
    ```
    #### Manual step the simulation
* Normally the simulation starts and runs with a target frequency as specified by the parameter `simulator_realtime_rate`.
In order to step the simulation manually the parameter `manually_step_sim` in the simulator's config file has to be set to `true`.
The simulator then provides a service `/step_sim` of type [StepSimulation.srv](ws/src/interfaces/srv/StepSimulation.srv)
which will advance the simulation by the amount of seconds specified in the request.
    ```bash
    ros2 service call /step_sim interfaces/srv/StepSimulation dt:\ 10.0\

    ```

### Running the real Hardware
> **Disclaimer:** This is research code, this code can destroy your hardware or harm yourself! \
> **------------------------ USE THIS SOFTWARE ON YOUR OWN RISK ----------------------------** \
> When working with a real system be careful and mind the following safety measures:
> - Brushless motors can be very powerful, moving with tremendous force and speed. Always limit the range of motion, power, force and speed using configurable parameters, current limited supplies, and mechanical design.
>   - Make sure you have access to emergency stop while doing experiments.

#### Unitree GO2 Edu
1. Connect the target computer (that should run the controller, leg driver, hardware driver, state estimation) via ethernet to the quadruped, and make sure it is configured as described [here](https://github.com/unitreerobotics/unitree_ros2?tab=readme-ov-file#connect-to-unitree-robot).
2. Install build docker, install the software stack using the GO2 option, as described above.
3. On the target computer launch a shell (maybe via ssh) and start the hardware driver:
    ```bash
   ./run_docker # if not already done, or maybe ./new_docker_shell
    sg # This is a shorthand to source the workspace and set the correct ROS DDS, FOR THE GO2 QUAD THIS HAS TO BE DONE IN EVERY SHELL
   ros2 launch drivers unitree_ros2_motor_driver.launch.py
   ```

    > **Note:** The target computer could also be the additional onbaord computer of the GO2 EDU. Details on how to acces the computer can be found [here](https://www.docs.quadruped.de/projects/go2/html/go2_driver.html#go2-network-interface).

#### Ulab quadruped
1. Connect the robot via ethernet to your computer.

2. Connect to robot via ssh:
    ```bash
    ssh ubuntu@10.0.0.20
    ```
   password: `ulab-dfki`

3. Set date on robot:

   from a new terminal
   ```bash
   ssh ubuntu@10.0.0.20 "sudo date -s '$(date +"%Y-%m-%d %H:%M:%S")'"
   ```

4. source repo:
   ```bash
   ws
   ```
   this is a shortcut for `cd dfki-quad-ros2/ws && source .install/setup.bash`

5. rezero legs:
   attach the 3d printed parts to the robot legs to bring them in streched out pose and use
   ```bash
   sh ~/dfki-quad-ros2/ws/src/drivers/scripts/zero_fl_leg.sh
   sh ~/dfki-quad-ros2/ws/src/drivers/scripts/zero_fr_leg.sh
   sh ~/dfki-quad-ros2/ws/src/drivers/scripts/zero_bl_leg.sh
   sh ~/dfki-quad-ros2/ws/src/drivers/scripts/zero_br_leg.sh
   ```
   on the quadruped or
   ```bash
   sh ~/dfki-quad-ros2/ws/src/drivers/scripts/zero_single_leg.sh
   ```
   on the single leg robot.

6. launch motor driver:
   ```bash
   ros2 launch drivers mjbots_ros2_motor_driver.launch.py
   ```
   to change the parameters like the *control frequency* adapt the file `src/drivers/config/mjbots_test_params.yaml`.

7. **stop motor driver** with `ctrl-c`.
   In case the motors don't stop you can use
   ```bash
   . src/drivers/scripts/shutdown.sh
   ```
   and/or
   ```bash
   sudo killall  mjbots_ros2_motor_driver
   ```

    **Troubleshooting**
   - *interfaces* does not compile:

     deactivate pyenv using:
     ```bash
     pyenv deactivate
     ```

### State estimation
* Start the state estimation with:
    ```bash
    # select robot with sim:=ulab, sim:=go2, real:=ulab or real:=go2
    ros2 launch state_estimation state_estimation.launch.py sim:=go2
    ```

### Leg driver
* Start the leg driver with:
    ```bash
    # select robot with sim:=ulab, sim:=go2, real:=ulab or real:=go2
    ros2 launch drivers leg_driver_launch.py sim:=go2
    ```

### Dynamic walking controller
* First make sure the robot is standing, this can be archived by:
    ```
    # select robot with sim:=ulab, sim:=go2, real:=ulab or real:=go2
    ros2 launch controllers quad_stand_up.launch.py sim:=go2
    ```
* Then launch the controller (starts the gamepad driver automatically — see [Gamepad control](#gamepad-control))
    ```
    # select robot with sim:=ulab, sim:=go2, real:=ulab or real:=go2
    ros2 launch controllers mit_controller.launch.py sim:=go2
    ```
* This command will start the safe launch, which waits for a state message (coming from the simulation or state estimation) and makes sure that the state estimation is not drifting
---
<details>
  <summary> <b> Note on Code Formatters </b> </summary>

## Code Formatters

To insure a smooth code development between multiple developers, we urge you to install the following code formatters:

- Clang format and tidy for `c++` (we use `C++17`)
- Black for `python`
- CMake format for `CMakeLists`

We recommend to install them using the following instructions and apply them using vscode's "formatOnSave". This is however not an obligation as every developer has understandably their own preferred IDE and OS. _However, submitting code that is not formatted according to the above mentioned formatters may result in a longer time to merge your code into the master branch._

To install `clang-format` and `clang-tidy` you can simply use an apt install:

```bash
sudo apt install clang-format clang-tidy
```

To install `black` you can use `pip`. However, we highly recommend that you never use `pip` to install packages globally, but rather `pipx`:

```bash
python3 -m pip install --user pipx
python3 -m pipx ensurepath
pipx install black
```

Same for installing `cmake-format`:

```bash
pipx install cmakelang
```

Once all four code formatters have been installed, you need to add the following extensions to your vscode:

- C/C++ (ms-vscode.cpptools)
- Clang-Format (xaver.clang-format)
- cmake-format (cheshirekow.cmake-format)
- markdownlint (davidanson.vscode-markdownlint)
- XML Tools (dotjoshjohnson.xml)

You can then add the following lines to your vscode JSON "workspace" or "user" settings file and save it:

```json
"settings": {
  // C++
  "[cpp]": {
   "editor.defaultFormatter": "xaver.clang-format",
   "editor.formatOnSave": true
  },
  "clang-format.fallbackStyle": "Google",
  "clang-format.style": "{BasedOnStyle: 'Google', BreakBeforeBinaryOperators: NonAssignment, BinPackParameters: 'false', BinPackArguments: 'false', PointerAlignment: 'Left', ColumnLimit: 120}",
  "C_Cpp.intelliSenseEngineFallback": "enabled",
  // Python
  "[python]": {
   "editor.formatOnSave": true
  },
  "python.formatting.provider": "black",
  "python.formatting.blackPath": "$HOME/.local/bin/black",
  "python.formatting.blackArgs": [
   "--line-length",
   "120"
  ],
  // Markdown
  "[markdown]": {
   "editor.formatOnSave": true,
   "editor.formatOnPaste": true
  },
  "markdownlint.config": {
   "MD013": false,
   "MD033": false
  },
  // Xml
  "[xml]": {
   "editor.formatOnSave": true
  },
  // General
  "workbench.editor.highlightModifiedTabs": true,
  "editor.renderWhitespace": "all",
  "editor.renderControlCharacters": true,
  "files.trimTrailingWhitespace": true,
  "editor.formatOnSave": true
 }
```

Make sure the `"python.formatting.blackPath"` points to the correct location for vscode to find your `black` installation.

</details>


