# Myhal Tour Guide Project 

## Table of Contents

  * [Usage](#usage)
     * [Dependencies](#dependencies)
     * [Installation and Compilation](#installation-and-compilation)
     * [Running the Simulation](#running-the-simulation)
     * [World Parameter Specification](#world-parameter-specification)
     * [Robot Parameter Specification](#robot-parameter-specification)
     * [Creating New Tours](#creating-new-tours)
     * [The Data](#the-data)
  * [The Navigation Stack](#the-navigation-stack)
     * [Point-cloud conversion and filtering](#point-cloud-conversion-and-filtering)
     * [Localization](#localization)
     * [Cost-maps](#cost-maps)
     * [Planning](#planning)
  * [Simulation Details](#simulation-details)
     * [World Generation](#world-generation)
     * [World Control](#world-control)
     * [Ground Truth Classifications](#ground-truth-classifications)
  * [The Dashboard](#the-dashboard)

## Usage

### Dependencies

ROS-melodic and Gazebo9 are used in this simulation, along with various other ROS packages and external dependencies.
A GPU is required to run the Velodyne VLP-32 LiDAR simulation.
See [this Dockerfile](https://github.com/BenAgro314/ROS-Dockerfiles/blob/master/docker_ros_melodic/Dockerfile) for a list of the required programs and environment variables to run the simulation.

### Installation and Compilation

This repository should be located in `/home/$USER/catkin_ws`, meaning the root JackalTourGuide folder should be renamed to `catkin_ws`:

``` bash
cd ~
git clone https://github.com/BenAgro314/JackalTourGuide.git catkin_ws 
```

For data storage, there must be a directory `~/Myhal_Simulation/simulated_runs/`:

``` bash
cd ~
mkdir -p Myhal_Simulation/simulated_runs/
```

To compile, run:

``` bash
cd ~/catkin_ws
catkin build
```

As aforementioned, various environment variables must be set prior to running the simulation.
Add these lines to your `~/.bashrc`:

``` bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib
export GAZEBO_MODEL_PATH=~/catkin_ws/src/myhal_simulator/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/myhal_simulator/models:${GAZEBO_RESOURCE_PATH}
```

Finally, `source ~/.bashrc` to make these changes in your current shell.

### Running the Simulation

#### Master Script

To run the simulation, call the `master.sh` script. This script has many options (listed by frequency of use):

-t [arg], the name of the tour being used (default = `A_tour`). The tour files can be found in `src/myhal_simulator/tours/`.

-f, if set, the simulation will use point-cloud filtering. If there is no online classification, the -g flag must be passed as well.

-g, if set, the simulation will use ground truth LiDAR classifications.

-m, if set, the simulation will use [Gmapping](http://wiki.ros.org/gmapping) for SLAM, otherwise it will use [AMCL](http://wiki.ros.org/amcl) for localization only.

-e, if set, various topics (pose estimate, global plan, local plan, and current target) will be visualized in the simulation.

-v, if set, the GUI for Gazebo and Rviz will be launched.

-l [arg], if this option is given, the simulation will load the provided world file located in `/home/$USER/Myhal_Simulation/simulated_runs/[arg]`, otherwise it will generate a new world based on the specified parameters (default = None).

-p [arg], what parameter file is being used (default = `default_params`).

For example, some common calls are:

+ `./master.sh -t E_tour -emfg`
+ `./master.sh -t J_tour -v -l 2020-08-04-17-04-21`

The first command would launch the simulation with the tour `E_tour`, visualize topics in the simulation, use Gmapping and ground truth classifications with point-cloud filtering.
The second command would launch the simulation with the tour `J_tour` along with a GUI, and load the world file from the previous run `2020-08-04-17-04-21`.

#### Using Online Classifications

Online classifications can only be used through docker. To obtain the necessary docker images:

``` bash
cd ~
git clone https://github.com/BenAgro314/ROS-Dockerfiles.git
cd ~/ROS-Dockerfiles/docker_ros_melodic/ && ./docker_build.sh
cd ~/ROS-Dockerfiles/docker_ros_noetic/ && ./docker_build.sh
```

By calling the script `~/ROS-Dockerfiles/run_files/classification_test.sh`, two containers will be launched, one running the Myhal simulation in ROS Melodic, the other running the online classifications in ROS Noetic.

Note: the script `classification_test.sh` will have to be modified to suit your needs.
Currently it mounts directories based on the usernames `bag` and `hth`.

To run a specific test in the docker:

``` bash
./classification_test.sh -c "./master.sh <insert_flags>"
```

The containers are by default detached upon creation.

### World Parameter Specification

Parameter directories are located in `src/myhal_simulator/params/`. The default parameters are in a directory called `default_params`. 
The files in this folder can be modified directly, or a copy of this folder can be made.
To use a non-default parameter directory, it's name must be specified using the -p flag when calling `master.sh` (see [above section](#Master-Script)).
The files that have parameters to be modified are (in order of usefulness):

- `room_params_V2.yaml`
- `scenario_params_V2.yaml`
- `camera_params.yaml`
- `common_vehicle_params.yaml` 
- `plugin_params.yaml` 
- `model_params.yaml` 
- `animation_params.yaml` 

#### Room Parameters

Room parameters are specified in `room_params_V2.yaml`.
The name of any room to be included in the simulation must be included in the list `room_names`.
Each room name has a corresponding entry in the form (note that angled braces are meant to be filed in with a name):

```yaml
<room_name>:
    geometry: <geo_name> # specifies the name of the geometry list
    position: <pos_name> # specifies the name of the position list 
    enclosed: <"0" or "1"> # whether or not people can enter and exit the room 
    scenario: <scenario_name> # the scenario in the room, see scenario params

<geo_name>: # this specifies the rectanglar bounds of the room
    x_min: <float>
    y_min: <float>
    x_max: <float>
    y_max: <float>

<pos_name>: # this specifies the suitable posiions to place tables and chairs as (x,y) pairs. This can be as long as desired.
    - <float x1> 
    - <float y1>
    - <float x2> 
    - <float y2>
```

Note: the reason for the convoluted yaml specification is because ROS only allows lists and dictionaries of primitive data types to be loaded to the parameter sever.
This is a problem I am actively working on fixing by bypassing the parameter server all together and reading a yaml file directly.

#### Scenario Parameters

Scenario parameters are specified in `scenario_params_V2.yaml`.
The name of any scenario to be included in the simulation must be included in the list `scenario_names`.
Each scenario name has a corresponding entry in the form:

```yaml
<scenario_name>:
  pop_density: <float as a string> # specifies the population density of the scenario in people/m^2
  table_percentage: <float between "0" and "1" as a string> # specifies what percentage of the avaible model positions will be filled in the room
  actor: <actor_type> # see the model params file
  table_group_list: <table_list> # which table groups are available to be placed in the room, see the the end of the scenario_params_V2.yaml file

<table_list>: # specifies which tabel groups (table model and chair model) can be used in the simulation. See model_params.yaml for table groups
    - <table_group_1>
    - <table_group_2>
```

#### Camera Parameters

Camera parameters are specified in `camera_params.yaml`.
There are two ways of placing cameras in the simulation (which can be used together).

The first method requires that every camera placed in the world is given a name and listed in `camera_names`. 
Every named camera has a corresponding entry in the form:

``` yaml
<camera_name>:
    - 0  # Mode: (0 - sentry), (1 - hoverer), (2 - stalker)
    - -8 # x 
    - 3 # y 
    - 10 # z 
    - -1 # orbiting period  (if < 0, no rotation)
    - -1 # following distance  (if < 0, NA)
```

The first number in the list sets the cameras mode, See [Cameras](#Cameras) below.
The next three numbers set the x,y and z position of the camera.
If the camera is a Sentry (mode 0), then this is a pose relative to the world.
If the camera is a Hoverer (mode 1), then this is a fixed pose relative to the robot.
If the camera is a Stalker (mode 2), then then z param is used to set the height of the camera.
The next parameter is the orbiting period and is used for the Hoverer. If this param is greater than 0, then the camera will complete one orbit around the robot every T seconds, where T is the period.
The last parameter is the following distance and is used for the Stalker. It specifies the distance behind the Jackal (on the Jackal's path), that the camera will follow.

#### Model Parameters

Model parameters (including actors) are specified in `model_params.yaml`.
Any model to be included must have a name specified in `model_names`.
Each named model has a corresponding entry in the form:

``` yaml
<table_name>:
  filename: model://<file_name>
  width: <float as string> # the width of the model (default x direction) in m 
  length: <float as string> # the length of the model (default y direction) in m
  height: <float as string> # optional parameter that can be used to set the height of a chair model in m
```

Similarly, any actor to be included must have a name specified in `actor_names`.
Each named actor has a corresponding entry in the form:

``` yaml
<actor_name>: 
  filename: model://<file_name>
  plugin: <plugin_name> # see plugin_params.yaml
  obstacle_margin: <float as string> # the buffer distance given to each actor when being placed in the world (in m), to avoid being placed in another object.
```

Note that the files listed in `filename` must be included in a file listed in the environment variables `$GAZEBO_MODEL_PATH` and `$GAZEBO_MODEL_PATH`.

Finally, table and chair models can be grouped in `table_groups`. Include the name of the table group in `table_group_names`.
Each named group has a corresponding entry in the form:

``` yaml
<table_group_name>:
  table: <table_model_name>
  chair: <chair_model_name>
```

These are referenced in `scenario_params_V2.yaml` discussed above.

The other parameter files are rarely modified. They can be customized in a similar format to the parameters discussed above:

- `plugin_params.yaml`: used for specifying the plugins that are added to the actors. Each plugin includes a `vehicle_type` (see [Actors](#Actors) below) and any corresponding fields for that actor behaviour. For example if `vehicle_type: stander`, then the plugin should include the characteristics for a stander: `standing_duration: '5'`. See the function `CreateVehicle()` in `src/myhal_simulator/src/puppeteer.cpp` for more information. 
- `animation_params.yaml`: contains the names of the animations given to every actor and the file of that animation.
- `common_vehicle_params.yaml`: contain various parameters that are common to each vehicle such as their mass and maximum speed. 
- `myhal_walls.ply`: this specifies the bounding boxes for the walls of the Myhal model. This should not be modified unless the model used changes.

More parameter descriptions are on the way.

#### Creating New Tours

The files used to specify tours are stored in `src/myhal_simulator/tours/`.
To create a new tour, make a copy of the directory `template`:

``` bash
cd ~/catkin_ws/src/myhal_simulator/tours/
cp -r template/ <tour_name>
```

There are two files in the directory for each tour:

- map.txt, which is a text representation of the blueprint of the 5th floor of Myhal
- config.yaml, which specifies the map resolution (how many meters does each character in the map represent), and the rectangular bounds of the map in the form `[min_x, min_y, max_x, max_y]` 

To specify the targets along the tour, open map.txt in a text editor that can zoom out (I recommend [Geany](https://www.geany.org/)), or at least retains the formatting of the file.
Upon zooming out as far as possible, the map will look something like this:

![maptxt](imgs/map.png)

Each black dot is a '#' character. To add a target on the tour, delete the space character where you want to add the target, and replace it with an alphabetical character (from A to Z and a to z).
The order of the targets on the tour is determined by the ASCII value of that character, so it is alphabetical with all uppercase letters preceding all lowercase letters.

### Robot Parameter Specification

Various parameters of the mobile base Jackal robot can be modified through files in `src/jackal_velodyne/params/` and `src/jackal_velodyne/launch/`.

`src/jackal_velodyne/params/` contains:

- `base_local_planner_params.yaml`, parameters for the local planning, details [here](http://wiki.ros.org/base_local_planner).
- `move_base_params.yaml`, parameters for the ROS package `move_base`, details [here](http://wiki.ros.org/move_base#Parameters).
- All other files (including those in `map_nav_params/`) pertain to the local and global cost-map parameters, details [here](http://wiki.ros.org/costmap_2d). 

`src/jackal_velodyne/launch/` contains various launch files. The [Master Script](#Master-Script) only calls `p1.launch`, while `p2.launch` is later called by the world plugin `src/myhal_simulator/src/puppeteer.cpp` once the simulation is loaded.
`p1.launch` is responsible for launching the Gazebo simulation, while `p2.launch` spawns the Jackal in the simulation with the correct configurations.

Parameters for the conversion and filtering of the 3D LiDAR data to 2D laser-scans sorted by classes can be found in `src/jackal_velodyne/launch/include/pointcloud_filter2.launch`.
Here you will find many if statements which choose the correct configuration depending on the localization method used (Gmapping or AMCL), the classification method used (ground truth or online classifications), and whether or not filtering is being used. 
For each configuration, multiple laser-scans need to be created. In general this includes one for local planning, one for global planning and one for navigation, but this is not true in all cases.
See details on [The Navigation Stack](#The-Navigation-Stack) below for more details.
For each laser-scan that is created, a `pcl_filter` nodelet is launched with specific parameters. The parameter that dictate which classes are to be included in a specific nodelet's laser-scan is:

``` yaml
catagories:
  - <int class>
  - <int class>
  - <int class>
```

Two other launch files contain useful parameters to modify:

- `src/jackal_velodyne/launch/include/amcl.launch`, which contain various parameters for AMCL, details [here](http://wiki.ros.org/amcl) 
- `src/jackal_velodyne/launch/include/gmapping.launch`, which contain various parameters for Gmapping, details [here](http://wiki.ros.org/gmapping) 

### The Data

Whenever you run the simulation, a date stamped folder will be created in this directory eg. `~/Myhal_Simulation/simulated_runs/2020-08-06-22-45-03`.
This folder will contains log files and post-processed data:

- `raw_data.bag`, a bag file of data from select topics during the run (see line ~119 in ./master.sh).
- `gt_pose.ply`, a binary representation of the ground truth pose of the robot during it's tour.
- `sim_frames/`, a directory containing binary time-stamped point-cloud frames

Also contained in this directory is a directory called is a logs-<date> (e.g. logs-2020-08-06-22-45-03) which holds:

- `processed_data.pickle`, serialized data used by the [Dashboard](#Dashboard) for assessing navigation performance and creating plots.
- pcl.txt and params.yaml are log files storing all the parameters used during the trials. 
- log.txt stores the command used to launch the simulation and whether or not the robot made it to each waypoint on it's tour. 
- meta.json stores meta data (see `src/dashboard/src/meta_data.py`) about the run which is used by the [Dashboard](#Dashboard) for data management and creating plot series.
- `myhal_sim.world`, a copy of the world file used during this run. 
- videos/, a directory that holds the videos produced during the run.

Note that if the run is cut short with SIGTERM (including stopping a docker container), or by running either of the files `scripts/shutdown.sh` or `scripts/clear.sh`, all data files will be deleted.
To stop a run while saving the data files, run `./scripts/save_shutdown.sh`.

## The Navigation Stack 

The two diagrams below depict two configurations of the navigation stack, the first using AMCL and the second using Gmapping.
If ground truth classifications are used, then the points go the ground truth classification node instead of the online classifier, following the dotted lines.
Various nodes of the navigation stack are discussed below.

![AMCLstack](imgs/AMCL.png)

![Gmappingstack](imgs/GMAPPING.png)

### Point-cloud conversion and filtering 

Currently, the main localization nodes used (see [below](#Localization)) subscribe to 2D laser-scan messages, not 3D point-clouds.
This means that the 3D point-cloud data must be converted to a 2D laser-scan. This is done via `pcl_filter_nodlet.cpp` located in the `jackal_velodyne` package.
Moreover, it is in this conversion step that the classified point-cloud is segregated into different laser-scan messages depending on which parts of the navigation stack we want the various classes to go.
For example, the global planning node will be given a laser-scan message made from the points classified as non-moving, whereas the local planning node message will include all classes of points. 
See the figures above for more detail as to where the classes are sent depending on the localization node used.

### Localization

There are currently two available nodes for robot localization: [AMCL](http://wiki.ros.org/amcl) and [Gmapping](http://wiki.ros.org/gmapping). Both nodes subscribe to a 2D laser-scan and publish a transformation from the odom frame to the map frame. 

#### AMCL

The AMCL node is used purely for localization. It does not create a live map of the environment itself, and thus must be supplied a pre-made occupancy grid map via the map server of the Myhal simulation walls.
When estimating robot pose, AMCL will compare the laser-scans against the map. Thus, to increase localization accuracy when using AMCL, the node will only receive laser-scans constructed from the LiDAR points in the wall class.

#### Gmapping

The Gmapping node provides a SLAM algorithm, not only localizing the robot but providing a building an occupancy grid map at run-time. This means that no pre-made map is needed when using Gmapping.
The map that Gmapping produces is fed directly to the global planner (see below), and thus it is given laser-scans constructed from all static points (walls, tables, chairs etc), so they can be avoided.

### Cost-maps

Both global and local cost-maps are produced using the ROS package [costmap\_2d](http://wiki.ros.org/costmap_2d).
If Gmapping is used, it produces a global occupancy grid which is given to costmap\_2d to create the global cost-map.
If AMCL is used, the costmap\_2d directly subscribes to a laser-scan of consisting of static points.
Regardless of the localization node, the local cost-map is built off of a laser-scan consisting of all points (for local collision avoidance). 

The parameter files for the cost-maps can be found in `src/jackal_velodyne/params/` 

### Planning

#### Global Planner:

The global planner is sent tour goals via the node `navigation_goals_V2.cpp`. Using the global cost-map, it plans a route for the Jackal to follow.
The ROS package used to generate the plan is [navfn](http://wiki.ros.org/navfn) which uses Dijkstra's algorithm.

#### Local Planner:

The local planner's goal is the position furthest along the global path that is also within the bounds of the local cost-map.
The local planner has the responsibility of controlling the mobile base, providing the connection between the global plan and the robot.
This local planner uses the ROS package [base\_local\_planner](http://wiki.ros.org/base_local_planner) which in turn uses [Dynamic Window Approach](https://ieeexplore.ieee.org/document/580977?arnumber=580977) algorithm to create a kinematic local trajectory for the Jackal.

The parameter files for the planners can be found in `src/jackal_velodyne/params/`.

## Simulation Details

### World Generation

If the simulation is launched without specifying a world file to use (with the -l tag), then a new world is created.
To create a world based on the supplied parameters, the file `src/myhal_simulator/worlds/myhal_sim.world` is written by the program `world_factory.cpp` located in the `myhal_simulator` package.
This file is based off of a template file called `myhal_template.txt` located in the same directory, which includes some of the basic [SDF](http://sdformat.org/) for a Gazebo world, as well as various unchanging aspects of the simulation such as the model of the 5th floor of Myhal.
This program reads the ROS parameter server for the various room, scenario, camera, plugin, and animation parameters and uses this information to write the SDF for the corresponding models into `myhal_sim.world`.
All the required actors (people), tables, chairs, and cameras are added by this program.

### World Control

The dynamic elements of the world are controlled by a Gazebo world plugin called Puppeteer, located in `src/myhal_simulator/puppeteer.cpp`.
This plugin is responsible for controlling the motion of the actors and cameras, as well as adding and dynamically modifying the models that allow for topics to be visualized in the Gazebo simulation.

#### Actors

Actors are the people in the simulation. They can have a variety of behaviors, most of which are based off of [Craig W. Reynolds, Steering Behaviors For Autonomous Characters](http://www.red3d.com/cwr/steer/gdc99/) 
The various types of actors are defined in `src/myhal_simulator/src/vehicles.cpp`. All actors follow some common rules and parameters such as collision avoidance with objects, one another, and the robot, as well as a maximum speed and acceleration. Currently available actor types include:

- Wanderers, following the [Wander](http://www.red3d.com/cwr/steer/gdc99/) random steering behaviour.
- Random Walkers, which walk to a random target found by ray-casting  
- Boids, following a [flocking behaviour](http://www.red3d.com/cwr/steer/gdc99/).
- Standers, who follow (randomized) periodic cycle of standing and then wandering.
- Sitters, who sit in chairs (currently they do not start walking).
- Followers, who will follow the Jackal. [Followers](http://www.red3d.com/cwr/steer/gdc99/) can either blocking (intentionally try and get in the way of the robot) or non-blocking (targeting some position behind the robot and attempting to avoid obstructing it).
- PathFollowers, who will follow a provided gradient map, allowing them to exhibit intelligent path finding.

#### Cameras

While the cameras are added to the world on creation of the `myhal_sim.world` file, they are controlled by two plugins, the Puppeteer world plugin and the CameraController sensor plugin located in `src/jackal_velodyne/src/camera_controller.cpp`. 

The CameraController plugin is responsible for saving the image frames received by the camera sensor to .jpg files. These .jpg files are later converted to a .mp4 as part of the data processing in the `src/dashboard/src/data_processing.py`.
This plugin also dynamically throttles the simulation step size, and pauses the simulation while saving the .jpg file, in an attempt to reach it's target frame rate (usually 24 fps).

The Puppeteer plugin is responsible for moving the camera. There are a three types of cameras defined in `src/myhal_simulator/src/vehicles.cpp`, each with different movement rules:

- Sentry: a stationary camera that points towards a desired target (in this case the robot) .
- Hoverer: a moving camera that hovers at a specified relative position with respect to its target. It will point towards the target and can optionally orbit around the target.  
- Stalker: a moving camera that follows it's targets path a specified distance behind, while always pointing at it's target.

#### Topic Visualization

By passing the -e flag when running [master.sh](#Master-Script), the simulation will visualize four ROS topics in Gazebo:

- The global plan, shown as a green line of dots.
- The local plan, shown as a blue line of dots.
- The current target on the tour, shown as a red X.
- The estimated pose from the localization node, shown as a teal rectangle representing the footprint of the robot. The rectangle has a purple strip to show where the front of the footprint is.

The visualization of these topics is managed by the Puppeteer world plugin.

### Ground Truth Classifications 

By passing the -g flag when running [master.sh](#Master-Script), the point-clouds produced by the simulation of the VLP-32 LiDAR sensor will have accompanying ground truth classifications. 
This is achieved with a Gazebo sensor plugin, located in `src/myhal_simulator/src/custom_velodyne.cpp`. 
This sensor plugin is based on [the official velodyne\_simulator package](https://bitbucket.org/DataspeedInc/velodyne_simulator/src) with a few modifications:

- The simulator preforms a collision check between each point and the objects in the world to determine it's class.
- During this computation, the Gazebo world is paused.

This means that despite the extra time taken for ground truth classifications, there is 0 latency from the simulation perspective, and no LiDAR frames are dropped.

## The Dashboard 

The Dashboard is a python module that allows for organization and analysis of data gathered from the simulation.
First, ensure the module can be imported by adding it to the `PYTHONPATH` environment variable:

``` bash
export PYTHONPATH=${PYTHONPATH}:~/catkin_ws/src/
```

Or for solution that will persist between sessions:

``` bash
echo 'export PYTHONPATH=${PYTHONPATH}:~/catkin_ws/src/' >> ~/.bashrc
source ~/.bashrc
```

To use the dashboard, start up a python interpretor. The dashboard has many functions for organizing run data:

``` python
>>> from dashboard import *
>>> d = Dashboard() 
>>> d.list_runs() #prints a table with information on the most recent 10 runs 
>>> n = 20
>>> d.list_runs(n) # prints a table with information on the most recent n runs
>>> d.run_info('2020-07-17-12-47-30') # prints information on the run with the name '2020-07-17-12-47-30'
>>> d.run_info(1) # prints information on the run with the index 1
>>> d.list_dirs() # lists all of the folders in the directory ~/Myhal_Simulation/simulated_runs/, even if they are not valid runs
>>> d.remove_dir('directory_name') # removed a directory with name 'directory_name' from ~/Myhal_Simulation/simulated_runs/
>>> d.remove_dir(clear_old = True) # will remove all directories from the folder that are not valid runs
```

In order to visualize data, first you must specify what data you want to see. This is done by adding `Series()` objects to the Dashboard.
Series represent a set of runs adhering to various characteristics. Currently available characteristics include:

- Time ranges: `earliest_date`, `latest_date`, `date`, type: int.
- Index ranges:`min_ind`, `max_ind`, `ind`, type: int.
- Filtering status: `filtering_status`, type: bool.
- Classification method: `class_method`, type: string.
- Tour name: `tour_names`, type: string.
- Success status: `success_status`, type: bool.
- Localization method: `localization_technique`, type: string.
- Scenarios present during the test: `scenarios`, type: list of strings. 
- Whether or not the run was a localization test or not: `localization_test`, type: bool.

The series must be given a name. Series colors will be chosen randomly if not specified:

``` python
# creates a series named 'Online Classifications' of all runs with tour 'E_tour' which used 'online_predictions'
>>> D.add_series('Online Classification', tour_names = 'E_tour', class_method = 'online_predictions') 
# this series we give a list of matplotlib colors. The order of the list is the priority of color use (some plot types use two colors for one series).
>>> D.add_series('No Classification', colors = ['r','b'], tour_names = 'E_tour', class_method = 'none') 
```

Then you can add various plot types to the Dashboard. Currently available plot types include:

- `TEBoxPlot`: translation error box plot
- `YEBoxPlot`: yaw error box plot
- `TranslationError`: translation error vs distance travelled (line graph)
- `YawError`: yaw error vs distance travelled (line graph)
- `SuccessRate`: bar graph of the success rate of each series. 
- `PathDifference`: Box plot of the difference in length between the optimal path and the path the robot took (for each series).
- `TrajectoryPlot`: a birds eye view of the robot's trajectory (usually useful if only plotting one or two runs)

``` python
>>> d.add_plot(TEBoxPlot()) 
>>> d.add_plot(YEBoxPlot()) 
>>> d.show() # this will show the plots 
>>> d.show(font_size = 30) # change the font size 
```

To create presentable plots, often the best was is to save them to a PDF and then edit the formatting manually later:

``` python
>>> d.show(path = 'plot.pdf') # this will show the plot and save it to plot.pdf in your present working directory 
```

Some other useful functions include:

``` python
>>> d.plot_info(TEBoxPlot) #  text information on a specific plot:
>>> d.plot_run(3, TEBoxPlot()) # plot a specific run by it's index or name 
>>> d.remove_plot(TEBoxPlot) # remove plots and series by type or name respectivly
>>> d.clear_series() # clear all series and plots
>>> d.watch('2020-07-17-12-47-30') # watch the gazebo videos for a specific run (by name or index)
>>> d.rviz_run(0) # play the bag file for a run and view it in rviz (by name or index)
>>> d.remove_series_dirs('series_name') # this will delete the directories of all runs in the series 
```
