# Myhal Tourguide Project 

## Usage

### Dependencies

ROS-melodic and Gazebo9 are used in this simulation, along with various other ROS packages and external dependencies.
See [this Dockerfile](https://github.com/BenAgro314/ROS-Dockerfiles/blob/master/docker_ros_melodic/Dockerfile) for a list of the required programs to run this simulation.

### Running the Simulation

#### Master Script

To run the simulation, call the `master.sh` script. This script has many options (listed by frequency of use):

-t [arg], the name of the tour being used (default = A\_tour). The tour files can be found in src/myhal\_simulator/tours/.

-f, if set, the simulation will use pointcloud filtering. If there is no online classification, the -g flag must be passed aswell.

-g, if set, the simulation will use ground truth lidar classifications.

-m, if set, the simulation will use [gmapping](http://wiki.ros.org/gmapping) for SLAM, otherwise it will use [amcl](http://wiki.ros.org/amcl) for localization only.

-e, if set, various topics (pose estimate, global plan, local plan, and current target) will be visualized in the simulation.

-v, if set, the GUI for Gazebo and Rviz will be launched.

-l [arg], if this option is given, the simulation will load the provided world file located in /home/$USER/Myhal\_Simulation/simulated\_runs/[arg], otherwise it will generate a new world based on the specified parameters (default = None).

-p [arg], what parameter file is being used (default = defautl\_params).

For example, some common calls are:

+ `./master.sh -t E_tour -emfg`
+ `./master.sh -t J_tour -v -l 2020-08-04-17-04-21`

The first command would launch the simulation with the tour `E_tour`, visualize topics in the simulation, use gmapping and ground truth classifications with pointcloud filtering.
The second command would launch the simulation with the tour `J_tour` along with a GUI, and load the world file from the previous run `2020-08-04-17-04-21`.

#### Parameter Specification

Parameter directories are located in src/myhal\_simulator/params/. The default parameters are in a directory called default\_params. 
The files in this foler can be modified directly, or a copy of this folder can be made.
To use a non-default parameter directory, it's name must be specified using the -p flag when calling `master.sh` (see [above section](#Master-Script)).
The files that have parameters to be modified are (in order of usefulness):

- room\_params\_V2.yaml
- scenario\_params\_V2.yaml
- camera\_params.yaml
- common\_vehicle\_params.yaml 
- plugin\_params.yaml 
- model\_params.yaml 
- animation\_params.yaml 

##### Room Params

Room parameters are specified in room\_params\_V2.yaml.
The name of any room to be included in the simulation must be included in the list `room_names`.
Each room name has a corrisponding entry in the form (note that angled braces are meant to be filed in with a name):

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

Note: the reason for the convoluted yaml specification is because ROS only allows lists and dictionarys of primitve data types to be loaded to the parameter sever.
This is a problem I am actively working on fixing by bypassing the parameter server all together and reading a yaml file directly.

##### Scenario Params:

Scenario parameters are specified in scenario\_params\_V2.yaml.
The name of any scenario to be included in the simulation must be included in the list `scenario_names`.
Each scenario name has a corrisponding entry in the form:

```yaml
<scenario_name>:
  pop_density: <float as a string> # specifies the population density of the scenario in people/m^2
  table_percentage: <float between "0" and "1" as a string> # specifies what percentage of the avaible model positions will be filled in the room
  actor: <actor_type> # see the model params file
  model_list: <model_list> # which models are available to be placed in the room, see the model params file
  table_group_list: <table_list> # which table groups are available to be placed in the room, see the model params file
```

More paramter descriptions are on the way.

## The Navigation Stack 

TODO

## Simulation Details

TODO

## Sources 

[Jackal Simulation](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html)

[Craig W. Reynolds, Steering Behaviors For Autonomous Characters](http://www.red3d.com/cwr/steer/gdc99/)

[Daniel Shiffman, THE NATURE OF CODE](https://natureofcode.com/book/chapter-6-autonomous-agents/)
