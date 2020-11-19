# duckietown-visualization
This is a ROS package for visualization of Duckietown maps and duckiebots.

## Features
- [x] Visualization of maps from duckietown-world
- [x] Realtime visualization of duckiebots
- [x] Visualization of road signs

## Prerequisites
- Desktop-Full installation of ROS
- [duckietown-world](https://github.com/duckietown/duckietown-world)
- [geometry](https://github.com/AndreaCensi/geometry)

## Installing
From the `src` directory of your ROS Workspace, run
```
$ git clone https://github.com/duckietown/duckietown-visualization
```
From your workspace directory, run
```
$ catkin build 
```
Run `catkin_make` instead if you don't use `python-catkin-tools`.

Next, source your workspace using
```
$ source devel/setup.zsh
```
Run `source devel/setup.bash` instead if you use bash.

## Running the map visualization
Run the visualization of the `robotarium1` map, which is currently the default 
by using
```
$ roslaunch duckietown_visualization publish_map.launch
```

You can specify different map names to be loaded according to the maps in 
`duckietown-world`. For example,
```
$ roslaunch duckietown_visualization publish_map.launch map_name:="small_loop"
```
You'll notice that in this case, the RViz visualization has the map loaded at 
the corner. This is because the default configuration for RViz (located in `duckietown_visualization/config/default.rviz`) is configured for the 
`robotarium1` map. You can load your own rviz configuration by running
```
$ roslaunch duckietown_visualization publish_map.launch map_name:="small_loop" rviz_config:="path/to/myconfig.rviz"
```


## How it works

To understand the working of this package, you need a basic understanding of the
[ROS Transform library](http://wiki.ros.org/tf2) and 
[RViz Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker). This package reads
the map having name `map_name` from `duckietown-world` and broadcasts markers
([MESH_RESOURCE](http://wiki.ros.org/rviz/DisplayTypes/Marker#Mesh_Resource_.28MESH_RESOURCE.3D10.29_.5B1.1.2B-.5D)) 
for each element of the map. Each class of elements (tiles, road signs, etc.) is 
published in a different namespace under the transform `/map` to provide the 
feature of turning off a certain category of map elements.

## Using duckietown_visualization with your pipeline
Go through the example code in the package `visualization_example` to understand
how to use `duckietown_visualization`.

- First, create a list of duckiebot names which you want to visualize in a yaml 
file similar to `visualization_example/config/example.yaml`
- For each duckiebot, publish a transform from frame `duckiebot_link` to that 
duckiebot. `duckiebot_link` is a fixed frame which is the origin of your 
measurements for each duckiebot.
- It may so happen that your coordinate system does not match with the 
`duckietown-world` coordinate system, which has the origin on the bottom left. 
You can account for this by publishing a static transform between frame `map` 
and frame `duckiebot_link`. An example is present in 
`visualization_example/launch/example.launch`. Note that if your coordinate 
frame aligns with the `map` frame, you would still need to broadcast a transform
, which in this case would be an identity transform.
