# Mapping

Credit to University of California, Berkley for the starter code. Link to the lab instructions https://pages.github.berkeley.edu/EECS-106/fa21-site/assets/labs/Lab_8__Occupancy_Grids.pdf

### Introduction

In the Fall of 2022, I worked with the Worcester Fire Department, Worcester, MA to build a scalable mobile robot solution to assess fire safety of building floors. For this purpose, my team and I deployed standard turtlebot packages such as move base and gmapping to achieve reasonable autonomy and slam. 

We achieved great results in our attempt to build a novel approach for detecting obstacles in the way to fire exits and scoring a safety metric for safety preparedness of buildings. One thing that bothered me was the abstraction in our solution that arrised from deploying standard ROS packages. This repository contains my attempt to build my own package for mapping. 

The occupany_grid_2d.py node under the mapping package implements one of the most important data structures in mobile robotics: <i> occupany grids </i>. It is essentially a 2d matrix (in this case due to 2 dimensional lidar), each cell containing a probability value of the cell being either occupied or free. This matrix keeps a track of each cell in the environment. 
  
### Set-Up 
  
The package named "mapping" is a ROS package which is contains the occupancy_grid_2d.py node for mapping. The mapping class is initialized and its member functions are responsible for mapping. This class has a subscriber and a publisher. The node subscribes from the /scan topic and publishes to the topic "/map/vis". The type of the message published in Marker. 
  
Marker Msg
http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
  
Prior to runnning the occupancy node for mapping the environment, a turtlebot gazebo node must be launched. 
  
The following are the ROS Paramater Server Params that are set through the launch file called "demo.launch".
  
``` 

 <arg name="fixed_frame" default="odom" />
 <arg name="sensor_frame" default="base_footprint" />

 <!-- Topics. -->
 <arg name="sensor_topic" default="/scan" />
 <arg name="vis_topic" default="/vis/map" />

 <!-- Dimensions and bounds. -->
 <arg name="x_num" default="250" />
 <arg name="x_min" default="-10.0" />
 <arg name="x_max" default="10.0" />
 <arg name="y_num" default="250" />
 <arg name="y_min" default="-10.0" />
 <arg name="y_max" default="10.0" />

 <!-- Update parameters. -->
 <arg name="occupied_update" default="0.7" />
 <arg name="occupied_threshold" default="0.97" />
 <arg name="free_update" default="0.3" />
 <arg name="free_threshold" default="0.03" />
  
```

These parameters can be adjusted to build maps with requried resolution and speed given the available compute.

### Update Rule

Updating With Log-odds

In the code for this lab, each voxel in the map actually stores the log-odds of the cell instead of the probability. Conversion from probability to log-odds transforms the range of possible values from [0, 1], which is bounded and centered around 0.5, to the range (−∞, ∞), which is unbounded and centered around 0, making things easier for analysis.

<p align="center"><img align="center" src="https://raw.githubusercontent.com/deveshdatwani/lidar-mapping/main/assets/log-odds.png" height=300, width=300></p>

<i> When a scan ray terminates at a particular cell, that cell’s log-odds ratio is incremented by some small amount — i.e., ` ←− ` + ∆ occ — and then upper bounded by a maximum threshold to ensure numerical stability. Likewise, when the ray passes through a cell (and does not terminate there), that cell’s log-odds ratio is decremented by some other amount — i.e., ` ←− ` + ∆ f ree , where by convention ∆ f ree is negative — and similarly lower bounded by a minimum threshold. In particular, these increments are computed as the log-odds ratios corresponding to the probability that a cell is occupied given that a ray terminates there and the probability that a cell is occupied given that a ray passes through it, respectivel </i> - Lab 8 documentation, UC Berkley

Below is Bayes' rule for updating each cell with its conditional probability.

<p align="center"><img align="center" src="https://raw.githubusercontent.com/deveshdatwani/lidar-mapping/main/assets/bayesian.png" height=300, width=300></p>

### Traversing Algorithm

Once the end point is located using geometric principles, the algorithm "walks" back towards the robot and updates the occupancy of each voxel in its path. This is carried out through the Bresenham's algorithm.

<p align="center"><img align="center" src="https://raw.githubusercontent.com/deveshdatwani/lidar-mapping/main/assets/bresenham.png" height=300, width=300></p>


### Running The Code 

Follow the steps below in order to map the environment in gazebo 

1. Source all necassary setup files in the devel folder
2. Launch a turtlebot in the Gazebo environment <br>
``` roslaunch turtlebot3_gazebo turtlebot3_world.launch ```
3. Run the demo <br>
``` roslaunch mapping demo.launch ```
4. Launch rviz and add visualization through the topic /map/vis <br>
``` roslaunch turtlebot3_gazebo_rviz turtlebot3_gazebo turtlebot3_gazebo_rviz.launch ```

### Example Mapping

Turtlebot World 1

``` 
x_res, y_res = 0.01
x_num, y_num  = 450 

```

<p align="center"><img align="center" src="https://raw.githubusercontent.com/deveshdatwani/lidar-mapping/main/assets/map1.png" width=600></p>

  
  
