# Myhal Tourguide Project 

### Introduction

The aim of this project is to develop a simulated enviroment for data collection and the testing of a novel SLAM algorithm. The enviroment being simulated is the 5th floor of the [Myhal Building](https://www.engineering.utoronto.ca/myhal-centre-for-engineering-innovation-entrepreneurship/) at the University of Toronto. The robot testing platform is a [Clearpath Jackal](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) robot equiped with a [Velodyne VLP-32](https://velodynelidar.com/products/ultra-puck/) lidar sensor and a [Primesense Carmine 1.09](http://xtionprolive.com/primesense-carmine-1.09) RGBD camera. 

The first goal of the project is to produce pointcloud and pose data from the Jackal as it completes predetermined tours around Myhal. This data is then used to train a novel deep-learning SLAM algorithm to distinguish between objects that are moving (eg. humans), objects that can be moved (eg. tables), and immovable objects (eg. walls). The creation of a diverse enviroments and situations within the 5th floor of Myhal (within a range of desired parameters), and the process of Jackal touring are automated. This allows for increased amounts of data to be collected without manual intervention.

### Description

##### Packages

##### jackal_velodyne

##### custom_plugins

##### kinect_v2

##### social_navigation_layers

##### Scripts 

### Installation and Usage

### Sources 

[Jackal Simulation](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html)


