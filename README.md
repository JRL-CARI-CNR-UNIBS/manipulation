# manipulation #

[![Build Status](https://travis-ci.com/JRL-CARI-CNR-UNIBS/manipulation.svg?branch=master)](https://travis-ci.com/JRL-CARI-CNR-UNIBS/manipulation)

## Description
The CNR manipulation framework is a collection of ROS package that provides a series functionalities to manipulation skills.

The package is integrated with MoveIt! to automatically compute, simulate, and execute trajectories. Thanks to the integration with the MoveIt! planning pipeline the motion planners can be loaded as plugins and collision free planning is possible through the collision detection features provided by the MoveIt! planning scene, as for the motion planners, also collision detector can be loaded as plugins.

## Manipualation framework packages
The following packages are available:
1. [manipualtion_msgs](manipulation_msgs/README.md): implement all the messages, services and actions available in the manipulation framework.

2. [manipulation_utils](manipulation_utils/README.md): implement all the basic functionalities provided by the framework. Three basic Skill Actions:
- pick objects
- place objects
- goto location

are currently avaible but additional can be added by the user.

The packages:

3. [inbound_pick](inbound_pick/README.md)

4. [outbound_place](outbound_place/README.md)

5. [go_to_location](go_to_location/README.md)

are packages that show how to create server, client and loader for the basic Skill Actions: pick, place, goto.   

## Installation and ROS-Version 
This framework was developed using the ROS Noetic distribution. 

Clone the repository into your catkin working directory, a .rosinstall file is provided to solve all the dependencies. Compile with ```catkin build```. 
## Developer Contact

**Authors:**   
- Enrico Villagrossi (enrico.villagrossi@stiima.cnr.it)  
- Manuel Beschi (manuel.beschi@unibs.it)  

 
_Software License Agreement (BSD License)_    
_Copyright (c) 2021, National Research Council of Italy, Institute of Industrial Technologies and Automation_    
_All rights reserved._



