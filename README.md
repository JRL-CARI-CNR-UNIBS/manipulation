# manipulation #

[![Build Status](https://travis-ci.com/JRL-CARI-CNR-UNIBS/manipulation.svg?branch=master)](https://travis-ci.com/JRL-CARI-CNR-UNIBS/manipulation)
[![codecov](https://codecov.io/gh/JRL-CARI-CNR-UNIBS/manipulation/branch/master/graph/badge.svg?token=OZF414VCOL)](https://codecov.io/gh/JRL-CARI-CNR-UNIBS/manipulation)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/c1d5c62b533141428918ecd2c0d8628c)](https://www.codacy.com/gh/JRL-CARI-CNR-UNIBS/manipulation/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=JRL-CARI-CNR-UNIBS/manipulation&amp;utm_campaign=Badge_Grade)
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJRL-CARI-CNR-UNIBS%2Fmanipulation.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2FJRL-CARI-CNR-UNIBS%2Fmanipulation?ref=badge_shield)


## Description
The CNR manipulation framework is a collection of ROS package that provides a series functionalities to manipulation skills.

The package is integrated with MoveIt! to automatically compute, simulate, and execute trajectories. Thanks to the integration with the MoveIt! planning pipeline the motion planners can be loaded as plugins and collision free planning is possible through the collision detection features provided by the MoveIt! planning scene, as for the motion planners, also collision detector can be loaded as plugins.

### Base definitions
Before starting with the manipulation framework modules explanation it is necessary to provide the following definitions:

- *Task*: is the final goal to be reached by the robotic system represented by a group of *Skill Actions*. The *Task* decomposition into single *Skill Action* is made by a task planner, or by an action planner if present, or by the user, that assign single *Skill Action* to the manipulation framework.

- *Skill Action*: is a single action that represent an elementary skill such as pick, place, screw, etc...

- *Sub Action*: is the elementary movement executed by the robotic system, a group of *Sub Action* forms a *Skill Action*. For example, the pick *Skill Action* can be decomposed in: move to approach position, move to object position, actuate the grasping system, move to leave position, etc...

The *Task* decomposition previously described is reported in the following picture:

![*Task*, *Skill Action* and *Sub Action* hierarchy.](documentation/Task_SkillAction_SubAction.png)


### Manipulation framework layers

The manipulation framework is made by three main layers: the bottom layer (the green layer) has in charge the geometrical information management and the motion planning. The intermediate layer (the orange layer) manages the trajectories execution, the robot controllers and tools management. Finally, the top layer (the blue layer) deals with the *Skill Actions* decomposition and the *Sub Actions* execution.

The task or the action planners can add and remove dynamically *Skill Actions*, multiple *Skill Actions* can contemporary exists.

The scheduling of the actions is in charge to the action planner or to a task planner, in general, the framework can be used by the user even without the presence of planners on the above levels, by simply exploiting the motion planning and motion control functionalities.

![Manipulation framework layers description.](documentation/manipulation_framework_scheme.png)



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

show how to create server, client and loader for the basic Skill Actions: pick, place, goto.   

## Installation and ROS-Version
This framework was developed using the ROS Noetic distribution.

The software can be installed with the following [rosinstall file](manipulation.rosinstall).


## Work in progress
The manipulation framework package is continuosly evolving. If you find errors or if you have some suggestions or if you want to contribute [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/manipulation/issues).

## Developer Contact
**Authors:**   
- Enrico Villagrossi (enrico.villagrossi@stiima.cnr.it)  
- Manuel Beschi (manuel.beschi@unibs.it)  


_Software License Agreement (BSD License)_    
_Copyright (c) 2021, National Research Council of Italy, Institute of Industrial Technologies and Automation_    
_All rights reserved._


## Acknowledgements
The manipualtion framework is developed by CNR-STIIMA (http://www.stiima.cnr.it/)

***

![EC-H2020](documentation/Sharework.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](documentation/flag_yellow.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.



## License
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJRL-CARI-CNR-UNIBS%2Fmanipulation.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2FJRL-CARI-CNR-UNIBS%2Fmanipulation?ref=badge_large)
