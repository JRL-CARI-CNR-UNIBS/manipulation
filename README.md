# manipulation #

[![Build Status](https://travis-ci.com/JRL-CARI-CNR-UNIBS/manipulation.svg?branch=master)](https://travis-ci.com/JRL-CARI-CNR-UNIBS/manipulation)


The CNR manipulation framework is a collection of ROS package that provides a series functionalities to manipulation skills.

The package is integrated with MoveIt! to automatically compute, simulate, and execute trajectories. Thanks to the integration with the MoveIt! planning pipeline the motion planners can be loaded as plugins and collision free planning is possible through the collision detection features provided by the MoveIt! planning scene, as for the motion planners, also collision detector can be loaded as plugins.

The following packages are available:
1. ([manipualtion_msgs](manipulation_msgs/README.md))
2. ([manipulation_utils](manipulation_utils/README.md))

the basic Skill Actions:
- pick objects
- place objects
- goto location

are currently avaible but additional can be added by the user.

The packages:
3. ([inbound_pick](inbound_pick/README.md))
4. ([outbound_place](outbound_place/README.md))
5. ([go_to_location](go_to_location/README.md))

are packages that show how to create server, client and loader for the basic Skill Actions: pick, place, goto.   




