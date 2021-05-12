
# manipulation_utils #

The manipulation_utils package contains all the basic modules to build a Skill Action.

## Description

### Base definitions
Before starting with the manipulation framework modules explanation it is necessary to provide the following definitions:

- *Task*: is the final goal to be reached by the robotic system represented by a group of *Skill Actions*. The *Task* decomposition into single *Skill Action* is made by a task planner, or by an action planner if present, or by the user, that assign single *Skill Action* to the manipulation framework.

- *Skill Action*: is a single action that represent an elementary skill such as pick, place, screw, etc...

- *Sub Action*: is the elementary movement executed by the robotic system, a group of *Sub Action* forms a *Skill Action*. For example, the pick *Skill Action* can be decomposed in: move to approach position, move to object position, actuate the grasping system, move to leave position, etc...

The *Task* decomposition previously described is reported in the following picture:

![*Task*, *Skill Action* and *Sub Action* hierarchy.](../documentation/Task_SkillAction_SubAction.png)


### Manipulation framework layers

The manipulation framework is made by three main layers: the bottom layer (the green layer) has in charge the geometrical information management and the motion planning. The intermediate layer (the orange layer) manages the trajectories execution, the robot controllers and tools management. Finally, the top layer (the blue layer) deals with the *Skill Actions* decomposition and the *Sub Actions* execution.

The task or the action planners can add and remove dynamically *Skill Actions*, multiple *Skill Actions* can contemporary exists.

The scheduling of the actions is in charge to the action planner or to a task planner, in general, the framework can be used by the user even without the presence of planners on the above levels, by simply exploiting the motion planning and motion control functionalities.

![Manipulation framework layers description.](../documentation/manipulation_framework_scheme.png)


### Location Manager module

All the object that need to be manipulated can be described by the data structure *Location* for a generic manipulation pose (position + orientation) (see [Location](../manipulation_msgs/msg)). The *Location Manager* is the module that has in charge the management of multiple *Locations*.

The *Location Manager* embeds a [*MoveIt!*](https://moveit.ros.org/) planning pipeline that enables the use the of multiple motion planners, a *MoveIt!* planning scene and a kinematics module able to compute the inverse kinematics of the overall robotic system for a given *Location*. Every time a new *Location* is added the inverse kinematics for a given move group (robotic arm + grasping system) is computed (with [*RosDyn*](https://github.com/CNR-STIIMA-IRAS/rosdyn) library instead of *MoveIt!* IK functions) and stored. The planning scene is evaluated only one time before planning a new trajectory, the online trajectory replanning is not supported, when a new trajectory planning is required the planning scene is updated and used to generate a collision-free trajectory avoiding collisions between the robotic system and all the entities in the planning environment. The motion planners can be loaded as ROS plugins, as the collision detectors.

A *Location Manager* can handles multiple move groups. The planning scene is unique and shared between multiple *Location Manager*, objects can be dynamically added or removed depending on the real scene evolution, a perception system can be used to this scope.

![*LocationManager* module description.](../documentation/LocationManager_scheme.png)

### Skill Base module

The *Skill Base* module allows to dynamically load/unload the robot controllers (through the [cnr_ros_control](https://github.com/CNR-STIIMA-IRAS/cnr_ros_control) package) depending on the required robot behavior during the execution of a specific *Sub Action*. The module allows to start and monitor the execution of the trajectories planned by the *Location Manager* for a specific move group and enables the control of the tool required by the *Skill Actions*.

![*SkillBase* module description.](../documentation/SkillBase_scheme.png)

### Skill Action module and manipulation framework pipeline

The *Skill Action* ground on the top of the modules *Skill Base* and *Location Manager* defining the following pipeline:

![Manipulation framework pipeline.](../documentation/SkillAction_scheme.png)

The *Skill Action* need to be defined by the user, as for the services to interact with the module. Once a new action is taken in charge, the *Skill Action* module supervises the action execution, checks if all the *Sub Actions* are properly completed and partially manages unexpected behaviors, if severe errors occurs an error message is returned to the action planner and the action execution is interrupted.


## Services

### Location Manager module

The *LocationManager* module provide the following actions/services servers:

| Services | Type | Description |
|:--- | :----  | :------------------ |
| `add_locations` | manipulation_msgs::AddLocations | To add a list of locations from the *Location Manager* |
| `remove_locations` | manipulation_msgs::RemoveLocations | To remove a list of locations from the *Location Manager* |

### Skill Action *PickObjects*

The *PickObjects* module provide the following actions/services servers:

| Actions | Type | Description |
|:--- | :----  | :------------------ |
| `GROUP_NAME/pick` | manipulation_msgs::PickObjects | Standard action, used to pick an object from a box |


| Services | Type | Description |
|:--- | :----  | :------------------ |
| `add_boxes` | manipulation_msgs::AddBoxes | To add a list of boxes |
| `remove_boxes` | manipulation_msgs::RemoveBoxes | To remove a list of boxes |
| `add_objects` | manipulation_msgs::AddObjects | To add a list of objects to a specific box N.B. an object need to be always contained in a box |
| `remove_objects` | manipulation_msgs::RemoveObjects | To remove a list of objects from all the available boxes  |
| `list_objects` | manipulation_msgs::ListOfObjects | To get a list of all the objects available for picking providing the names and the types |
| `inbound/reset_box` | manipulation_msgs::ResetBoxes | To reset all the objects in a list of boxes |


### Skill Action *PlaceObjects*

The *PlaceObjects* module provide the following actions/services server:

| Actions | Type | Description |
|:--- | :----  | :------------------ |
| `GROUP_NAME/place` | manipulation_msgs::PlaceObjects | Standard action. Used to place an object in a slot |


| Services | Type | Description |
|:--- | :----  | :------------------ |
| `add_slots_group` | manipulation_msgs::AddSlotsGroup | To add a list of slots groups |
| `remove_slots_group` | manipulation_msgs::RemoveSlotsGroup | To remove a list of slots groups |
| `add_slots` | manipulation_msgs::AddSlots | To add a list of slots N.B. a slot need to be always contained in a group |
| `remove_slots` | manipulation_msgs::RemoveSlots | To remove a list of slots |
| `remove_obj_from_slot` | manipulation_msgs::RemoveObjectFromSlot | To remove a single object from a specific slot  |
| `outbound/reset_slot` | manipulation_msgs::ResetSlots | To reset a list of slots from the objects contained |

### Skill Action *GoToLocations*

The *GoToLocation* module provide the following actions/services servers:

| Actions | Type | Description |
|:--- | :----  | :------------------ |
| `GROUP_NAME/go_to` | manipulation_msgs::GoTo | Standard action. Used to move a group in a position |



## Work in progress
The manipulation framework package is continuosly evolving. If you find errors or if you have some suggestions or if you want to contribute  [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/manipulation/issues).

## Developer Contact
**Authors:**   
- Enrico Villagrossi (enrico.villagrossi@stiima.cnr.it)  
- Manuel Beschi (manuel.beschi@unibs.it)  


_Software License Agreement (BSD License)_    
_Copyright (c) 2021, National Research Council of Italy, Institute of Industrial Technologies and Automation_    
_All rights reserved._
