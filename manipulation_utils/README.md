
# manipulation_utils #

The manipulation_utils package implement all the basic classes to build a Skill Action.

## Description

The following definitions are used for the framework basic modules explanetion:

- Task: is the  final  goal  to  be  reached  by  the  roboticsystem represented by a group of Skill Actions. The Task decomposition into single Skill Action is made by a task planner that assign single Skill Action to the manipulation framework.

- Skill Action: is a single action that represent an elementary skill such aspick, place, screw, etc....

- Sub Action: is  the  elementary movement executed by the robotic system, a group of Sub Action forms a Skill Action. For example, the pick Skill Action can bedecomposed in: move to approach position, move to object position, close the gripper, move to leave position.

![XXXX](../documentation/Task_SkillAction_SubAction.png)


The hierarchical layers


![XXXX](../documentation/manipulation_framework_scheme.png)

The manipulation framework pipeline is the following:

![XXXX](../documentation/SkillAction_scheme.png)


The SkillBase module is the following:

![XXXX](../documentation/SkillBase_scheme.png)


The LocationManager module is the following:

![XXXX](../documentation/LocationManager_scheme.png)

