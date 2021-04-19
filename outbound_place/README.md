# outbound place #

The PlaceObjects class provide the following actions/services:
 
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

