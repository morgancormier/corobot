#
# Morgan Cormier <mcormier@coroware.com>
#
# Message used to set or get the position of a servo motor connected to a Phidget servo controller.
#

####### Possible Indexes #########
uint8 BASE_ROTATION = 0
uint8 SHOULDER = 1
uint8 ELBOW = 2
uint8 WRIST_FLEX = 3
uint8 WRIST_ROTATION = 3
uint8 GRIPPER = 4
uint8 LEFT_WHEEL = 5 # only used in case the left wheel is a stepper motor
uint8 RIGHT_WHEEL = 6 # only used in case the right wheel is a stepper motor
# Index is the index from the list of possible index. 
int8 index

# position is the angle in degree set for the servo motor selected with index
float32 position
