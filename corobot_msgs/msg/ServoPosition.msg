#
# Morgan Cormier <mcormier@coroware.com>
#
# Message used to set or get the position of a servo motor connected to a Phidget servo controller.
#
# Index is the index of the servo motor connected to the phidget device. 
# The maximum value of index depends on the Phidget device and how many connections it accepts

int8 index

# position is the angle in degree set for the motor selected with the index value. 
# position has a minimum and maximum value possible, that can be retrieve through two services
float32 position
