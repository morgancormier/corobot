#
# Morgan Cormier <mcormier@coroware.com>
#
# Service used to set the type of servo motor connected at the specified index on the phidget servo controller
#
# Index is the index of the servo motor connected to the phidget device. 
# The maximum value of index depends on the Phidget device and how many connections it accepts
#
# type is the type of motor. See enum CPhidget_ServoType of the Phidget library for the list of type.

int8 index

int8 type
