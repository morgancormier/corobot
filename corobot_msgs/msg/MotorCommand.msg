# Bill Mania <bmania@coroware.com>
#
# Service used with the CoroWare CoroBot to move the robot
# base. Originally developed for the Phidget HC Motor Controller
#
# leftSpeed and rightSpeed are signed integers which specify the
# combined speed and direction to rotate the wheels on each side
# of the robot. 0 indicates full stop. positive integers indicate
# a forward rotation and larger integers indicate a higher
# rotational speed.
#
int16 leftSpeed
int16 rightSpeed
#
# when at least one of leftSpeed or rightSpeed is non-zero,
# secondsDuration indicates for how many seconds to rotate those
# wheels at the requested speed. when secondsDuration is zero,
# the wheels will be rotated at the requested speed, in the
# requested direction until another request is received.
#
uint16 secondsDuration
#
# the number of seconds over which to effect the change in
# rotational speed from the current speed to the speed requested.
# a value of zero indicates "as fast as possible". the greater
# this value, the greater amount of time taken to change the
# speed.
uint16 acceleration
