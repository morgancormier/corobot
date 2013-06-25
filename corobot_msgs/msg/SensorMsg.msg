# This message can be used to send some simple sensor data that require only one value. The type of the sensor can be unkown.

#define sensor type
int8 BUMPER = 0
int8 OTHER = 1
uint8 ULTRASOUND=2
uint8 INFRARED_FRONT=3
uint8 INFRARED_REAR=4

# type of sensor, defined above
int8 type

# index of the sensor, for example index on the Phidget Interface Kit. This can be used to filter data when the sensor type is unknown, but the user know the data type
int8 index

# Value of the sensor, it can be digital in which case the value will be 0.0 or 1.0, or analog in Volts. If the type of the sensor is known, for example ultrasound or infrared, the value is in the corresponding standard format, for example meters. 
float64 value
