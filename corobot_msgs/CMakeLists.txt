cmake_minimum_required(VERSION 2.8.3)
project(corobot_msgs)

find_package(catkin REQUIRED
    COMPONENTS message_generation std_msgs sensor_msgs genmsg)

  # Declare the message files to be built
  add_message_files(FILES
    SensorMsg.msg
    GPSFix.msg
    GPSStatus.msg
    MotorCommand.msg
    MoveArm.msg
    PanTilt.msg
    PosMsg.msg
    PowerMsg.msg
    ServoPosition.msg
    ServoType.msg
    state.msg
    takepic.msg
    videomode.msg
  )


  # Actually generate the language-specific message and service files
  generate_messages(DEPENDENCIES std_msgs sensor_msgs)

  # Declare that this catkin package's runtime dependencies 
  catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
  )
