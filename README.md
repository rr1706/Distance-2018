# Distance 2018 (FRC PowerUp)
Distance Sensor Controller - Arduino

This is a basic arduino project that uses Software i2c to talk to multiple VL53L0X laser Time of Flight (rangfinder) sensors.  The sensor readings are used to determine if the robot has a cube in position and ready for pickup.

Four sensors (#'s 0 - 3) are used across the front of the robot to detect the prsence of a power cube.

A fifth sensor is used in the gripper to determine if we have a good hold on a cube.

Three analog outputs are used as digital outs to send signals to the RoboRIO robot controller.
    * A0 -->  Gripper  (Active Low)
    * A1 -->  Cube in good position (Active High)
    * A2 -->  Cube in poor but actionable position (Active High)
    
