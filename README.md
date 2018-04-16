# Distance 2018 (FRC PowerUp)
Distance Sensor Controller - Arduino

This is a basic arduino project that uses Software i2c to talk to multiple VL53L0X laser Time of Flight (rangfinder) sensors.  The sensor readings are used to determine if the robot has a cube in position and ready for pickup.

Four sensors (#'s 0 - 3) are used across the front of the robot to detect the prsence of a power cube.

A fifth sensor is used in the gripper to determine if we have a good hold on a cube.

Three analog outputs are used as digital outs to send signals to the RoboRIO robot controller.
    * A0 -->  Cube fully engaged in intake gripper  (Active Low)
    * A1 -->  Cube in good position for pickup (Active High)
    * A2 -->  Robot against wall or field element (Active High)
    * A3 -->  Cube partially engaged in intake gripper (Active High)
    * A4 -->  Robot centered in front of cube stack (Active High)
    

This project uses two modofied libraries:
    * https://github.com/rr1706/vl53l0x-arduino
    * https://github.com/rr1706/SoftwareWire


