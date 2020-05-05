Autonomous Stewart Platform
=============================

This Arduino program was used to control a 6-servo Stewart Platform equipped with a resistive touch screen. A maze was fixed to the platform on the touchscreen, with the goal of manipulating a marble to complete the maze. The touchscreen provides real-time positional data, which is then used to calculate servo angles through inverse kinematics, in order to guide the marble. 

Autonomous operation of the platform is shown in the gif below.

![](/demo/auto.gif)

![](/demo/autonomous.gif)

The platform can also be controlled manually using a joystick, although it is clear that autonomous operation can solve the maze faster than manual operation.

![](/demo/manualmode.gif)

The maze is divided into zones, with a target position for each zone as shown in the figure below. The axis of rotation of the platform is determined as the axis perpendicular to the vector from the current position of the marble to the target position. The angle of rotation of the platform is proportional to the length of this vector, i.e. farther the marble from the target point, steeper the orientation of the platform. Once a target is hit, the target moves to the next target position, and this process continues till the final target is hit.

<img src="/demo/zones.JPG" width=300>
