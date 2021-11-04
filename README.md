# WallFollowingRobot

## REQUIREMENTS:

* Arduino Uno board
* 2 x Ultrasonic sensors
* Mini breadboard
* 8  x 1300 mAh 1.2V NiMH batteries
* Car Kit 

## Desctription
<p align="center">
  <img width="430" height="470" src="https://github.com/TheodoreGisis/WallFollowingRobot/blob/main/Robot/ROBOT.jpg" >
</p>
  
This is my Wall Following robot using Arduino Uno.The current robot follows the left wall, it has two ultrasonic sensors one in the front of the vehicle and one on the left side.
The first ultrasonic sensor has connected to the front side of the robot and it's responsible to detect and avoid obstacles.The second one is to the left side and it's responsible to find the left wall and continue to drive parallel with it.
To make my robot drive smoothly i design two PID controllers.One PID is responsible to keep my robot in straight line and the other to help my robot take a smoothly left turn.
