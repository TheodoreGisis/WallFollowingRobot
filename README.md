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
  
  
This is my Wall Following robot using Arduino UNO. The goal of the current robot is to find and keep following the left wall, and that's why we use two ultrasonic sensors. The first ultrasonic sensor has connected to the front side of the robot, and it's responsible to detect and avoid obstacles. The second one is to the left side, and it's responsible to find the left wall and continue to drive parallel with it.

To make my robot drive smoothly, I design two PID controllers. One PID is responsible to keep my robot in a straight line and the other to help my robot take a smoothly left turn.
