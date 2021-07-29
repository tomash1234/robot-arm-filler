# Auto Cup Filler

Have you ever been working on your computer and suddenly your cup of coffee was empty? 
And you have no other option than to stand up and walk to get a new drink?
If you reply yes to both of these questions I got a solution for you
**Auto Robotic Cup Filler**

<img src="https://github.com/tomash1234/robot-arm-filler/blob/main/doc/overview_small.png" width="300">

A python script running on your laptop is scanning the table using a webcamera.
Once an empty cup is detected, the robotic arm, with a hose connected 
to a pump in tank with your favourite liquid, quickly moves and fills your cup. 



## Hardware

The Robo-arm is made of 3 servos, wooden sticks and hot glue. 


Arm and pump is controlled using an esp-8266 board (Wemos D1 R2), 
which is connected via WiFi to laptop, where a python script is running.


## How to Install


## Dependencies
* Numpy
* OpenCV
* Matplotlib 

## Arm Dimensions

<img src="https://github.com/tomash1234/robot-arm-filler/blob/main/doc/dimensions.jpg" width="600">
