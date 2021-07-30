# Auto Cup Filler

Have you ever been working on your computer and suddenly your cup of coffee was empty? 
And you have no other option than to stand up and walk to get a new drink?
If you reply yes to both of these questions I got a solution for you
**Auto Robotic Cup Filler**

<img src="https://github.com/tomash1234/robot-arm-filler/blob/main/doc/overview_small.png" width="300">

A python script running on your laptop is scanning the table using a webcamera.
Once an empty cup is detected, the robotic arm, with a hose connected 
to a pump in tank with your favourite liquid, quickly moves and fills your cup. 

## Viewer

## How to Install & Run
* Build or buy a Robo Arm, measure its dimensions and write them into
  the config file (**config.json**)
* Clone this repository
* Find Arduino sketch (sketch_robo_arm, sketch_pump_controller and upload it to your board
* Download python and required packages (SEE: dependencies)

* Connect the board and laptop to same WiFi network 
  (I use a hotspot on my laptop)
    
* Set the board IP address and port into **main.py** file and **webc.py**  
* Run **main.py** to start GUI (without autofilling) OR 
* Connect your webcam to see the desk and run **webcam.py** script

### Dependencies
* Numpy   `pip install numpy`
* OpenCV  `pip install opencv-python`
* Matplotlib  `pip install matplotlib`


### Hardware

The Robo-arm is made of 3 servos, wooden sticks and hot glue. 


The Arm and pump is controlled using an esp-8266 board (Wemos D1 R2), 
which is connected via WiFi to laptop, where a python script is running.


### Arm Dimensions

The Arm dimensions are stored in JSON file **config.json**,
write dimensions of your robo arm in this file.

<img src="https://github.com/tomash1234/robot-arm-filler/blob/main/doc/dimensions.jpg" width="600">

