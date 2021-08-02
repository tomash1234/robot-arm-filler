# Auto Cup Filler

[![Youtube video](https://img.youtube.com/vi/q4_bpxAipl8/0.jpg)](https://www.youtube.com/watch?v=q4_bpxAipl8)
Check this YouTube video: https://youtu.be/q4_bpxAipl8

Have you ever been working on your computer and suddenly your cup of coffee was empty? 
And you have no other option than to stand up and walk to get a new drink?
If you reply yes to both of these questions I got a solution for you
**Auto Robotic Cup Filler**


<img src="https://github.com/tomash1234/robot-arm-filler/blob/main/doc/overview_small.png" width="300">

A python script running on your laptop is scanning the table using a webcamera.
Once an empty cup is detected, the robotic arm, with a hose connected 
to a pump in tank with your favourite liquid, quickly moves and fills your cup. 

## Viewer
I implemented a simple robot arm viewer and controller. So you can test if everything is setup corrected.
You can click on the plot and the angles needed to reach this position will be calculated
<img src="https://github.com/tomash1234/robot-arm-filler/blob/main/doc/viewer.gif" width="500">

## How to Install & Run
* Clone this repository
* Build or buy a Robo Arm, measure its dimensions and write them into
  the config file (**config.json**)
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

## How does it work

### Cup detector
The Cup detector is a simple circle detector ([Hough circle transform](https://docs.opencv.org/4.5.2/da/d53/tutorial_py_houghcircles.html)) .

The parameters of detector are set up to work for me, so if you have 
a problem with the cup detection, try to adjust these parameters (CupDetector in detectors.py)


### End of arm detector

There is a circular marker on the end of arm. The marker can be detected in same way as a cup, using Hough circle detector.

Again, if you have a problem with the detection, try to adjust the circle parameters.

<img src="https://github.com/tomash1234/robot-arm-filler/blob/main/doc/marker.png" alt="Marker" width="200">
