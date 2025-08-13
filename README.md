# SmartStride
This repository contains all of the code and information for the SmartStride system. All of the code in this is for Arduino, the specific model we used is the Arduino UNO R3.

The programs in this will only be able to upload to your Arduino if you have the Arduino IDE and you have a wired connection to your Arduino UNO R3.

The Smart Pedestrian Crossing System (SPCS) which uses sensors and smart switches to help illuminate the crossings to improve visibility for pedestrians while alerting drivers.
The Smart Pedestrian Crossing System (SPCS) consists of:
LiDAR Range Finder Sensor to detect pedestrian at the crossing
Arduino Uno REV3
LED Strobe Lights mounted on a pedestrian sign
DC power source - solar powered battery pack - such that it can be added to existing infrastructure or minimize requirements for deployment
Other items:
Solar power management module to regulate power supply from solar panel to  rechargeable battery pack
DC to DC Boost converter for LED strobe lights (from power source to LED strobe lights)

How it works:
The LED strobe lights will be activated when the LiDAR detects a pedestrian for a specified duration. It then resets after 20 seconds.

This can be extended to include:
Photocell - to activate this only when it is dark. This also aids in power consumption.
LED spotlights to illuminate the crosswalk when a person is detected

For more information on this project and details - please visit:
