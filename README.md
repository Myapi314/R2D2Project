# R2D2Project
Senior Project Spring 2023

# Overview
This is the repository for development done on an R2D2 robot using a raspberry pi/Jetson Nano. The R2D2 robot was originally from a hobby project that fans of the Star Wars universe could build and put together themselves. However, all of the electronics and internal software came as is, with little to no ability to make your own hardware or software improvements. Our team then took apart the existing robot and rebuilt his insides to improve existing functionality and expand his capabilities. 

This project used ROS2 to run the bulk of the project on our main computer (Raspberry Pi / Jetson Nano). This includes several nodes:
* main_subscriber
* voice_publisher
* arduino_subscriber
* ir_publisher
* ultrasonic_publisher

We used an arduino nano to handle the control of our motors and LEDs which communicated with our robot via a serial USB connection. 

# Development Environment


# Resources
* [Arduino Nano Docs](https://docs.arduino.cc/static/6442e69a615dcb88c48bdff43db1319d/A000005-datasheet.pdf)
* [GitHub Reference](https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/How-to-push-an-existing-project-to-GitHub)
* [Schematic Diagram](https://crcit.net/c/94c71480c5b7491aa2f13e43693fd637)
* [Arduino Sensor Schematic](https://crcit.net/c/a2194848292040c281df357a803242cb)
* [R2D2 Build Instructions](https://myr2d2build.com/build)


# Future Work
