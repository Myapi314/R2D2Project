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
* opencv_subscriber

We used an arduino nano to handle the control of our motors and LEDs which communicated with our robot via a serial USB connection (see [arduino.md](my_id_robot/my_id_robot/ArduinoControl/arduino.md)). 

# Development Environment
Raspberry Pi 4

Ubuntu 22.04

ROS2 Humble

Used [Circuit Diagram](https://www.circuit-diagram.org/editor/) for creating schematic layouts.

ArduinoIDE, Visual Studio Code, Python, OpenCV, Vosk

### Setup environment 
Check if virtualenv is installed \
```virtualenv --version``` 

To install: \
```sudo apt update``` \
```sudo apt install python3-virtualenv``` 

Setup virtualenv in ros2_ws directory \
```virtualenv -p python3 ./venv``` 

Activate environment: \
```source ./venv/bin/activate``` 

Ignore the virtual environement folder (venv) by creating a file named COLCON_IGNORE within the folder. \
```touch ~/ros2_ws/venv/COLCON_IGNORE```

Install dependencies from requirements.txt. \
```(venv) $ python -m pip install -r src/requirements.txt```\
Be sure to have the virtual environment active and to update this file when adding new dependencies. \
```(venv) $ python -m pip freeze > src/requirements.txt```

[Venv Python Docs](https://docs.python.org/3/tutorial/venv.html)

Note: For the robot to launch on bootup of computer it required having dependencies installed globally. This command seemed to do the trick for the few modules that this was an issue for: \
```sudo -H pip3 install -U -I <package_name>```\
Or according to the ros2 docs this command can be used: \
```python3 -m pip install -U <package_name>``` \
Specifically- vosk, sounddevice, RPi.GPIO

Can install all but vosk by uncommenting dependencies in package.xml file and running ```rosdep install --from-paths src -y --ignore-src --reinstall```, however it will give you warnings about using pip as the root. Run without --reinstall flag if not uncommenting those two lines. See [docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) for more on rosdep.

For automatic startup a service file was used ([see here](my_id_robot.service.txt)). This file was created ```sudo nano /etc/systemd/system/my_id_robot.service```. Then run the following commands to set it up: \
```sudo systemctl daemon-reload``` \
```sudo systemctl enable yourscriptname.service``` \
```sudo systemctl start yourscriptname.service``` \
From [Run a script on startup in Linux](https://www.tutorialspoint.com/run-a-script-on-startup-in-linux)

Information for environment variables was found in [ROS2 Humble Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html). Recommended to follow documentation for adding sourcing to shell startup script. There is also information under Demos/Logging for the ROS_LOG_DIR variable.

Anytime the service file is edited, run the reload command. Can stop and restart the service file using: \
```sudo systemctl stop turtlebot4.service``` \
```sudo systemctl start turtlebot4.service``` 

Can view logs from the service with either \
```sudo journalctl -u my_id_robot -r``` \
```sudo systemctl status my_id_robot``` \
From [How Does the ROS2 Turtlebot4 Service Launch When the TurtleBot Boots Up?](http://iotdesignshop.com/2022/11/06/how-does-the-ros2-turtlebot4-service-launch-when-the-turtlebot-boots-up/)

# Hardware
Replaced Arduino Nano with custom board (see __.zip)

Camera, projector, mic, motor drivers

# Schematic
* [Schematic Diagram](https://crcit.net/c/94c71480c5b7491aa2f13e43693fd637)
* [Arduino Sensor Schematic](https://crcit.net/c/a2194848292040c281df357a803242cb)
* [Potential Layout for Additional Arduino](https://crcit.net/c/a2194848292040c281df357a803242cb)

# Resources
* [ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
* [Robotics Backend ROS2 Interfaces](https://roboticsbackend.com/ros2-create-custom-message/)
* [Arduino Nano Docs](https://docs.arduino.cc/static/6442e69a615dcb88c48bdff43db1319d/A000005-datasheet.pdf)
* [R2D2 Build Instructions](https://myr2d2build.com/build)
* [GitHub Reference](https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/How-to-push-an-existing-project-to-GitHub)


# Future Work
- Interfacing with camera
- Use arduino for interrupt based interfacing with sensors
- Using Jetson Nano in place of Raspberry Pi
- Lightsaber Mechanism
