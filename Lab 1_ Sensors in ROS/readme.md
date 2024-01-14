## Frequently Asked Questions (FAQ) - Lab1

### Environment

**Q1: Where can I download TurtleBot3 packages for my catkin workspace?**  
*A1: To download TurtleBot3 packages for ROS Noetic, follow these commands. This will clone the necessary repositories and install the TurtleBot3 bringup package:*

```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
sudo apt install ros-noetic-turtlebot3-bringup
cd ~/catkin_ws && catkin_make
```

**Q2: I'm getting an error "/usr/bin/env: python: No such file or directory". What should I do?**  
*A2: This error typically occurs due to Python version mismatches, often when scripts are expecting Python 2 but only Python 3 is installed. To resolve this, you can create a symlink for Python 3 as 'python'. Run the following command:*

```
sudo apt install python-is-python3
```

*This command will set Python 3 as the default version when you invoke 'python' in the terminal.*

