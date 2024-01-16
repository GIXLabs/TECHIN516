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


### ROS

**Q3: Where can I add the robot model in RViz?**  
*A3: To add the robot model in RViz, use the 'Add' button located at the bottom of the left panel. This is different from the '+' icon on the top bar. The 'Add' button allows you to add various display types, including 'RobotModel', to your RViz session.*

**Q4: After adding the robot model in RViz, why are there multiple errors in the Global Status?**  
*A4: If you encounter errors in the Global Status after adding the robot model, it may be due to an incorrect fixed frame setting. To resolve this, change the fixed frame from 'map' to 'odom' in the RViz settings. This adjustment often rectifies common Global Status errors related to frame reference.*

**Q5: What should I do if I receive an RLException error when trying to launch 'turtlebot3_remote.launch'?**  
*A5: If you encounter an RLException stating that 'turtlebot3_remote.launch' is neither a launch file in the package 'turtlebot3_bringup' nor a launch file name, the issue is likely due to the absence of the required package. Install the 'turtlebot3_bringup' package for ROS Noetic by executing the following command in the terminal:*

```
sudo apt install ros-noetic-turtlebot3-bringup
```

*This command installs the necessary TurtleBot3 bringup package, which should resolve the launch file error.*

**Q6: How can I read rqt_graph?
*A6: Rqt graph is a useful tool to see what’s happening in your ROS graph. Please refer to the following link to learn more:
[rqt_graph](https://roboticsbackend.com/rqt-graph-visualize-and-debug-your-ros-graph/#:~:text=Rqt%20graph%20is%20a%20GUI,be%20displayed%20inside%20their%20namespace.)


