# Lab 2: Intro to Turtlebots

![turtlebot3_exploded](/docs/turtlebot3_exploded.jpg)

During this class we will be using [Turtlebots](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) to learn more about robots and autonomy.  
The lab has a selection of Turtlebot 3s and 4s.  
We will start with Turtlebot 3s to help you understand what components are involved in making a simple robot work.  


## Learning Objectives

- What are the necessary components of a basic robot.
- How to set-up a robotic system between the robot and your computer.
- How to programmatically control a differential-drive robot.


## TODO

1. Draw a hardware architecture diagram of a Turtlebot3 including all the numbered components from the photo above.  
Label all hardware components, be specific.  
If information is passed between hardware, label it and be specific.  
Feel free to use which ever tool you're most comfortable with.
Some common tools to draw architecture diagrams are [diagrams.net](), [mermaid.js](), [miro](), and [figjam]().  

2. Follow steps 3.1, 3.2, 3.3, 3.5, and 3.6 of the [quick start guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) to configure the robot and your computer to work together using ROS2.  
Make sure you follow the "Humble" version of the guide.  
In step [3.2. SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup), connect the robot and your computer to the `mkrspc_robot` network.  
In step [3.6](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#teleoperation), you only need to teleop with the keyboard, not a game controller.  

3. On your remote computer, run `rqt` in a terminal while you're connected to the robot and the teleop node is active.  
Open the node visualizer by selecting: Plugins > Introspection > Node Graph. 

4. Use information from the rqt graph to complete the python script in this lab folder.  
Create a new ROS2 python package in the turtlebot workspace called "lab2".  
Copy `circle_driver.py` from this folder into the package's python package.  
You will need to review the graph to decide which topic to publish to.  
After you find the topic, you can use tools like `rqt` or the command line `ros2 topic info <topic>` to see what type of message it uses.  
Make sure to complete the `setup.py` and build your workspace to run the script using ROS.   
**Make sure the robot has a lot of space before you run the script!**  

5. A Turtlebot uses 2 wheels to drive in any direction.  
This is a common style of robot called a "differential drive robot".  
These are common because they are simple; they only need 2 parameters for navigation.  
Review the code to learn more about those parameters.  


## Deliverables

1. Submit your hardware architecture diagram with all Turtlebot 3 components.

2. Submit a video of you teleoperating the Turtlebot.

3. Submit an image of the Node graph.

4. Submit your updated python code.

5. Write in your report which parameters in the ROS message are used to drive the robot.  

6. Submit a video of your script working with the robot.


## FAQ

**Q:** I don't know what a component is called, how should I label it in the diagram?  
**A:** Continue with the lab, more information is given in the guide of step 4.  

**Q:** Someone else's ROS is interfering with mine, what do I do?  
**A:** Make sure you use a unique `ROS_DOMAIN_ID`.  
The guide sets the ID as 30, but you should use a different number than your classmates.  

**Q:** The Turtlebot hit something while driving, what should I do?  
**A:** A Turtlebot probably isn't going to hurt you or much else.  
Hitting something while driving might wear on the motors, but should be fine.  
In the provided code, you can press Control-C in the terminal to stop the script.  
**Considering safety (emergency stops, physical work spaces, sensor-control systems) is vital to everything in robotics!**


## Resources

[Advanced ROS tutorials by Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics)  
Awesome tutorials that go way past the official tutorials into deeper topics.  

[Robotics and ROS interviews and tutorials by Muhammad Luqman](https://www.youtube.com/@robotisim/videos)  
Great videos covering many topics in robotics.

[Robotics Stack Exchange](https://robotics.stackexchange.com/)  
Good for reading through and asking more in-depth questions, search before posting.  

[ROS subreddit](https://www.reddit.com/r/ROS/)  
Good for seeing other peoples' projects and reading through questions.
