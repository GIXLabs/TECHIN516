# Lab 8: Putting It All Together

![project example](/assets/behavior_tree.png)

In this final lab, you will compete against your classmates to see who can finish a maze the fastest.  
You will need to apply all the lessons so far, modify, and assemble your code from previous labs into a single application.  

The instructors will design a randomized maze with an aruco cube somewhere inside.  
Everyone will begin at the same starting position on the Turtlebot 4's dock.  
The robot will need to complete the following sequence:

1. Search the maze and generate a map. 
2. Use computer vision to identify where the aruco cube is in the map.
3. Return to the start position.
4. Drive from the start position to within 10cm of the aruco cube.

The total time required to complete the sequence above will be measured and ranked against your classmates.  
You will need to apply everything you've learned so far; you are also encouraged to try new things.  
You are not allowed to change the code on the robot, the physical configuration of the robot, or the physical environment.  
**You will only be allowed to start one program in one terminal.**

The above steps represent a very simple, linear "behavior tree".  
Behavior trees can be represented as graphs, with different actions and applications interconnected based on the outcomes of previous actions.  
Once you design how you want your robot to perform in different scenarios, you can label each behavior as a "state".  
Then you can update the state of the robot based on the outcome of the previous state to match the connections of your behavior tree.  
This style of application can also be thought of as a "state machine".  

For example, in this race, the first state (1) could be wall-following to map the maze.  
Once the robot sees the aruco cube, it can transition to state two (2) and return to the dock, and so on...  

For this lab, you will write one central ros node that keeps track of the robot state, and calls different "behaviors" as ros2 actions and services.  
Some of the behaviors you need already exist (driving to a point on the map, docking the robot, etc.).  
For other behaviors you will need to modify your code from this quarter to complete.  
The suggested sequence for developing this lab is given below.  


## Learning Objectives

- Learn how to write ROS2 launch files.  
- Learn how to track the state of a robot across steps of a larger application.  
- Gain experience integrating many technologies together in a larger application.


## TODO

The following are suggestions for how to approach this lab.  
You are welcome to work on it any way you would like within the guideline given above.  

1. Start by getting every step in the sequence working using modified code from previous labs.  
i. Make sure your wall-follower script still works.  
ii. Make sure you can identify the aruco cube with the camera.  
iii. Create a ros action that explores the maze, creates a map, and stops when it see the aruco cube.  
iv. Research how to send navigation goals from python instead of Rviz.  
v. Use the Turtlebot's built-in docking function.  

2. Write a new node to orchestrate the full sequence.  
This node should keep track of which step, or "state" the robot is in the sequence to decide what the robot should do next.  
The robot should only switch states based on the results of previous behaviors.  

3. Write a launch file to run the full application.  
You will only be allowed to run this launch file in one terminal.  
Any other interaction with your computer while the robot is performing will add a penalty to your final time.  

4. Once the full application is working, make some guesses about where you can decrease time the most.  
Work on upgrading those steps to improve the whole system.  
Keep things modular so that the same orchestration node interacts with these upgrades in the same manner as before.  

5. Explore all of the parameters and variables in the system to find opportunities to fine-tune the system for speed.  
Start by trying different parameters for navigation to see if the robot can go faster.  


## Deliverables

1. Submit your code.  

2. Write a paragraph for each step of the sequence, and a conclusion paragraph:  
Discuss your strategy, challenges, and how you overcame those challenges.  
Conclude with what else you could do to decrease the time required if you had more time, resources, and less constraints.   

3. Draw a behavior tree for a simple in-home robot vaccuum cleaner.


## FAQ

**Q:** How do I define new behaviors and keep track of their states?  
**A:** In ROS, you can develop features and track progress using [actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).  
Refer to the [ROS tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html) to see how to write new actions.  


## Resources

[Official ROS2 Tutorial on Understanding Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)  
ROS2 actions are one way to develop "behaviors".  
They provide a convenient interface for calling, reading feedback, cancelling behaviors, and reading results.  

[OpenCV Guide on Detecting Aruco Markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)  
Aruco codes are like QR codes, except they encode numbers instead of links.  
They are easily identifiable using computer vision.  
If you know the size of the tag, you can estimate the pose without the need for the camera's depth topic.  

[Official ROS2 Launch File Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)  
Launch files are essential when you know you need to [run many ROS nodes at once](https://www.reddit.com/r/ROS/comments/1jn9rl4/you_will_not_regret/).  
Launch files allow you to run many nodes in a single termainl which will save you a lot of time.

[simple_launch](https://github.com/oKermorgant/simple_launch)  
[better_launch](https://github.com/dfki-ric/better_launch)  
The ROS2 launch system is far more complicated than it was in ROS1.  
This has inspired attempts at simplifying it to make it more usable.  
It is still worth learning the default system.  
These alternatives provide another option if you have to write many launch files.  

[Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/index.html)  
Nav2 implements its own behavior trees specifically for handling different scenarios during navigation.  
Check out their documentation to learn more.  

[SMACC - State Machine Asynchronous C++](https://smacc.dev/)  
This is linked to give you an idea of how other state machines are built, and what they are used for.  
You don't need to use this to complete the lab.