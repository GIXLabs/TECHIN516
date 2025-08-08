# Lab 5: Mapping and Navigation

Currently, most of the time, robots are used repeatedly in contained, controlled environments.  
In environments like this, autonomous navigation is enabled by creating maps, and tracking the location of the robot using its sensors.  
In this lab we will explore how to create maps, and how to navigate with them.  


## Learning Objectives

- How to create 2D environment maps.
- How to use those maps to enable autonmous navigation.
- A basic understanding of algorithms for autonmous navigation. 


## TODO

1. Checkout the [documentation](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html), open the maze world using a turtlebot4 lite with `slam`, `nav2`, and `rviz` enabled.  

2. Either teleop, or use your maze solver script to wander around the maze, completeling the whole map.  

3. Save the map to a folder called "maps" in a python package called lab5.  
Rename the files from "map" to "gazebo_maze" to avoid new maps overwriting them.  

4. Stop all of your terminals, [use the navigation launch file]() to reopen the simulation using your map.  
Also make sure to start the nav2 launch file.

5. Use the "2D Pose Estimate" tool to roughly locate the robot on the map.

6. Use the "Nav2 Goal" to send goals to the robot.  
Play around with the waypoints tools to send multilpe goals.  

7. Go through the same process with the real robot.  
Create a new map of the mini mazes in the robotics lab.  
Send nav goals to the real robot.  

8. Complete the `a_star.py` script in this folder.  
The code should work in your Jupyter Notebook lab report, or you can run the script from the command line. 


## Deliverables

1. Attach your map of the real-world maze.  

2. Attach a video of your robot autonomously navigating the maze.  
Show yourself setting the nav goal, then the robot completing the task.  

3. Include your code from `a_star.py`.

4. Attach the completed maze from running your script.  


## FAQ

**Q:** My robot has a hard time moving around tight spaces, even though there's enough room, how do I make it work?  
**A:** Checkout [this tutorial](https://www.youtube.com/watch?v=y2eq_yfpAQM&list=LL&index=12) from Muhammad Luqman about tuing the navigation parameters.  


## Resources

[Understanding SLAM guide from MATLAB](https://www.youtube.com/watch?v=saVZtgPyyJQ&list=PLn8PRpmsu08rLRGrnF-S6TyGrmcA2X7kg&index=3)  
MATLAB is an awesome computing platform for engineers and scientist, from research to production.  
Their YouTube chanel has a lot of awesome content.  
This video neatly summarizes how these mapping and navigation systems work.  
Checkout the [whole playlist](https://youtube.com/playlist?list=PLn8PRpmsu08rLRGrnF-S6TyGrmcA2X7kg&si=gvzXuFdEGRW-ewbw) to learn more about autonomous navigation.  

[A* explanation by Computerphile](https://www.youtube.com/watch?v=ySN5Wnu88nE)  
Computerphile's channel is also full of great content specifically for computer science, many of which also apply to robotics.  