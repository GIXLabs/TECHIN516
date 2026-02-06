# Lab 5: Mapping and Navigation

![solved_maze](/assets/a-star.png)

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

4. Stop all of your terminals, [use the navigation launch file](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html) to reopen the simulation using your map.  
Also make sure to start the nav2 launch file.

5. Use the "2D Pose Estimate" tool to roughly locate the robot on the map.

6. Use the "Nav2 Goal" to send goals to the robot.  
Play around with the waypoints tools to send multilpe goals.  

7. Go through the same process with the real robot.  
Create a new map of the mini mazes in the robotics lab.  
Send nav goals to the real robot.  

8. Run both Dijkstra and A* planner algorithms.  
Once you have a map of the real maze, make the robot navigate from one corner to anoother using both path planning algorithms.  
First, look into the [config file](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/config/nav2.yaml) and [documentation](https://docs.nav2.org/index.html) to learn which planner is used, and how to switch.  
For each algorithm, navigate from one corner to another.  
Make sure you re-build, and re-source your workspace after changing the `.yaml` file.  

9. Complete the `TODO`s in `lab5.ipynb` in this folder.  
You can use this notebook as the base for your report.  


## Deliverables

1. Attach your map of the real-world maze.  

2. Attach a video of your robot autonomously navigating the maze.  
Show yourself setting the nav goal, then the robot completing the task.  

3. Which planner algorithm is used by default on the Turtlebot4?

4. How do you send a navigation goal from the command line?

5. Include your code from `lab5.ipynb`.

6. Attach the completed maze from running your script.  


## FAQ

**Q:** My robot has a hard time moving around tight spaces, even though there's enough room, how do I make it work?  
**A:** Checkout [this tutorial](https://www.youtube.com/watch?v=y2eq_yfpAQM&list=LL&index=12) from Muhammad Luqman about tuning the navigation parameters.  


## Resources

[hello-slam repo by nstathou](https://github.com/nstathou/hello-slam)  
This repo goes into some of the background hisotry, math, code, and theory that enables map generation, localization, and autonomous navigation.  
Check out the Jupyter notebooks to learn morea about SLAM.  

[Understanding SLAM guide from MATLAB](https://www.youtube.com/watch?v=saVZtgPyyJQ&list=PLn8PRpmsu08rLRGrnF-S6TyGrmcA2X7kg&index=3)  
MATLAB is an awesome computing platform for engineers and scientist, from research to production.  
Their YouTube chanel has a lot of awesome content.  
This video neatly summarizes how these mapping and navigation systems work.  
Checkout the [whole playlist](https://youtube.com/playlist?list=PLn8PRpmsu08rLRGrnF-S6TyGrmcA2X7kg&si=gvzXuFdEGRW-ewbw) to learn more about autonomous navigation.  

[ROS2 Nav2 SLAM documentation](https://docs.nav2.org/index.html)  
Turtlebots use open-source SLAM components from Nav2.  
The beauty of ROS is that these components can be used on many different systems.  
Checkout the official documentation to learn more about how they work, and how to use them.  
The documentation covers many topics and features we don't have time for, like [keepout zones](https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html), [speed limit zones](https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html), and more.

[A* explanation by Computerphile](https://www.youtube.com/watch?v=ySN5Wnu88nE)  
Computerphile's channel is also full of great content specifically for computer science, many of which also apply to robotics.  

[SLAMBotics SLAM System Comparison](https://www.slambotics.org/blog/which-slam-algorithm-should-i-choose)  
There are many type SLAM systems, the one you use should be based on your constraints and resources.  
There are many more systems than what's listed here, but it's a good list to get an overview of what's out there. 
