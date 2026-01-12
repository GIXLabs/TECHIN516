# lab 4: Maze Solver

![turtlebot4_lidar_frame](/assets/turtlebot4_lidar_frame.png)

In the last lab you learned about open versus closed loop control.  
These simple examples could only drive straight, in this lab we explore more advanced closed loop control to solve mazes.  
Basic mazes can be solved by staying close to a wall.  
You will tune a PID controller to maintain a constant distance from a wall.  
You could use a similar controller to stay in the middle of a road lane, or control the speed of an actuator.  


## Learning Objectives

- Begin developing autonomous robotic systems.
- Practice tuning a PID controller.
- Apply simulation tested systems on the real robot.


## TODO

1. Create a new python package called `lab4` and copy in the `wall_follower.py` script.  

2. Complete all `TODOs` in the script.  
Reference the photo above, notice how the LiDAR frame is different than the robot's base frame.  

3. Create a copy of your `plot_odom.py` script as `plot_wall_distance.py` in the new package.  
Modify the script to plot distance from the wall over time.  
The deliverable will ask for min and max oscilations, you can make this script print those values for convenience.  

4. Copy the `walls.sdf` Gazebo world into the worlds folder from the previous lab.  

5. Test your wall_follower script in the `walls.sdf` world.  
For each of the following table rows, change the P, I, and D values and plot the results to measure the oscilations as the robot completes a loop around the walls. 

    | Case | Kp | Ki | Kd |
    | - | - | - | - |
    | No PID | 1 | 0 | 0 |
    | Only P | 1.2 | 0 | 0 |
    | PI | 1.3 | 0.1 | 0 |
    | PD | 1.2 | 0 | 0.1 |
    | PID | 1.2 | 0.25 | 0.2 |
    | Set your own | - | - | - |

    You can build your workspace with `colcon build --symlink-install`.  
    This enable you to edit python files without re-building.  
    Instead of copying files into the `install` directory, this creates a "symbolic link" to the file instead.  

6. Once you have found values that work for your robot, test the script in the existing `maze.sdf` world, plot your results.  

7. Test your values on the real robot against the flat wall in the robotics lab, take a video and plot the results.    


## Deliverables

1. Submit your completed `wall_follower.py` script.

2. Submit your code to plot wall distance.

3. What is the minimum and maximum oscilations for each of the PID value rows?  

4. What custom values did you choose?  
How did you choose these values?

5. Submit your wall-distance plot from running the simulation with your custom PID values.  

6. Qualitatively, how well does this script explore the maze world?  
Does it explore the whole area?  
How would you suggest to improve the controller to explore more efficiently?  

7. Attach the video of the real-world experiment.

8. Attach the plot of the real-world experiment.

9. How did the real-world compare to the simulation?  
Quantitatively, how did the min and max oscilation values compare in each scenario?  
Qualitatively, did the real-world behave differently than the simulation?


## FAQ

**Q:** In simulation, the LiDAR has a very short range, what do I do?  
**A:** There is a bug where the LiDAR sometimes doesn't simulate properly, try one of these solutions: [[1](https://robotics.stackexchange.com/questions/24883/turtlebot-4-simulation-rplidar-not-working), [2](https://www.linkedin.com/posts/rahgirrafi_ros2-turtlebot4-gazebo-activity-7284297279061028864-JWBu?utm_source=share&utm_medium=member_desktop)].

**Q:** How do I connect to the real robot?  
**A:** Refer to the [connection guide](/docs/connecting_to_the_turtlebots.md) in the docs.  

**Q:** These plots are taking forever, my laptop runs Gazebo really slowly, what do I do?  
**A:** You can change the script so the P, I, and D values can be provided as ros parameters or python arguments, then you can script your computer to run simulations sequentially, then you can run it overnight and wake up to your completed work.  


## Resources

[Micromouse mini documentary](https://www.youtube.com/watch?v=ZMQbHMgK2rw)  
Competitions are a great driver of robot advancement.  
The [Micromouse](https://en.wikipedia.org/wiki/Micromouse) competition has hugely pushed autonomous maze solving.  
In this lab we only build the most basic strategy - wall following.  
Check out the video to learn more about advanced strategies.  

[Mike Likes Robots PID Controller Blog Post](https://mikelikesrobots.github.io/blog/understand-pid-controllers/)  
PID controllers are common in robotics, so there are many good resources to understand them further.  
The above blog post is a good starting point with its interactive visualizations.  

[Official ROS 2 PID controller documentation](https://control.ros.org/humble/doc/ros2_controllers/pid_controller/doc/userdoc.html)  
One benefit of ROS is that we can re-use components from the community.  
The `ros2_control` community supports many common controllers, including PID.  

[ROS 2 DiffBot & PID Controller example](https://github.com/ros-controls/ros2_control_demos/tree/master/example_16)  
ros2_control includes example differential drive and PID controller examples.  
In the latest LTS version of ROS 2 (Jazzy), these controllers can be "chained" together.  
The PID controller can use a topic as an "external measure state" to use measurements outside of the standard command and state interfaces.  
