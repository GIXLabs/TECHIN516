# Lab 3: Intro to Simulation

![intro_to_simulation](/assets/turtle_in_the_matrix.png)

In the previous lab, you ran the real robot without fully knowing what was going to happen.  
This is fine for a small, slow Turtlebot, but can lead to serious injury in larger, faster systems.  
This lab explores how to test robot systems in simulations, so we can safely try different control strategies.


## Learning Objectives

- Simulate a robot for safe experimentation.
- Compare open versus closed-loop control in simulation.  
- Extract experiment result values and visuals from ROS for research comparisons. 


## TODO

1. Move the `flat.sdf` file into the devcontainer and open it in Gazebo.  
Review the [Turtlebot 4 simulation documentation](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) to learn which launch file to use to open Gazebo.  
Read through that launch file to see which folder it looks for world files in.  
Move the `flat.sdf` file into that folder, and provide it as an argument to open the Gazebo simulation. 
Drive around using the teleop node to confirm everything works.  

2. Comment out dock spawning in the simulation launch file.
Read through the launch file(s) again to identify which lines spawn the Turtlebot 4's docking station.  
Comment out those lines so the dock doesn't get in the way of immediately driving around.  

3. Complete the `TODOs` in `plot_odom.py`.  
Create a new ROS2 python package, and copy the script into the package to run the script using `ros2 run`.  
Test it while running the `circle_driver` script in the above simulation environment.  

4. Create a new Gazebo world called `cube.sdf`.  
Copy the `flat.sdf` Gazebo world and rename it in the same folder as before.  
Review the [Gazebo documentation](https://gazebosim.org/docs/latest/getstarted/), the [.sdf documentation](http://sdformat.org/tutorials) and/or other world files to learn how to spawn a cube.  
The cube should be 1m on all sides, and 3 meters directly in front of the robot when it spawns.  

5. Finish this script for open loop control; plot the results 3 times.
The goal is to move 2 meters forwards.  
Choose a speed, and calculate how much time is required to move 2m at that speed.  
Run the script 3 times while plotting the results.  

6. Finish this script for closed loop control; plot the results 3 times.  
This script will use the LiDAR sensor to check how much the robot has moved.  
Copy this script into your `lab3` ros2 python package, complete all `TODOs`, and run it 3 times while plotting the results.  


## Deliverables

1. Write down which file you modifed to remove the dock from simulation, and copy in the lines you commented out.

2. Submit your modified code that you used to plot the robot's position.  

3. Attach the plot from running your circle_driver script.

4. Submit your modified `.sdf` file to show how you added the cube.  

5. Submit your modified open-loop code,

6. Attach your plots from running open-loop control.

7. Submit your modified closed-loop code.  

8. Attach your plots from running closed-loop control. 


## FAQ

**Q:** I can't run the turtlebot packages, why not?  
**A:** Make sure they were cloned in properly, this repo has the turtlebot packages as submodules.  
Either re-clone the repo or update the submodules and rebuild the container.

**Q:** Gazebo runs suuuuuper slowly, is there any way to speed it up?  
**A:** Look into the Gazebo world's `.sdf` file. Consider changing the `<max_step_size>` parameter and/or adding a `<real_time_update_rate>` parameter as described [in this robotics stack exchange post](https://robotics.stackexchange.com/questions/26259/what-does-update-rate-control).

**Q:** Lidar works in the flat world, but not the cube world, why?  
**A:** All fields in the sdf file need to be consistent, including the title and the world name or else you might experience strange issues.

**Q:** How do I save my plots?  
**A:** Matplotlib has a save button in the pop-up window.  
You can also modify the code to save the plot automatically. 

**Q:** Sometimes when I relaunch Gazebo, there's red text that says the "controllers" don't work?  
**A:** Try re-launching. If that continues to not work, exit the devcontainer and rebuild it; just make sure you save all your stuff first.  


## Resources

[Turtlebot 4 user manual](https://turtlebot.github.io/turtlebot4-user-manual/)  
Any well-made robot you use should have documentation.  
This documentation should be the first place you look if you have any questions or issues.

[Turtlebot 4 code](https://github.com/turtlebot)  
For code issues, the next best place to look is the Github issues of the code you're using.  
This Github organization manages all production code for the Turtlebot 4.

[Articulated Robotics ROS devcontainer guide](https://www.youtube.com/watch?v=dihfA7Ol6Mw&t=475s)  
This video does a great job explaining how and why to use docker to develop robotics projects.

[Gazebo Documentation](https://gazebosim.org/docs/dome/getstarted/)  
Review the Gazebo documentation to learn more about robotic simulation. 

[Black Coffee Robotics Guide to Speed Up Gazebo Simulations](https://www.blackcoffeerobotics.com/blog/gazebo-simulator-5-ways-to-speedup-simulations)  
Black Coffee Robotics is an engineering firm specializing in robotics.  
They have a lot of experience running efficient simulations and wrote this blog post to share some of their expertise.  