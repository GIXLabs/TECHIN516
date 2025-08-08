# Lab 6: Saving and Sending Navigation Goals

Once you have a map for your robot, you can use it for applications.  
This lab explores how to programmatically send and save navigation goals.  
These capabilities can be used to remember semantic room and object locations to increase the autonomy of your system.  


## Learning Objectives

- Begin developing custom interfaces for robotic systems.  
- Connect the robot's spatial understanding with a user's semantic understanding.  
- Learn how to add information to Rviz2 with markers.  
- Begin developing ROS2 nodes from scratch.  


## TODO

1. Create a new package in your `turtlebot4_ws` called `lab6`.

2. Write a new python ros node from scratch called `map_annotator.py`:  
The goal of this lab is to create a package that allows you to save poses (including position and orientation) in the map and assign names to those poses.  
You should then be able to send the robot to those poses by name.  
Additionally, you should be able to list, overwrite, and delete poses.  
The data should be saved between runs so that you can reuse poses after restarting the system.  
This node should have a terminal user interface like the one depicted below:

    ```
    ros2 run lab6 map_annotator
    
    commands:
    - list
    - save <name>
    - delete <delete>
    - goto <name>
    - exit
    - help

    > 
    ```

    `list` should print all of the existing poses.

    `save <name>` should create a new pose at the current position and orientation of the robot.  
    Saved poses should be in the `map` frame to make sure they're in the correct place relative to the environment.  
    Saving a new pose with the same name as an existing one should overwrite the previous pose.  

    `delete <name>` should remove the named pose from the system.  
    If a pose does not exist, the system should inform the user.

    `goto <name>` should publish a navigation goal for the robot to move to.  
    Use the `PoseStamped` message to send navigation goals.

    `help` should print all of the commands again.

    `exit` should save all poses and shutdown the system.

    The user should be able to type commands into the terminal in front of the `>` symbol.

    The node should continuously publish a list of poses for the next part of this lab; use the `geometry_mesgs.msg.PoseArray` message type.  
    All data should be saved to a `.csv` file when the user enters `exit`.  
    You will need to save the pose name, position, and orientation.  
    The format of your `.csv` file should be something like:  
    ```
    name,px,py,pz,ox,oy,oz,ow
    ```  
    Consider using the pandas python package to manage your poses.  
    If poses exist in the csv file, they should be re-loaded into the map annotator system and available for the `list`, `delete`, and `goto` commands.
    
    Example usage:
    
    ```
    ros2 run lab6 map_annotator
    
    commands:
    - list
    - save <name>
    - delete <delete>
    - goto <name>
    - exit
    - help

    > save room1
    > list
    Poses:
    - room1
    > goto room1
    > delete room2
    'room2' does not exist
    > delete room1
    > list
    No poses
    ```

3. Write another python ros node in the same package called `ui_markers.py`.  
This node should subscribe to the PoseArray message published by the map annotator.  
It should then publish Rviz2 markers to display the poses like a user interface.  
You should use the `Marker` and/or `MarkerArray` message types to create Rviz2 markers.  
You can use [any markers](https://wiki.ros.org/rviz/DisplayTypes/Marker) that you'd like; they should accurately depict position and orientation.  

4. Open Gazebo in the maze world with a turtlebot4 lite using the map you made in the previous lab.  
Then run both of your new nodes in separate terminals to use your map annotator system.  

## Deliverables

1. Submit all of your code.

2. Submit a video of your system demonstrating all of the commands in simulation, include the terminal, gazebo, and rviz views.  
Start the system with a pose saved in the `.csv` file to demonstrate the system loading old poses.  

3. Submit a copy of your `.csv` database with poses saved from your demo.  
Copy/paste the contents into your notebook report.


## FAQ

**Q:** I don't see my markers in Rviz but everything seems to be working, why?  
**A:** Make sure you add a marker display to Rviz, then make sure it is set to the correct topic that you are publishing markers to.

**Q:** I added the marker topic to Rviz but still nothing shows, why?  
*A:** Make sure you create markers in the map frame. 

**Q:** My `map_annotator` node can't find my csv file, why?
**A:** After building your workspace, the code is run from the `install` directory, so relative file paths will be different.  
Try using absolute file paths instead.  

**Q:** The robot won't move to the goal even though the `.csv` file and markers look correct, why?  
**A:** Make sure all the information in the header of the `PoseStamped` message is correct like the frame and timestamp.

## Resources

