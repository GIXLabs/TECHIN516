# TECHIN 516 Winter 2025 Class Project

![project example](/assets/final_project_example.jpg)

Over the course of the quarter you will develop a novel robot to accomplish a specific application of your choosing.  
You will work in groups to customize a [Turtlebot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).  
You will add 1 new motor, 1 new sensor, and change the body to fit your application.  
Your team will meet weekly with the instructors to discuss the weekly deliverables and ask questions.  


## Learning Objectives

- Develop a full-stack robotics application
- Explore your specific interest in robotics


## TODO

- Join a team.
- Develop an idea.
- Add a motor to your Turtlebot3, follow the instructions in [this repo](https://github.com/GIXLabs/t516_OpenCR).
- Add a sensor to your Turtlebot3, follow an existing guide of that sensor working with a Raspberry Pi 4.
- Change the robot body and description. 
- Make it all work in ROS2 Humble.
- Develop a demo application.
- Document it all on Github. 

**Schedule**

|  Week | TODO | Deliverable |
| - | - | - |
| 1 | Team formation<br>Concept ideation | Add team to class spreadsheet |
| 2 | Have concept approved by teaching team | Concept sketches<br>Concept diagrams|
| 3 | Research existing examples | Sensor <> Raspberry Pi documented example<br>Sensor selection from lab or purchasing |
| 4 | Actuator integration | Actuator progress |
| 5 | Actuator integration | Working actuator |
| 6 | Sensor integration | Sensor progress |
| 7 | Sensor integration | Working sensor |
| 8 | Sketching / measuring / test printing | New body iteration 1|
| 9 | Body refinement<br>Robot description (URDF) | New body iteration 2 |
| 10 | Assemble components into an application | Launch file of functioning application |
| 11 | | Final deliverables<br>Final presentation<br>[ Optional ] extra credit |


## Robot Design Requirements

- 1 (new) motor used in your application
- 2 (existing) motors to drive the wheels
- 1 (new) sensor used in your application
- LiDAR with enough field of view for mapping
- You must be able to demo your application with a single command
- You must be able to teleoperate drive the robot from your computer
- The body design must match its application
- The desing and function must be reflected in Rviz


## Final Deliverables

- A short video demo of your robot performing its application
- A Github repository containing the following:
    - All of your customized code required to use the robot
    - Contributions from all team members
    - A LICENSE file that complies with all dependencies' licenses
    - A README.md file with:
        - A short explanation of your project
        - A link to your video demo
        - Setup instructions, including installation instructions of any dependencies
        - Usage instructions
- A short presentation in front of the class containing:
    - A brief explanation of your design, application, and motivation
    - A working demo of the robot performing its purpose


## Extra Credit

**[ +5% ]** Modify the OpenCR firmware and ros2_control code to control the motor's velocity, not position.  
Check out the [gix_motor git commit]() to see how the current 3rd motor works to start.  

**[ +10% ]** Expand on your code to accurately simulate your design in a Gazebo simulation.  
Checkout the original [Turtlebot 3 Manipulation repo](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation) and the [Turtlebot 3 Manipulation Simulations repo](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations) for a good place to start.


## FAQ

**Q:** Help my motor isn't working!  
**A:** Follow DYNAMIXEL's [firmware recorvery guide](https://youtu.be/FAnVIE_23AA?si=Dddv99FITJ24RPSZ) using the OpenCR motor driver built into the Turtlebot3.  
Use the "XL430-W250" model since they don't list the "XL430-W250-T".  

**Q:** I designed my robot with different wheels and/or different spacing and now it doesn't drive correctly.  
**A:** You need to change [the parameters in the OpenCR firmware that you flash to the motor driver board](https://github.com/GIXLabs/T516_OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp#L26) to match your design.  

**Q:** I want to design a robot with a velocity controlled joint, how do I do that?  
**A:** The OpenCR firmware in this repo was modified to handle a 3rd position controlled motor.  
To understand what was changed run a diff on the custom [turtlebot3.cpp](https://github.com/GIXLabs/t516_OpenCR/blob/master/src/turtlebot3/turtlebot3.cpp) and the [file it was based on](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp).  
Also check out the new [gix_motor_driver.cpp file](https://github.com/GIXLabs/t516_OpenCR/blob/master/src/turtlebot3/gix_motor_driver.cpp) to see how the 3rd motor is controlled, and the original [turtlebot3_motor_driver.cpp file](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3_motor_driver.cpp) to see how velocity motors are controlled. 


## Resources

- [Example project](https://github.com/GIXLabs/t516_project_example)  
This repo shows a basic example of this project.  
This example adds a camera to identify dogs, and adds a motor to dispense dog treats.  
As a team, you will be expected to take the project further than provided.  
Forking this repo provides a good start for the code to control your new motor.  

- [Random Nerd Tutorials Raspberry Pi Guides](https://randomnerdtutorials.com/projects-raspberry-pi/)  
You should choose a sensor that is well documented to work with Raspberry Pis.  
Random Nerd Tutorials has many tutorials on how different sensors work.

- [Fusion to URDF](https://github.com/syuntoku14/fusion2urdf)  
This Fusion360 extension allows you to export robot descriptions from your model.  
There are many great guides online showing how to use the extension like [this one from Ammr-1](https://www.youtube.com/watch?v=Pokyu91hb_o).  
If you get an error about python 3.9, try using [this pull-request](https://github.com/syuntoku14/fusion2urdf/pull/89) from the same repo.

- [ROBOTIS Example Turtlebot 3 Mods](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-monster)  
The Turtlebot 3 is meant to be a highly customizable learning platform.  
ROBOTIS (the company behind Turtlebot 3) has demonstrated many potential modifications in the link above.

- [Dusty Robotics](https://www.dustyrobotics.com/)  
Dusty is a great example of a commercially operating differential drive robot developed by [Dr Tessa Lau](https://www.linkedin.com/in/tessalau) from UW.  
It uses an external survey laser to find its location, and a printer to draw construction layouts to increase speed and accuracy in building.  
