# TECHIN 516 Winter 2025 Class Project

Over the course of the quarter you will develop a novel robot to accomplish a specific application.  

## Learning Objectives

- Develop a full-stack robotics application
- Explore your specific interest in robotics


## TODO

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
**A:** You need to change [the parameters in the OpenCR firmware that you flash to the device](https://github.com/GIXLabs/T516_OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp#L26) to match your design.  

**Q:** I want to design a robot with a velocity controlled joint, how do I do that?  
**A:** 


## Resources

- [ROBOTIS Example Turtlebot 3 Mods](https://emanual.robotis.com/docs/en/platform/turtlebot3/locomotion/#turtlebot3-friends-monster)  
The Turtlebot 3 is meant to be a highly customizable learning platform.  
ROBOTIS (the company behind Turtlebot 3) has demonstrated many potential modifications in the link above.