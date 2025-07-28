# Lab 1: Intro to Linux and ROS

Most robotics technologies are developed for Linux.  
All labs for the GIX robotics studios are to be completed in the Ubuntu 22.04 distribution of Linux.  
It is highly recommended to dual-boot your machine to run Ubuntu for full compatability with robotic technologies.  
M-series Macs are not yet able to dual-boot Ubuntu.  
Linux will get easier as you use it; this lab will demonstrate the basics of the "[BASH](https://en.wikipedia.org/wiki/Bash_(Unix_shell))" terminal and ROS2.  
Thoroughly understanding the following content will make everything else in the robotics studios and beyond much easier. 


## Learning Objectives

- Functional understanding of Linux and the BASH terminal: how to navigate, run programs, manipulate data, etc.
- Beginner understanding of ROS2 Humble: how nodes communicate, how to add new nodes, how do inspect what is happening in a robotic system.


## TODO

1. Dual-boot [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) on your computer.

2. Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

3. Complete all [ROS 2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html) through "Creating custom msg and srv files".  
You do not need to complete any tutorials using C++.

4. Read the remaining tutorials through the end of the intermediate section, "RViz".


## Deliverables

1. Submit your code for the tutorial "Writing a simple publisher and subscriber (Python)".

2. Submit a link to the bag file you recorded in the tutorial "Recording and playing back data".

3. Fill in the description and a (different) example for each of the bash commands in the following table.  
Copy and paste the table into your lab report to fill-in.  
**Hint:** for most commands and programs, you can append `--help` or `-h` to learn more about it; e.g. `ls --help`.

    | Command | Description | Example |
    | - | - | - | 
    | `ls` | | | 
    | `ls -la` | | |
    | `cd` | | | 
    | `mv` | | | 
    | `rm` | | | 
    | `rm -rf` | | |
    | `cp` | | | 
    | `cp -r` | | |
    | `cat` | | | 
    | `touch` | | | 
    | `source` | | | 
    | `export` | | |
    | `grep` | | |
    | `echo` | | |
    | `sudo` | | |
    | `chmod` | | |
    | `chown` | | |
    | `*` (e.g. `rm ./*`) | | | 
    | `\|` (e.g. `cat ./fil1.txt \| grep search`) | | | 
    | `.` (e.g. `python ./script.py`) | | | 
    | `..` (e.g. `cd ..`) | | | 
    | `~` (e.g. `source ~/.bashrc`)| | | 
    | `>` (e.g. `cat ./file1.txt > ./file2.txt`) | | | 
    | `>>` (e.g. `cat ./file1.txt >> ./file2.txt`) | | | 
    | `!!` (e.g. `sudo !!`)| | | 


## FAQ

**Q:** I have an M-series Mac, what should I do?  
**A:** Consider purchasing a (non-ARM) Winows and/or Linux computer if you would like to pursue a career in robotics.  
Alternatively, speak with the professor about borrowing a laptop from GIX IT for a short period of time.

**Q:** Can I use a virtual machine instead of dual-booting?  
**A:** Virtual machines work for some basic features of Ubuntu and ROS2, but they have issues utilizing GPUs for heavy computations and simulations, and have issues connecting to real robots.

**Q:** Can I use WSL2 insteaf of dual-booting?  
**A:** There are mixed reviews online about how well ROS2 works on WSL.  
You may try, but WSL unique issues will not be supported in this class. 


## Resources

[MIT Missing Semester IAP 2020](https://www.youtube.com/playlist?list=PLyzOVJj3bHQuloKGG59rS43e29ro7I57J)  
Many good lectures covering many applicable computer-science topics, including Linux.  
Highly recommend you watch [lecture 1](https://www.youtube.com/watch?v=Z56Jmr9Z34Q&t=771s) if you have never used Linux before.  

[ROS2 Humble YouTube tutorials by Kevin Wood](https://www.youtube.com/playlist?list=PLSK7NtBWwmpTS_YVfjeN3ZzIxItI1P_Sr)  
Many good short tutorials, good for getting more information on topics covered in the official ROS tutorials.

[ROS2 Humble YouTube tutorials by Robotics Back-End](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)  
Longer tutorials that roughly follow the official tutorials.