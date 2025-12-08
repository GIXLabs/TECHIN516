# Lab 1: Intro to Linux and ROS

![welcome_to_techin516](/assets/graphic_design_is_my_passion.png)

Most robotics technologies are developed for Linux.  
All labs for the GIX robotics studios are to be completed in the Ubuntu 22.04 distribution of Linux.  
It is highly recommended to dual-boot your machine to run Ubuntu for full compatability with robotic technologies.  
M-series Macs are not yet able to dual-boot Ubuntu.  
Linux will get easier as you use it; this lab will demonstrate the basics of the "[BASH](https://en.wikipedia.org/wiki/Bash_(Unix_shell))" terminal and ROS2.  
Thoroughly understanding the following content will make everything else in the robotics studios and beyond much easier. 


## Learning Objectives

- Functional usage of Linux and the BASH terminal: how to navigate, run programs, manipulate data, etc.
- Overview ROS2 Humble: how nodes communicate, how to add new nodes, how do inspect what is happening in a robotic system.
- Compile resources of where to look for help, links to documentation, and what terminology to use.  
- Manage ROS environments and dependencies.  


## TODO

1. Dual-boot [Ubuntu 24.04 LTS](https://ubuntu.com/download/desktop) on your computer.

2. Build and enter the devcontainer.  
Refer to the [Docker guide](/docs/vscode_devcontainers.md) in the docs.  
[Install Docker](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04); make sure you can run `docker` commands without `sudo`.  
Install the [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extension for VS Code or Cursor.  
In a new VS Code or Cursor window, open the `turtlebot_ws` directory in this repo.  
Reopen the directory in the container.  
Build the ROS workspace inside the container.

3. Complete all [ROS 2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html) through "Using `ros2doctor` to identify issues".  
You do not need to complete any tutorials using C++, only the python tutorials.

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
    | `\|` (e.g. `cat ./file1.txt \| grep search`) | | | 
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
**A:** Virtual machines work for some basic features of Ubuntu and ROS2.  
They have issues utilizing GPUs for heavy computations and simulations, and have issues connecting to real robots.

**Q:** Can I use WSL2 insteaf of dual-booting?  
**A:** There are mixed reviews online about how well ROS2 works on WSL.  
You can try, but WSL unique issues will not be supported in this class. 

**Q:** Something went wrong inside the container and now I can't use ROS, what do I do?  
**A:** If your container isn't working, you can always rebuilt by opening the `turtlebot_ws` folder outside of the container and selecting:  
"Dev Container: Rebuild and Reopen in Container" from VS Code's command pallet (accessed with Control-Shift-P).  
Your ROS workspace will be saved because that folder is shared between the host machine and the container.  
**All other files will be deleted!**  
Make sure you backup anything you'll need later by moving the files to the shared workspace folder.  

**Q:** When I build the container I get an error about GPUs, what do I do?  
**A:** If your computer doesn't have a GPU, remove the line `"--gpus", "all",` from the `decontainer.json` file.

**Q:** When I build the container, it seems to load forever, the last line says "Container started", how do I make it work?  
**A:** Press the bottom left corner of VS Code and select "Close Remote Conncetion".  
Then under the "Recent" choices, select "turtlebot4_ws [Dev Container]".  


## Resources

[MIT Missing Semester IAP 2020](https://www.youtube.com/playlist?list=PLyzOVJj3bHQuloKGG59rS43e29ro7I57J)  
Many good lectures covering many applicable computer-science topics, including Linux.  
Highly recommend you watch [lecture 1](https://www.youtube.com/watch?v=Z56Jmr9Z34Q&t=771s) if you have never used Linux before.  

[ROS2 Humble YouTube tutorials by Kevin Wood](https://www.youtube.com/playlist?list=PLSK7NtBWwmpTS_YVfjeN3ZzIxItI1P_Sr)  
Many good short tutorials, good for getting more information on topics covered in the official ROS tutorials.

[ROS2 Humble YouTube tutorials by Robotics Back-End](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)  
Longer tutorials that roughly follow the official tutorials.
 
[Linux File System/Structure Explained by DorianDotSlash](https://www.youtube.com/watch?v=HbgzrKJvDRw)  
This video goes through all linux directories explaining what each one is for.
