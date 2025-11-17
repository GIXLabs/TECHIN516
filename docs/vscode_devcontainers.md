# VS Code Docker DevContainers

It is highly recommended to run ros in dedicated development containers.  
Robot systems typically have many dependencies that might interfere with other systems on your computer.  
For example, one robot might use numpy 1.x.x while another uses numpy 2.x.x.  
If you tried to install both systems in the same environment, the conflict would cause parts of your code to malfunction.  
Docker containers are used to isolate systems on the same computer and ensure everything runs smoothly.  


## Installation

1. Download Docker  
    Follow a tutorial for your operating system; [this](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04) is a good one for Ubuntu 22.  
    Make sure you follow through step 2 to ensure you can run docker commands without the need for `sudo`.

2. Download VS Code and Extensions.
    Next download [VS Code](https://code.visualstudio.com/download).  
    Once you have VS Code open, go to the extensions tab and download the "Remote Development" extension.  
    This extension includes the "Dev Container" extension that allows you to use containers inside of VS Code with the source code mounted from your regular drive. 
    This makes it much easier to manage your code and the environment.  
3. Clone the Class Repo.  
    Inside of VS Code, open the folder called `turtlebot4_ws`.
    VS Code should throw a pop-up that says "Reopen in container" - press this to access a ore-configured development environment with all the necessary dependencies to work with the Turtlebot 4s.


## Make Your Own Dev Container

You should set up your own environment for your projects.  
[Articulated Robotics](https://www.youtube.com/watch?v=dihfA7Ol6Mw) has good tutorials about devcontainers and Docker as a whole.  
Bellow are the basic steps to configure a new devcontainer: 

1. If you would ever like to configure your own workspace for your own projects, click on the bottom left corner icon in VS Code and select "Add dev container configuration files..."

2. Select the option to "Add configuration to workspace".

3. Search for "ros", configurations from "ijnek" typically work well.

4. Look through the [class examples](/turtlebot4_ws/.devcontainer/devcontainer.json) to see how to further edit the new configuration files in the `.devcontainer` folder that the extension adds.

5. Press \<Cntrl-Shift-P> to open VS Code's command menu, search for "rebuild and reopen in container" to build your new environment.  
    If you change the `Dockerfile` or `devcontainer.json` configuration files you will need to use this command to rebuild the environment for the changes to take affect. 
