# Connecting to Turtlebots at GIX

For all robots at GIX, make sure you are connected to the `robolab_5` wifi network.  
The password for the network is written on the router in the robotics lab area.  

In order to edit text files like `~/.bashrc` on the robots and in your devcontainer, you will need to use a terminal-based text editor.  
Popular choices are [nano](https://www.howtogeek.com/42980/the-beginners-guide-to-nano-the-linux-command-line-text-editor/), [vim](https://opensource.com/article/19/3/getting-started-vim), and [emacs](https://www.gnu.org/software/emacs/tour/); engineers will judge your choice.  


## Turtlebot 3

For the Turtlebot 3s, you have to connect to its Raspberry Pi to start ros.

1. Connect to the `robolab_5` wifi; the password is written on the router in the robotics lab area.  

2. Plug in the robot's battery and turn it on, wait until the music stops.  

3. Connect to the Raspberry Pi through ssh:
    ```bash
    ssh ubuntu@<ip address>
    ```

    The IP address should be written on top of the Turtlebot 3.  
    If you just turned on the robot and you get an error about connecting, try again.  
    It may take some time after turning on before it accepts connections.  

4. Open the `~/.bashrc` file on both the Pi and your computer to check if the `ROS_DOMAIN_ID` is the same:
    ```bash
    nano ~/.bashrc
    ```

5. Start ros on the turtlebot using the bringup launch file on the Raspberry Pi:
    ```bash
    ros2 launch turtlebot3_bringup robot.launch.py
    ```

6. If you are able to connect with `ssh` and the `ROS_DOMAIN_ID`s are the same, you should be able to see topics using:
    ```bash
    ros2 topic list
    ```


## Turtlebot 4

The Turtlebot 4s are always running ros, so you don't need to `ssh` into them to start anything.

1. Turn on the robot by connecting it to the dock.  

2. Wait until the top light stops being red and the music plays, this might take a few minutes.  

3. Connect your computer to the `robolab_5` wifi; the password is written on the router in the robotics lab area.  

4. Edit your computer's `ROS_DOMAIN_ID` in your `.bashrc` to match the ID written on top of the Turtlebot 4 you are trying to use:
    ```bash
    nano ~/.bashrc
    ```

5. You should now see ros topics using:
    ```bash
    ros2 topic list
    ```

6. If you need to connect to the Turtlebot 4's Raspberry Pi, it broadcast its IP address on the `/ip` topic.  
    You shouldn't need to connect into the robot for most normal operation.  