import os
import time

# TODO: fill in your selected velocity and time values
# Move forward for 1.5m using open-loop strategy.
timenow = time.time()
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[<vel_value>, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
print("Speed: 0.15, Start timer: 0.0")
# the first pub latches for 3 seconds before continuing with next line
while time.time() - timenow < <time_value>: 
    pass
print("Speed 0.0 Stop timer: ", time.time() - timenow)
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
