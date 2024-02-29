# Where to put the code
It doesn't matter, but you'll need to know where you've placed it so you can give the launch files the correct path.

# CSV
Please download a copy of the Excel file. Otherwise everyone will try to edit it at once.

# Running plot_trajectory.py
- Your .csv file shoudl be in the same directory as this script.
- Install pandas before running it
    - `pip install pandas`
- Include the headers (time,x,y,z) in the csv file
- When you copy the table from Excel to your own csv file, you'll need to replace all of the spacing with commas (no comma at the end of line though).

# Editing start_gazebo.launch
Make sure to add the line in the bottom section (inside the <include>), or the world will spawn without anything in it
