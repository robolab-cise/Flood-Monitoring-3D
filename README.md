# Flood-Monitoring-3D
- This is a repository for a Robot Operating System (ROS) project.
- This project is for a Voronoi-based flood area detection using multi-agent systems.

## About
This simulation is similar to the [Flood-Monitoring](/../../../Flood-Monitoring) project. The difference is that it utilizes a 3D world instead of a flat 2D map.

## How to Download the Project Folder
The _flood_monitor_ folder we have uploaded to this repository is a folder located inside the _catkin_ws_ folder of the ROS environment. To download the whole folder, find the green **Code** button near the top of the page and click on it. A pop-up menu will appear showing an option to **Download ZIP**. A _flood_monitor.zip_ file will be downloaded to your computer. The zip file will contain all the files and folders listed in this repository under the _flood_monitor_ folder. 

## How to Install and Run the Simulation
- We assume you have a copy of ROS installed in your machine. Once you have downloaded the project folder, copy it to the folder _catkin_ws/src/_. Open a terminal on your PC and type **_roscore_** and then press Enter, this will start roscore. 
- Next, open another terminal window and type **_roslaunch flood_monitor new_drone_unity.launch_** and then press Enter, this will launch the project's world in Gazebo. You will see Gazebo loading the world into the simulator. Please, add a light source into the world, or else the drones will fail to see the water. Wait for Gazebo to complete loading the world. When the loading is complete, you will see this message displayed in the terminal where you used to launch Gazebo: **[ WARN] [1636000211.600138404, 28.075000000]: No command received for 28.075s, triggering estop** 
- Finally, open another terminal window, type **_cd catkin_ws/src/flood_monitor/src/_** and then press Enter. Make sure you are in that folder. Then type, **_./test_voronoi.sh_** and then press Enter, to start the simulation. 
- It will take a few seconds to initialize the drones. When the drones are activated you will see small windows pop-up showing the point of view from the drones' camera. When all the drones are in position, the Voronoi function will be executed and another window will pop-up showing the Voronoi and the density diagram. As all of these is taking place, you can see the drones in action in the Gazebo simulator. 
- To stop the simulation, press **_CTRL-C_** on the keyboard for each terminal window except for the Gazebo and roscore terminals. After all the terminals have been terminated, you can then stop Gazebo either from the Gazebo menu or from the Gazebo terminal by pressing **_CTRL-C_**. Then you can shutdown roscore by pressing **_CTRL-C_** on the roscore terminal.

### Note
- README.md files are Github readme files that are required by Github. You can ignore this files when you copy the project folder to the _catkin_ws_ folder.
