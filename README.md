<div id="top"></div>

<br>
<div align="center">
 <img src="https://github.com/gmeidk/DelyBot/blob/d6c620261c2f81553f9dc13830cff278a25ad252/MEDIA/robot.png?raw=true" alt="DelyBot" height="180" width="200"> 
</div>
<h3 align="center">DelyBot</h3>
<p align="center">Ready to serve you anytime, anywhere.</p>
<br>

<!-- ABOUT THE PROJECT -->
## About The Project

<br>
DelyBot is a robot that uses six wheels to drive itself along pedestrian paths but also on public roads to reach requested delivery addresses.
The robot is able to locate itself through knowledge of the global static map and is able to detect and avoid moving obstacles.
<br>
<br>
<div align="center">
<img src="https://github.com/gmeidk/DelyBot/blob/cb787818e426946b98b9b5e84221a44c5c941cd8/MEDIA/simulation.png?raw=true" alt="DelyBot simulation"> 
</div>
<br>
DelyBot is composed of:
<br><br>
 <ul>
  <li>an <i>upper structure</i> designed to transport packages, objects, food</li>
  <li><i>six motorized wheels</i> that allow it to move using differential control</li>
</ul> 
<br>
<p align="right">(<a href="#top">back to top</a>)</p>

### Built With

* [ROS Noetic](https://www.ros.org/)
* [ROS Navigation Stack](http://wiki.ros.org/navigation)
* [ROS robot_localization](https://github.com/cra-ros-pkg/robot_localization)
* [Gazebo](https://gazebosim.org/)
* [Python](https://www.python.org/)

<p align="right">(<a href="#top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

At first it is necessary to install external packages:

* navigation
  ```sh
  sudo apt install ros-noetic-navigation
  ```

* slam-gmapping
  ```sh
  sudo apt install ros-noetic-slam-gmapping
  ```

* map-server
  ```sh
  sudo apt install ros-noetic-map-server
  ```
  
* robot_localization
  ```sh
  sudo apt-get install ros-noetic-robot-localization
  ```

<br>

### Installation

1. Clone the repo inside the catkin workspace
   ```sh
   cd ~/catkin_ws/src
   git clone https://github.com/gmeidk/DelyBot.git
   ```
2. Build packages
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```
   
<p align="right">(<a href="#top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage
The possible  **_optional parameter_** values are listed in the table. <br>

| Optional Parameter | Values | Default |
| --- | :---: | :---: |
| `open_rviz` | true, false | true |
| `world` | empty, delybot_test_map, delybot_test, district_map, district | district |
| `dwa_local_planner` | true, false | true |

<br>

The **world** parameter can be used to load a specific world map (the *.world* file must be located in the *delybot_description/world/* folder). <br>
To test the different world files in Gazebo it is possible to run the following command:
  ```sh
  # Usage example to open district.world in Gazebo
  roslaunch gazebo_ros empty_world world_name:=/home/your_username/catkin_ws/src/DelyBot/delybot_description/world/district.world
  ```

<br>
<br>

### delybot_description

  ```sh
  roslaunch delybot_description display.launch
  ```
Open a pre-configured Rviz session to display the robot.
  ```sh
  roslaunch delybot_description gazebo.launch open_rviz:=true
  ```
Spawn the robot into the gazebo simulation environment.
If the **open_rviz** optional parameter is *true* a pre-configured Rviz session is also opened.

<br>

### delybot_control

  ```sh
  roslaunch delybot_control ddr_control.launch world:=district
  ```
Spawn the robot with a differential drive control and a teleoperation node in gazebo. <br>

<br>

### delybot_slam

  ```sh
  roslaunch delybot_slam delybot_slam.launch world:=district_map
  ```
This command can be useful to create a 2d map of a specific world using a gmapping alghoritm. <br>

<br>

### delybot_navigation

  ```sh
  roslaunch delybot_navigation delybot_navigation.launch world:=district dwa_local_planner:=true
  ```
This command is used for the robot navigation, it's possible to give through Rviz a desired goal pose. <br>
If the **dwa_local_planner** parameter is *true* the *DWA* local planner is used, else if it is *false* the *Trajectory Rollout* local planner is used. 

<br>


#### waypoint_spawner node

  ```sh
  rosrun delybot_navigation waypoint_spawner.py
  ```
Run waypoint_spawner node used to send a specific goal pose selected from a predefined list to the robot. <br>
The list is imported from the **waypoint.json** file inside the *delybot_navigation/scripts/* folder. <br>

  ```sh
  # Output example
  user@user:~$ rosrun delybot_navigation waypoint_spawner.py 

  GOAL LIST:
  0) Origin
  1) Thrift Shop
  2) Salon
  3) Home
  4) Post Office
  5) Police
  6) School
  7) Fast Food

  Insert goal index: 
  ```

<!-- ROADMAP -->
## Roadmap

- [x] 3D robot modeling
- [x] Add diffrential drive control
- [X] Add laser and imu sensors using Kalman filter (pkg: robot_localization) 
- [X] Add world 3D model with static and moving obstacle 
- [X] Add delybot slam using gmapping to create 2D world map
- [X] Add delybot navigation
- [X] Add waypoint spawner node
- [X] Update README


<p align="right">(<a href="#top">back to top</a>)</p>

## Gazebo Simulation
https://user-images.githubusercontent.com/81641638/145631074-0517d654-1828-495e-8481-1b4d265e365c.mp4

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Alessandro Quatela - [@qualex97](https://github.com/qualex97) - a.quatela1@studenti.poliba.it 

Giuseppe Roberto - [@gmeidk](https://github.com/gmeidk) - g.roberto1@studenti.poliba.it

Project Link: [https://github.com/gmeidk/DelyBot](https://github.com/gmeidk/DelyBot)

<p align="right">(<a href="#top">back to top</a>)</p>
