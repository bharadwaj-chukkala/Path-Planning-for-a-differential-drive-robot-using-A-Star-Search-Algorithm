# Path-Planning-for-a-differential-drive-robot-using-A-Star-Search-Algorithm

ENPM 661 Project 3 Phase 2
**Description**: Adding differential drive constraints to the path planning algorithm for a rigid robot which uses A Star Search to find the goal node in the Configuration Space with Obstacles

| Team Members                   |
| ------------------------------ |
| Bharadwaj Chukkala             |
| Joseph Pranadeer Reddy Katakam |

## Introduction

The goal of this project is to implement a path planning algorithm for a rigid robot which uses A Star Search to find the goal node in the Configuration Space with Obstacles. The robot is a differential drive robot. In this case, a turtlebot is used.

## Dependencies

* Python 3.6
* Matplotlib
* Numpy
* argparse
* heapq

## How to run the code

* Clone the repository

```bash
git clone https://github.com/bharadwaj-chukkala/Path-Planning-for-a-differential-drive-robot-using-A-Star-Search-Algorithm.git
```

* Open the terminal and navigate to the directory where the repository is cloned

```bash
cd Path-Planning-for-a-differential-drive-robot-using-A-Star-Search-Algorithm
```

* To Run the following command to run the code for only visualization of the path
    * Run the python script **Phase2.py** with the following command
        ```bash
        python3 Phase2.py
        ```
    * Enter the inputs as prompted in the terminal
    * The path will be visualized in a plot after a few seconds.



* To run the code for simulation of the robot in Gazebo through ROS:

  * Copy paste the **a_star_turtlebot** package into the src folder of your catkin workspace
  * Run the following commands in the terminal
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    roslaunch a_star_turtlebot proj.launch
    ```
  * Enter the inputs as prompted in the terminal
  * The robot will start moving towards the goal node
  * To exit Gazebo press ``Ctrl+C`` in the terminal

## Results

### Phase 2 Part 1

This is the visualization of the path found by the A Star Search algorithm for a rigid robot which uses A Star Search to find the goal node in the Configuration Space with Obstacles. Click the below image to watch the video.

[![Phase 2 Part 1](https://github.com/bharadwaj-chukkala/Path-Planning-for-a-differential-drive-robot-using-A-Star-Search-Algorithm/blob/main/outputs/P3P1.jpg)](https://drive.google.com/file/d/105u4m5jktIU5xoYhF-J1HIAQVPftdcAq/view?usp=share_link)

### Phase 2 Part 2

This is the simulation of the above robot in Gazebo through ROS. Click the below image to watch the video.

[![Phase 2 Part 2](https://github.com/bharadwaj-chukkala/Path-Planning-for-a-differential-drive-robot-using-A-Star-Search-Algorithm/blob/main/outputs/P3P2.jpg)](https://drive.google.com/file/d/1tFrjVtalV8_ed-6tbaOr8Phx-Ovm0vvK/view?usp=share_link)

## Contact

**Bharadwaj Chukkala**<br>
UID: 118341705<br>
Bharadwaj Chukkala is currently a Master's student in Robotics at the University of Maryland, College Park, MD (Batch of 2023). His interests include Machine Learning, Perception and Path Planning.<br>
[![Website](https://img.shields.io/badge/Website-000000?style=for-the-badge&logo=google-chrome&logoColor=white)](https://bharadwaj-chukkala.github.io/)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/bharadwaj-chukkala/)
[![GitHub](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/bharadwaj-chukkala)
