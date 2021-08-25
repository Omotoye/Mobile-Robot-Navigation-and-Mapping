<a href="https://unige.it/en/">
<img src="images/unige.png" width="20%" height="20%" title="University of Genoa" alt="University of Genoa" >

</a>

# Software Architecture Project

## Project Members

|            Name            |       Email Address       |
| :------------------------: | :-----------------------: |
| Omotoye Shamsudeen Adekoya | adekoyaomotoye@gmail.com  |
|       Farzin Sarvari       | Sarvarifarzin94@gmail.com |
|      Amanzhol Raisov       |   cornytravel@gmail.com   |
|     Taha Hussain Raja      |    rajagenoa@gmail.com    |

<div align="center">
<h1>  Mobile Robot Navigation and Mapping </h1>
<img src="images/husky.jpg" width="50%" height="50%" title="Husky Robot equipped with Lidar" alt="Husky Robot equipped with Lidar" >

</div>

## Project Objectives

The aim of this project is to design and implement a **_software architecture_** for the control of a mobile robot _(Husky Robot)_ to Navigate and Map an environment _(simulated or real-life)_. The Mobile Robot in question is a robot called **Husky Robot**, the robot is a _non-holonomic robot_ equipped with a LaserScan sensor for detecting obstacle. A simulation environment is provided as a **Unity Scene** which is contained by the _husky robot_ and an environment with obstacles to navigate through, there is also a goal point which the robot is intended to navigate to. The control of the mobile robot to perform the aforementioned task is going to be done through ROS packages, therefore the ROS packages and how they interact through interfaces to perform the goal is what this project is going to focus on (basically investing the software architecture design for the ROS packages).

Enumerated goals the software architecture is intended to achieve with the robot (simulated and real-life)

- Create a 3D Map (SLAM)
- Implement an Exploration Logic to reach a target in the environment.

For more information about the project requirements, [Click Here](Docs/SofAR-Assignments-2020-2021.pdf).

**NOTE**: **_some objectives has been changed because of simulation limitations_**
