# MineRobot

This repository is the open-source demo for the paper *MineRobot: An Actuator-Centered Kinematic Modeling and Solving Framework for Underground Mining Robots*.

It is a lightweight implementation written in pure JavaScript and can be used interactively in a web browser.

- GitHub: <https://github.com/Housz/MineRobot>
- Online demo: <https://housz.github.io/MineRobot/>
  - **Shield-Type Hydraulic Support Robot:** [Online Demo](https://housz.github.io/MineRobot/demos/Shield-Type-Hydraulic-Support-Robot/index.html) | [MRDF File](https://github.com/Housz/MineRobot/blob/main/demos/Shield-Type-Hydraulic-Support-Robot/hydraulic_support.json)
  - **Top-Coal-Caving Hydraulic Support Robot:** [Online Demo](https://housz.github.io/MineRobot/demos/Top-Coal-Caving-Hydraulic-Support-Robot/index.html) | [MRDF File](https://github.com/Housz/MineRobot/blob/main/demos/Top-Coal-Caving-Hydraulic-Support-Robot/hydraulic_support.json)

<p align="center">
  <img src="./imgs/Shield-Type-Hydraulic-Support-Robot.gif" width="30%" alt="Shield-Type Hydraulic Support Robot">
  <img src="./imgs/Top-Coal-Caving-Hydraulic-Support-Robot.gif" width="30%" alt="Top-Coal-Caving Hydraulic Support Robot">
</p>

<p align="center">
  <img src="./imgs/rh-fk.gif" width="30%" alt="roadheader fk">
  <img src="./imgs/rh-ik.gif" width="30%" alt="roadheader ik">
  <img src="./imgs/rh-traj.gif" width="30%" alt="roadheader trajectory">
</p>

The demo supports:

- Interactive adjustment of actuator lengths to test forward kinematics
- Interactive specification of the end-effector configuration and solving to test inverse kinematics

Due to copyright restrictions, this demo only provides the key structures of two hydraulic support robots built with basic shapes.
This project is only a simplified implementation of the paper for demonstration purposes.


<p align="center">
  <img src="./imgs/graph_abstract.svg" alt="MineRobot Graphical Abstract" width="60%">
</p>