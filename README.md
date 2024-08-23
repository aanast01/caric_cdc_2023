# CARIC 2023 Competition - KIOS Team Solution

## About
This repository holds the winning solution of CARIC 2023 Competition which took place during the 62nd IEEE Conference on Desicion and Control (CDC) in Singapor.
The solution is in form of a ROS package. To run the solution please first visit https://github.com/aanast01/caric_mission and follow the instructions to install and setup the competition's environmet.

## Brief Description
This work introduces a cooperative inspection system designed to efficiently control and coordinate a team of distributed heterogeneous UAV agents for the inspection of 3D structures in cluttered, unknown spaces.
Our proposed approach employs a two-stage innovative methodology. Initially, it leverages the complementary sensing capabilities of the robots to cooperatively map the unknown environment.
It then generates optimized, collision-free inspection paths, thereby ensuring comprehensive coverage of the structure’s surface area.
We consider a system of multiple heterogeneous quadrotor UAVs each equipped with different sensors, such as gimballed cameras and LiDAR, each capable of inspecting the surface area of the 
infrastructure of interest using only limited knowledge about the environment, under communication and time constraints.

## Usage
You can run the solution after compiling this package with the following command
```shell
roslaunch kios_solution run_solution
```

### Citation
This work has been accepted for publication at the 2024 International Conference on Unmanned Aircraft Systems (ICUAS) to be held on June 4-7 in Chania, Crete, Greece.
As soon as the proceeding of the conference are available, a citation tamplate will be given here.

#### Note
This work was motivated by the Cooperative Aerial Robots Inspection Challenge (CARIC) competition, held during the 2023 IEEE Conference on Decision and Control (CDC),
whose purpose was to simulate and benchmark multi-UAV infrastructure inspection methodologies in real-world-like inspection scenarios.

## Authors
[Andreas Anastasiou](https://github.com/aanast01) <br>
[Angelos Zacharia](https://github.com/angeloszacharia)
