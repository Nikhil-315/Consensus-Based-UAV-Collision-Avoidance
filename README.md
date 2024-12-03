# Consensus-Based-UAV-Collision-Avoidance
This repository implements a consesus based collision avoidance method for two UAVs using Ardupilot and MAVROS flight control stack. The method has been implemented using Python and ROS1 Noetic.

When an adversary drone (drone2) is within alert distance, the user drone (drone1) listens for position and velocity data of drone2 (facilitated by ROS topics in this case). drone1 also receives the cooperative status of drone2, i.e. how much is drone2 willing to take corrective actions. If drone2 is not cooperative, drone1 will take entirety of the corrective action required. If drone2 is cooperative, drone1 takes a part of the corrective action required while assuming the other part is taken by drone2. 

The corrective action required is determined by a simple method which uses relative distance and velocity of the drones and cooperative status of each drone.


## Contents

1. [Prerequisites](#prerequisites)
2. [Theory](#theory)
3. [Working](#working)

## Prerequisites
1. Ardupilot flight control stack
2. ardupilot_gazebo plugin (provided by https://github.com/khancyr/ardupilot_gazebo.git)
3. ROS1 Noetic and Gazebo
4. MAVROS

    You may follow https://github.com/Intelligent-Quads/iq_tutorials.git upto 'Installing ROS and MAVROS', 'Drone Swarms with Ardupilot' and 'Drone Swarms with Ardupilot+MAVROS' for the above setup.

5. utm Library for Python
```
pip install utm
```

6. PyGeodesy Library for Python
```
pip install PyGeodesy
```

## Theory
