# Consensus-Based-UAV-Collision-Avoidance
This repository implements a simple consesus based collision avoidance method for two UAVs using Ardupilot and MAVROS flight control stack. The method has been implemented using Python and ROS1 Noetic.

When an adversary drone (drone2) is within the alert distance, the user drone (drone1) listens for the position and velocity data of drone2 (facilitated by ROS topics in this case). drone1 also receives the cooperative status of drone2, i.e. how much is drone2 willing to take corrective actions. If drone2 is not cooperative, drone1 will take entirety of the corrective action required. If drone2 is cooperative, drone1 takes a part of the corrective action required while assuming the other part is taken by drone2. 

The corrective action required is determined by a simple method which uses relative distance and relative velocity of the drones and cooperative status of each drone. For simplicity, corrective action in horizontal plane only is considered.


## Contents

1. [Prerequisites](#prerequisites)
2. [Theory](#theory)
3. [Working](#working)

## Prerequisites
1. Ardupilot flight control stack
2. ardupilot_gazebo plugin (provided by https://github.com/khancyr/ardupilot_gazebo.git)
3. ROS1 Noetic and Gazebo
4. MAVROS

    You may follow https://github.com/Intelligent-Quads/iq_tutorials.git from 'Installing Ardupilot and MAVProxy' upto 'Installing ROS and MAVROS', 'Drone Swarms with Ardupilot' and 'Drone Swarms with Ardupilot+MAVROS' for the above setup.

5. utm Library for Python
```
pip install utm
```

6. PyGeodesy Library for Python
```
pip install PyGeodesy
```

## Theory
This section contains 3 parts:
1. [Collision Detection](#collision-detection) where the possible violation of minimum safe distance by drone2 is detected using relative distance and relative velocity of drone1 wrt drone2.
   
2. [Corrective Action Calculation](#corrective-action-calculation) where the minimum required corrective velocity magnitude for the current velocity is determined for drone1 such that atleast the minimum safe distance is achieved. This is done using minimum safe distance value, relative distance, relative velocity and cooperative status of each drone. 
   
3. [Collision Avoidance](#collision-avoidance) where the corrective velocity is applied to drone1.
   

### Collision Detection
![Relative vectors](https://github.com/user-attachments/assets/ed5862d2-4c99-4cae-8fd7-bd4fceb7bb13)

From the dot product of relative velocity vector and relative distance vector of drone1 wrt drone2, the angle between the two vectors is obtained. By trigonometric calculations using this angle and the magnitude of relative distance, the minimum seperation distance between drone1 and drone2 (dist) and the scaling factor for relative velocity vector (ttc) is obtained.  

```math
cos(\theta) = \frac{\left( relative\_velocity\_vect \cdot relative\_distance\_vect \right)}{|relative\_velocity\_vect| |relative\_distance\_vect|}
```
```math
dist = tan(\theta) * |relative\_distance\_vect|
```

```math
ttc = \frac{|relative\_distance\_vect|}{cos(\theta) |relative\_velocity\_vect|}
```

If dist is less than the specified minimum safe distance, it would indicate a collision between drone1 and drone2 after time ttc.


### Corrective Action Calculation
![Desired Velocity](https://github.com/user-attachments/assets/bea884ac-e22b-4bb1-93bb-7d31ff759e24)

A desired relative velocity will be one where atleast the minimum safe distance is not violated. The magnitude of this desired relative velocity (vel_des) can be calculated by scaling by ttc and Pythagoras theorem or other trigonometric calculations.  

```math
vel\_des = \frac{\sqrt{ min\_safe\_dist^2 + |relative\_distance\_vect|^2 }}{ttc}
```

To achieve this desired relative velocity, a velocity correction would have to be given to the current velocity of drone1, which would be perpendicular to the relative velocity. For simplicity, perpendicular velocity correction only in the horizontal plane is considered. 

![Velocity Correction](https://github.com/user-attachments/assets/501ddae9-d959-43a1-8ba1-f59fc8488227)

The perpendicular velocity correction required (vel_correct_req) can be obtained by Pythagoras theorem.

```math
vel\_correct\_req = \sqrt{ vel\_des^2 - |relative\_velocity\_vect|^2 }
```

If drone2 is not cooperative, this corrective action will have to be taken by drone1 in entirety. 
But if drone2 is cooperative and has cooperative status D_2, while cooperative status of drone1 being D_1, the perpendicular velocity correction for drone1 can be set to be:

```math
vel\_correct\_req = vel\_correct\_req * \frac{D_2}{D_1 + D_2} 
```

Therefore, higher cooperative status (for ex. 3) would be given higher airspace priority and thus lesser corrective action will be assigned to it, while lower cooperative status (for ex. 1) would be given lesser airspace priority and thus more corrective action will be assigned to it.


### Collision Avoidance
The calculated magnitude of perpendicular velocity correction must be multiplied to a unit vector perpendicular to the relative velocity in the horizontal plane. For a given vector $(V_x, V_y, V_z)$, there can be two perpendicular unit vectors in the horizontal plane. 

```math
V_{p_1} = \left( \frac{-V_y}{\sqrt{V_x^2 + V_y^2}}, \frac{V_x}{\sqrt{V_x^2 + V_y^2}}, 0 \right) = V
```

```math
V_{p_2} = \left( \frac{V_y}{\sqrt{V_x^2 + V_y^2}}, \frac{-V_x}{\sqrt{V_x^2 + V_y^2}}, 0 \right) = -V
```
It can be seen that the two vectors are along the same line but in opposite directions. 
Now that two possible perpendicular velocity corrections have been determined, each can be applied to the current velocity of drone1, and the minimum seperation distance between drone1 and drone2 (dist) is again calculated. The velocity correction for which dist satisfies the specified minimum safe distance can be finally applied to the current velocity of drone1 for collision avoidance. 
![Required Velocity](https://github.com/user-attachments/assets/329a02d5-538d-4c1e-a3dd-4e1ebbc43675)
