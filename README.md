# Consensus-Based-UAV-Collision-Avoidance
This repository implements a simple consesus based collision avoidance method for two UAVs using Ardupilot and MAVROS flight control stack. The method has been implemented using Python and ROS1 Noetic.

When an adversary drone (drone2) is within the alert distance, the user drone (drone1) listens for the position and velocity data of drone2 (facilitated by ROS topics in this case). drone1 also receives the cooperative status of drone2, i.e. how much is drone2 willing to take corrective actions. If drone2 is not cooperative, drone1 will take entirety of the corrective action required. If drone2 is cooperative, drone1 takes a part of the corrective action required while assuming the other part is taken by drone2. 

The corrective action required is determined by a simple method which uses relative distance and relative velocity of the drones and cooperative status of each drone. For simplicity, corrective action in horizontal plane only is considered.
<br />

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
<br />

6. PyGeodesy Library for Python
```
pip install PyGeodesy
```
<br />

## Theory
This section contains 3 parts:
1. [Collision Detection](#collision-detection) where the possible violation of minimum safe distance by drone2 is detected using relative distance and relative velocity of drone1 wrt drone2.
   
2. [Corrective Action Calculation](#corrective-action-calculation) where the minimum required corrective velocity magnitude for the current velocity is determined for drone1 such that atleast the minimum safe distance is achieved. This is done using minimum safe distance value, relative distance, relative velocity and cooperative status of each drone. 
   
3. [Collision Avoidance](#collision-avoidance) where the corrective velocity is applied to drone1.
<br />   

### Collision Detection
![Relative vectors](https://github.com/user-attachments/assets/ed5862d2-4c99-4cae-8fd7-bd4fceb7bb13)
<br />

From the dot product of relative velocity vector and relative distance vector of drone1 wrt drone2, the angle between the two vectors is obtained. By trigonometric calculations using this angle and the magnitude of relative distance, the minimum seperation distance between drone1 and drone2 (dist) and the scaling factor for relative velocity vector (ttc) is obtained.<br />  

```math
cos(\theta) = \frac{\left( relative\_velocity\_vect \cdot relative\_distance\_vect \right)}{|relative\_velocity\_vect| |relative\_distance\_vect|}
```
<br />

```math
dist = tan(\theta) * |relative\_distance\_vect|
```
<br />

```math
ttc = \frac{|relative\_distance\_vect|}{cos(\theta) |relative\_velocity\_vect|}
```
<br />

If dist is less than the specified minimum safe distance, it would indicate a collision between drone1 and drone2 after time ttc.
<br />
<br />

### Corrective Action Calculation
![Desired Velocity](https://github.com/user-attachments/assets/bea884ac-e22b-4bb1-93bb-7d31ff759e24)
<br />

A desired relative velocity will be one where atleast the minimum safe distance is not violated. The magnitude of this desired relative velocity (vel_des) can be calculated by scaling by ttc and Pythagoras theorem or other trigonometric calculations.  
<br />

```math
vel\_des = \frac{\sqrt{ min\_safe\_dist^2 + |relative\_distance\_vect|^2 }}{ttc}
```
<br />

To achieve this desired relative velocity, a velocity correction would have to be given to the current velocity of drone1, which would be perpendicular to the relative velocity. For simplicity, perpendicular velocity correction only in the horizontal plane is considered. 
<br />

![Velocity Correction](https://github.com/user-attachments/assets/501ddae9-d959-43a1-8ba1-f59fc8488227)
<br />

The perpendicular velocity correction required (vel_correct_req) can be obtained by Pythagoras theorem.
<br />

```math
vel\_correct\_req = \sqrt{ vel\_des^2 - |relative\_velocity\_vect|^2 }
```
<br />

If drone2 is not cooperative (None), this corrective action will have to be taken by drone1 in entirety.<br /> 
But if drone2 is cooperative and has cooperative status D_2, while cooperative status of drone1 being D_1, the perpendicular velocity correction for drone1 can be set to be:
<br />

```math
vel\_correct\_req = vel\_correct\_req * \frac{D_2}{D_1 + D_2} 
```
<br />

Therefore, a higher cooperative status (for ex. 3) would be given a higher airspace priority and thus lesser corrective action will be assigned to it, while a lower cooperative status (for ex. 1) would be given a lesser airspace priority and thus more corrective action will be assigned to it.
<br />
<br />

### Collision Avoidance
The calculated magnitude of perpendicular velocity correction must be multiplied to a unit vector perpendicular to the relative velocity in the horizontal plane. For a given vector $(V_x, V_y, V_z)$, there can be two perpendicular unit vectors in the horizontal plane. 
<br />

```math
V_{p_1} = \left( \frac{-V_y}{\sqrt{V_x^2 + V_y^2}}, \frac{V_x}{\sqrt{V_x^2 + V_y^2}}, 0 \right) = V
```
<br />

```math
V_{p_2} = \left( \frac{V_y}{\sqrt{V_x^2 + V_y^2}}, \frac{-V_x}{\sqrt{V_x^2 + V_y^2}}, 0 \right) = -V
```
<br />

It can be seen that the two vectors are along the same line but in opposite directions.<br />
Two possible perpendicular velocity corrections are: 
<br />

```math
V_{correct_1} = vel\_correct\_req * \left( V \right)
```
<br />

```math
V_{correct_2} = vel\_correct\_req * \left( -V \right)
```
<br />

Now that two possible perpendicular velocity corrections have been determined, each can be applied to the current velocity of drone1, and the minimum seperation distance between drone1 and drone2 (dist) can again be calculated. The velocity correction for which dist satisfies the specified minimum safe distance can be finally applied to the current velocity of drone1 for collision avoidance. 
<br />

![Required Velocity](https://github.com/user-attachments/assets/329a02d5-538d-4c1e-a3dd-4e1ebbc43675)
<br />

## Working

Check out our YouTube playlist:

[YouTube Playlist](https://youtube.com/playlist?list=PLA8cbAjoUfdG9TNo93Xvplb8GFfCCfi2T&feature=shared)


![InShot_20241204_144305861](https://github.com/user-attachments/assets/b9f09a94-1a77-4827-87f7-eff7e746e1d9)

Head On Collision Case<br />
<br />
<br />

![InShot_20241204_163637047](https://github.com/user-attachments/assets/f928aba9-ee2e-4591-be65-afb5e4977b21) 

Collision At An Angle<br />
<br />
<br />

![Modes](https://github.com/user-attachments/assets/c0eabac6-c6c9-42e6-bfe7-b9a3e8bdb612)

Working Modes <br />
<br />
<br />

Having set up the workspace, you may replace the csv file path given at 'cas_ws/src/collision_avoidance/scripts/waypoint_follow_drone1.py' and 'cas_ws/src/collision_avoidance/scripts/waypoint_follow_drone2.py' line 458 with the path for csv file containing waypoints as per your mission. 

The csv file given at 'cas_ws/src/collision_avoidance/scripts/waypoints.csv' is an example of waypoints that can be given to drone1 and drone2. Please note that the columns alt1 and alt2 refer to relative altitude wrt initial takeoff altitude. lat1, lon1, alt1 are waypoints given to drone1 while lat2, lon2, alt2 are waypoints given to drone2. 

For non-cooperative drone, you may set the cooperative variable as 'None'. Else, you may set it as any positive integer such as 1, 2, 3, etc. 
Once the Gazebo environment has been spawned and the Ardupilot controller for each drone has been initialized, run the command:

```
roslaunch collision_avoidance 2_apm.launch
```
<br />

After the apm instances have been launched successfully, run the command:

```
roslaunch collision_avoidance collision_control.launch
```
<br />

In case the drones do not takeoff at all, abort the collision_control.launch task and run the following command before trying again:

```
rosclean purge
```
<br />

To record the trajectories of drone1 and drone2 in utm coordinates, run the command:

```
rosrun collision_avoidance gcs.py
```
<br />

After the recording the trajectories of the drones, to plot the trajectories you may run the command:

```
rosrun collision_avoidance plot.py
```
<br />

![head_on_graph](https://github.com/user-attachments/assets/15400eeb-cd0f-4873-ab2c-d8ab24b31927)
