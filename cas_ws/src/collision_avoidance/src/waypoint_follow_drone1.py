#!/usr/bin/env python3

import rospy
import csv
import time
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Header, Int32
from geographic_msgs.msg import GeoPoseStamped
import math
import utm
from pygeodesy.geoids import GeoidPGM

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)


#-----------------------------------------------------------------------------------------------------------------------------------#
class WaypointFollower:

    # Initialize parameters for class
    def __init__(self, agent_id, csv_file, cooperative):

        # Initialize ROS node
        rospy.init_node('waypoint_follower', anonymous=True)

        self.agent_id= agent_id
        self.agent_namespace = f'/drone{agent_id}/mavros/'

        self.current_state = State()
        self.initial_takeoff_altitude = 0
        self.current_position = None
        self.cooperative = cooperative
        
        self.drone2_position = None
        self.drone2_cooperative = None
        
        # Velocity variables
        self.drone1_velocity = TwistStamped()
        self.drone2_velocity = TwistStamped()

        # Set up subscribers and publishers using the namespace
        self.position_sub = rospy.Subscriber(self.agent_namespace + 'global_position/global', NavSatFix, self.position_callback)
        self.state_sub = rospy.Subscriber(self.agent_namespace + 'state', State, self.state_callback)
        self.drone2_position_sub = rospy.Subscriber('/drone2/mavros/global_position/global', NavSatFix, self.drone2_position_callback)
        self.drone2_cooperative_sub = rospy.Subscriber('/drone2/cooperative', Int32, self.drone2_cooperative_callback)
        self.drone1_velocity_sub = rospy.Subscriber(self.agent_namespace + 'local_position/velocity_local', TwistStamped, self.drone1_velocity_callback)
        self.drone2_velocity_sub = rospy.Subscriber('/drone2/mavros/local_position/velocity_local', TwistStamped, self.drone2_velocity_callback)

        self.setpoint_pub = rospy.Publisher(self.agent_namespace + 'setpoint_position/global', GeoPoseStamped, queue_size=10)
        self.cooperative_pub = rospy.Publisher(f'/drone{agent_id}/cooperative', Int32, queue_size=10)
        self.velocity_pub= rospy.Publisher(self.agent_namespace + 'setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

        self.rate = rospy.Rate(10)
        

        # Distance thresholds
        self.alert_dist = 40.0  
        self.min_safe_dist= 5.0

        # Alert parameters
        self.collision= False
        self.Alert = False
        self.Recommend = False
        self.min_dist= 100

        # Wait for FCU connection
        rospy.loginfo("Connecting Agent")
        while not self.current_state.connected:
            self.rate.sleep()

        # Arm the drone
        self.arm_drone()

        # Wait for the position to be available
        while self.current_position is None:
            rospy.loginfo("Waiting for initial position...")
            self.rate.sleep()

        # Set the initial takeoff altitude from the current position
        self.initial_takeoff_altitude = self.current_position.altitude

        # Takeoff
        self.takeoff(4.0) 

        # Note start time
        self.time= rospy.Time.now().secs

        # Load waypoints
        self.waypoints = self.load_waypoints(csv_file, agent_id)


    # Load waypoints from given csv file 
    def load_waypoints(self, csv_file, agent_id):

        waypoints = []
        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)

            # Skip the header row
            header = next(reader)
            for row in reader:

                try:

                    # Reads waypoints as columns: lat1, lon1, alt1, lat2, lon2, alt2, ...
                    # First 3 columns are waypoints for drone 1, next 3 for drone 2 and so on ...
                    # alt refers to relative altitude
                    index = (agent_id - 1) * 3
                    lat, lon, alt = map(float, row[index:index + 3])
                    waypoints.append((lat, lon, alt))

                except ValueError as e:
                    rospy.logerr(f"Error converting row {row}: {e}")

        return waypoints
    

    # Callback for drone 1 position subscriber
    def position_callback(self, msg):
        self.current_position = msg

        # Convert ellipsoid height to AMSL height
        self.current_position.altitude -= _egm96.height(self.current_position.latitude, self.current_position.longitude)


    # Callback for drone 2 position subscriber
    def drone2_position_callback(self, msg):
        self.drone2_position = msg

        # Convert ellipsoid height to AMSL height
        self.drone2_position.altitude -= _egm96.height(self.drone2_position.latitude, self.drone2_position.longitude)


    # Callback for drone 2 cooperative subscriber 
    def drone2_cooperative_callback(self, msg):
        self.drone2_cooperative = msg.data


    # Callback for drone 1 velocity subscriber
    def drone1_velocity_callback(self, msg):
        self.drone1_velocity = msg


    # Callback for drone 2 velocity subscriber
    def drone2_velocity_callback(self, msg):
        self.drone2_velocity = msg


    # Callback for drone 1 state subscriber
    def state_callback(self, msg):
        self.current_state = msg


    # Publisher function for drone 1 cooperative
    def send_cooperative(self, cooperative):
        value = Int32()
        value.data= cooperative
        self.cooperative_pub.publish(value)


    # Setpoint function for drone 1 velocity
    def publish_velocity(self, v_new):
        vel= Twist()
        vel.linear.x= v_new[0]
        vel.linear.y= v_new[1]
        vel.linear.z= v_new[2]
        vel.angular.x= 0
        vel.angular.y= 0
        vel.angular.z= 0
        self.velocity_pub.publish(vel)


    # Setpoint function for drone 1 position
    def publish_waypoint(self, waypoint):
        geo_pose = GeoPoseStamped()
        geo_pose.header = Header()
        geo_pose.header.stamp = rospy.Time.now()

        absolute_altitude = self.initial_takeoff_altitude + waypoint[2]

        geo_pose.pose.position.latitude = waypoint[0]
        geo_pose.pose.position.longitude = waypoint[1]
        geo_pose.pose.position.altitude = absolute_altitude

        geo_pose.pose.orientation.w = 0.7071
        geo_pose.pose.orientation.x = 0
        geo_pose.pose.orientation.y = 0
        geo_pose.pose.orientation.z = 0.7071

        self.setpoint_pub.publish(geo_pose)


    # Calculate distance to waypoint
    def distance_to_waypoint(self, wp):

        if self.current_position:
            lat1, lon1, alt1 = self.current_position.latitude, self.current_position.longitude, self.current_position.altitude
            lat2, lon2, alt2 = wp  # alt2 is relative height

            horizontal_distance = self.haversine(lat1, lon1, lat2, lon2)
            vertical_distance = abs(alt1 - (self.initial_takeoff_altitude + alt2))

            return math.sqrt(horizontal_distance**2 + vertical_distance**2)
        return None
    

    # Calculate distance to aircraft
    def distance_to_aircraft(self, wp):

        if self.current_position:
            lat1, lon1, alt1 = self.current_position.latitude, self.current_position.longitude, self.current_position.altitude
            lat2, lon2, alt2 = wp  # alt2 is AMSL height

            horizontal_distance = self.haversine(lat1, lon1, lat2, lon2)
            vertical_distance = abs(alt1 - alt2)

            return math.sqrt(horizontal_distance**2 + vertical_distance**2)
        return None


    # Calculate distance between two GPS coordinates
    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):

        R = 6371000  # Radius of Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))

        return R * c


    # Function to arm the drone
    def arm_drone(self):

        rospy.loginfo("Arming Vehicle...")
        rospy.wait_for_service(self.agent_namespace + 'cmd/arming')

        try:
            arm = rospy.ServiceProxy(self.agent_namespace + 'cmd/arming', CommandBool)
            arm(True)
            rospy.loginfo("Vehicle Armed.")

        except rospy.ServiceException as e:
            rospy.logerr(f"Arming failed: {e}")

        # Set mode to GUIDED
        rospy.wait_for_service(self.agent_namespace + 'set_mode')

        try:
            set_mode = rospy.ServiceProxy(self.agent_namespace + 'set_mode', SetMode)
            set_mode(base_mode=0, custom_mode='GUIDED')
            rospy.loginfo("Mode set to GUIDED.")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Setting mode failed: {e}")


    # Function to takeoff
    def takeoff(self, height):

        rospy.wait_for_service(self.agent_namespace + 'cmd/takeoff')
        takeoff_client = rospy.ServiceProxy(self.agent_namespace + 'cmd/takeoff', CommandTOL)
        takeoff_client(altitude=height)
        rospy.sleep(8)
        rospy.loginfo("Vehicle Took Off")


    # Function to land
    def land(self):

        rospy.wait_for_service(self.agent_namespace + 'cmd/land')
        land_client = rospy.ServiceProxy(self.agent_namespace + 'cmd/land', CommandTOL)

        try:
            land_client(altitude=0.0)
            rospy.loginfo("Vehicle Descending")

        except rospy.ServiceException as e:
            rospy.logerr(f"Landing failed: {e}")


    # Relative vector based collision avoidance method
    def check_collision(self):

        self.collision = False
        if self.drone1_velocity and self.drone2_velocity:
            
            # Calculate relative velocity vector
            relative_velocity_vect = (
                self.drone1_velocity.twist.linear.x - self.drone2_velocity.twist.linear.x,
                self.drone1_velocity.twist.linear.y - self.drone2_velocity.twist.linear.y,
                self.drone1_velocity.twist.linear.z - self.drone2_velocity.twist.linear.z
            )

            # Calculate relative velocity magnitude
            relative_velocity = math.sqrt(sum(v ** 2 for v in relative_velocity_vect))
            
            x1, y1, _, _ = utm.from_latlon(self.current_position.latitude, self.current_position.longitude)
            x2, y2, _, _ = utm.from_latlon(self.drone2_position.latitude, self.drone2_position.longitude)

            # Calculate relative distance vector 
            relative_distance_vect = (x2 - x1, y2 - y1, self.drone2_position.altitude - self.current_position.altitude)

            # Calculate relative distance magnitude 
            relative_distance = math.sqrt(sum(d ** 2 for d in relative_distance_vect))
            
            # Check for division by zero
            if relative_distance > 0 and relative_velocity > 0:

                # Calculate costheta using dot product
                costheta = ((relative_distance_vect[0] * relative_velocity_vect[0]) +
                            (relative_distance_vect[1] * relative_velocity_vect[1]) +
                            (relative_distance_vect[2] * relative_velocity_vect[2])) / (relative_distance * relative_velocity)

                # Clamp costheta to the range [-1, 1]
                costheta = max(-1, min(1, costheta))

                # Predict seperation distance after time 'ttc'
                dist = relative_distance * (math.tan(math.acos(costheta)))
                ttc= math.sqrt(dist**2 + relative_distance**2) / relative_velocity
                
                # Check if predicted seperation distance violates minimum safe distance 
                # and if costheta lies in quadrants 1 or 4
                if dist < self.min_safe_dist and costheta > 0:

                    self.collision = True

                    # Recommend warning only once
                    if not self.Recommend:
                        rospy.logwarn(f"Drone {self.agent_id}: Recommending Evasive Maneuvers...")
                        self.Recommend = True

                    # Velocity coefficient as per cooperative status
                    v_coeff= 1
                    if self.drone2_cooperative is not None:
                        v_coeff= self.drone2_cooperative / (self.cooperative + self.drone2_cooperative)

                    # Magnitude of desired velocity to avoid collision 
                    vel_des= relative_distance/(math.cos(math.atan(self.min_safe_dist/relative_distance)) * ttc)
                    
                    # Magnitude of corrective velocity required perpendicular to current relative velocity
                    vel_correct_req= math.sqrt(vel_des**2 - relative_velocity**2) * v_coeff


                    # Corrective velocity vector perpendicular to current relative velocity
                    # in the horizontal plane
                    mag= math.sqrt(relative_velocity_vect[0]**2 + relative_velocity_vect[1]**2)
                    v= (-(vel_correct_req * relative_velocity_vect[1])/mag, vel_correct_req * relative_velocity_vect[0]/mag, 0)

                    # Possible required corrected velocity vectors
                    v_new1= (self.drone1_velocity.twist.linear.x + v[0], self.drone1_velocity.twist.linear.y + v[1], self.drone1_velocity.twist.linear.z + v[2])
                    v_new2= (self.drone1_velocity.twist.linear.x - v[0], self.drone1_velocity.twist.linear.y - v[1], self.drone1_velocity.twist.linear.z - v[2])

                    # Calculated new predicted distance after velocity correction with v_new1
                    new_costheta = ((relative_distance_vect[0] * v_new1[0]) +
                                (relative_distance_vect[1] * v_new1[1]) +
                                (relative_distance_vect[2] * v_new1[2])) / (relative_distance * vel_des)

                    new_costheta = max(-1, min(1, new_costheta))
                    dist = relative_distance * (math.tan(math.acos(new_costheta)))
                    
                    # Corrected velocity vector which satisfies minimum safe distance is published
                    if dist >= self.min_safe_dist:
                        self.publish_velocity(v_new1)

                    else:
                        self.publish_velocity(v_new2)

                else:
                    if self.Recommend:

                        self.Recommend = False
                        rospy.loginfo(f"Drone {self.agent_id}: Collision Averted.")

            # Error in acquiring position/velocity of either drone
            else:
                rospy.logwarn("Zero relative distance or velocity; cannot calculate collision risk.")

             

    def run(self):
        
        for waypoint in self.waypoints:
            rospy.loginfo(f"Going to Waypoint... {waypoint}")

            while not rospy.is_shutdown():
                
                # If cooperative, publish cooperative status
                if self.cooperative is not None:
                    self.send_cooperative(self.cooperative)
                    
                # Calculate relative distance
                if self.drone2_position:
                    relative_distance = self.distance_to_aircraft((self.drone2_position.latitude, self.drone2_position.longitude, self.drone2_position.altitude))
                    self.min_dist= min(self.min_dist, relative_distance)

                if relative_distance is not None:

                    # Check if adversary drone is within alert distance
                    if relative_distance < self.alert_dist and self.cooperative is not None:

                        if self.Alert==False:
                            rospy.logwarn(f"Drone {self.agent_id}: Traffic Alert.")
                            self.Alert= True
                        
                        # Check for collision
                        # Publish corrective velocity if collision detected
                        self.check_collision()

                        # Publish waypoint if collision not detected
                        if self.collision==False:
                            self.publish_waypoint(waypoint)

                            # Check if current position is within 0.5m of waypoint
                            if self.current_position:
                                distance = self.distance_to_waypoint(waypoint)
                                if distance < 0.5:
                                    rospy.loginfo(f"Reached waypoint: {waypoint}")
                                    break

                    # Publish waypoint if not within alert distance
                    else:
                        self.Alert=False
                        self.publish_waypoint(waypoint)

                        # Check if current position is within 0.5m of waypoint
                        if self.current_position:
                            distance = self.distance_to_waypoint(waypoint)
                            if distance < 0.5:
                                rospy.loginfo(f"Reached waypoint: {waypoint}")
                                break

                self.rate.sleep()
            time.sleep(1)

        # Note time taken since start time
        self.time= rospy.Time.now().secs - self.time
        rospy.loginfo(f"Drone{agent_id} Time: {self.time}")
        rospy.loginfo(f"Drone{agent_id} Min Dist: {self.min_dist}")

        # Land after having reached all waypoints
        self.land()



#-----------------------------------------------------------------------------------------------------------------------------------#
if __name__ == '__main__':

    agent_id = 1                                                             # Change this to your agent ID
    cooperative = 1                                                          # Change this to your cooperative strategy
    csv_file = '/home/usr/cas_ws/src/collision_avoidance/src/waypoints.csv'  # Change this to your CSV file path

    waypoint_follower = WaypointFollower(agent_id, csv_file, cooperative)
    waypoint_follower.run()

