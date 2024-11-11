#!/usr/bin/env python3

import rospy
import csv
import time
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header, Int32
from geographic_msgs.msg import GeoPoseStamped
import math
import utm
from pygeodesy.geoids import GeoidPGM

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

class WaypointFollower:
    def __init__(self, agent_id, csv_file):
        rospy.init_node('waypoint_follower', anonymous=True)
        self.agent_namespace = f'/drone{agent_id}/mavros/'

        self.current_state = State()
        self.initial_takeoff_altitude = 0
        self.current_position = None
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
        self.drone1_velocity_sub = rospy.Subscriber(self.agent_namespace + 'global_position/gp_vel', TwistStamped, self.drone1_velocity_callback)
        self.drone2_velocity_sub = rospy.Subscriber('/drone2/mavros/global_position/gp_vel', TwistStamped, self.drone2_velocity_callback)

        self.setpoint_pub = rospy.Publisher(self.agent_namespace + 'setpoint_position/global', GeoPoseStamped, queue_size=10)
        self.cooperative_pub = rospy.Publisher(self.agent_namespace + 'cooperative', Int32, queue_size=10)

        self.rate = rospy.Rate(20)

        # Cooperative variable
        self.cooperative = 1
        self.cooperative_pub.publish(self.cooperative)

        # Distance alert threshold
        self.alert_dist = 20.0  
        self.safe_dist= 5.0
        self.collision= False
        self.Alert = False
        self.Recommend = False

        # Wait for FCU connection
        rospy.loginfo("Connecting Agent")
        while not self.current_state.connected:
            self.rate.sleep()

        self.arm_drone()

        # Wait for the position to be available
        while self.current_position is None:
            rospy.loginfo("Waiting for initial position...")
            self.rate.sleep()

        # Set the initial takeoff altitude from the current position
        self.initial_takeoff_altitude = self.current_position.altitude

        self.takeoff(3.0)  # Take off to 3 meters above the initial altitude

        self.waypoints = self.load_waypoints(csv_file, agent_id)

    def load_waypoints(self, csv_file, agent_id):
        waypoints = []
        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)

            # Skip the header row
            header = next(reader)

            for row in reader:
                try:
                    index = (agent_id - 1) * 3
                    lat, lon, alt = map(float, row[index:index + 3])
                    waypoints.append((lat, lon, alt))
                except ValueError as e:
                    rospy.logerr(f"Error converting row {row}: {e}")

        return waypoints

    def position_callback(self, msg):
        self.current_position = msg
        self.current_position.altitude -= _egm96.height(self.current_position.latitude, self.current_position.longitude)

    def drone2_position_callback(self, msg):
        self.drone2_position = msg
        self.drone2_position.altitude -= _egm96.height(self.drone2_position.latitude, self.drone2_position.longitude)

    def drone2_cooperative_callback(self, msg):
        self.drone2_cooperative = msg.data

    def drone1_velocity_callback(self, msg):
        self.drone1_velocity = msg

    def drone2_velocity_callback(self, msg):
        self.drone2_velocity = msg

    def state_callback(self, msg):
        self.current_state = msg

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

    def distance_to_waypoint(self, wp):
        if self.current_position:
            lat1, lon1, alt1 = self.current_position.latitude, self.current_position.longitude, self.current_position.altitude
            lat2, lon2, alt2 = wp
            horizontal_distance = self.haversine(lat1, lon1, lat2, lon2)
            vertical_distance = abs(alt1 - alt2)

            return math.sqrt(horizontal_distance**2 + vertical_distance**2)
        return None

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

    def takeoff(self, height):
        rospy.wait_for_service(self.agent_namespace + 'cmd/takeoff')
        takeoff_client = rospy.ServiceProxy(self.agent_namespace + 'cmd/takeoff', CommandTOL)
        isTakeoff = False
        isHeightAttained = (abs(self.current_position.altitude - (self.initial_takeoff_altitude + height)) <= 0.25)

        while not isHeightAttained:
            if not isTakeoff:
                isTakeoff = takeoff_client(altitude=height).success
                rospy.loginfo("Vehicle Ascending")
                rospy.sleep(7)
            isHeightAttained = (abs(self.current_position.altitude - (self.initial_takeoff_altitude + height)) <= 0.25)

        rospy.loginfo("Vehicle Took Off")

    def land(self):
        rospy.wait_for_service(self.agent_namespace + 'cmd/land')
        land_client = rospy.ServiceProxy(self.agent_namespace + 'cmd/land', CommandTOL)

        try:
            land_client(altitude=0.0)
            rospy.loginfo("Vehicle Descending")
        except rospy.ServiceException as e:
            rospy.logerr(f"Landing failed: {e}")


    def check_collision(self):
        self.collision= False

        if self.drone1_velocity and self.drone2_velocity:
            # Calculate relative velocity in the frame of reference of drone 1
            relative_velocity_vect = (
                self.drone1_velocity.twist.linear.x - self.drone2_velocity.twist.linear.x,
                self.drone1_velocity.twist.linear.y - self.drone2_velocity.twist.linear.y,
                self.drone1_velocity.twist.linear.z - self.drone2_velocity.twist.linear.z
            )

            relative_velocity= math.sqrt(relative_velocity_vect[0]**2 + relative_velocity_vect[1]**2 + relative_velocity_vect[2]**2)  

            y1, x1, _, _= utm.from_latlon(self.current_position.latitude,self.current_position.longitude)
            y2, x2, _, _= utm.from_latlon(self.drone2_position.latitude,self.drone2_position.longitude)

            relative_distance_vect= (x2-x1, y2-y1, self.current_position.altitude - self.drone2_position.altitude)

            relative_distance= math.sqrt(relative_distance_vect[0]**2 + relative_distance_vect[1]**2 + relative_distance_vect[2]**2)

            costheta= ((relative_distance_vect[0]*relative_velocity_vect[0]) + (relative_distance_vect[1]*relative_velocity_vect[1]) + (relative_distance_vect[2]*relative_velocity_vect[2]))/(relative_distance*relative_velocity)

            dist= relative_distance*(math.tan(math.acos(costheta)))

            if dist > self.safe_dist:
                self.collision= True

                if self.Recommend==False:
                    rospy.logwarn("Recomending Evasive Maneuvers...")
                    self.Recommend=True

            else:
                self.Recommend=False
             


    def run(self):
        for waypoint in self.waypoints:
            rospy.loginfo(f"Going to Waypoint... {waypoint}")

            while not rospy.is_shutdown():
                self.cooperative_pub.publish(self.cooperative)

                # Calculate relative distance before publishing waypoint
                if self.drone2_position:
                    relative_distance = self.distance_to_waypoint((self.drone2_position.latitude, self.drone2_position.longitude, self.drone2_position.altitude))

                if relative_distance is not None:
                    if relative_distance < self.alert_dist:
                        if self.Alert==False:
                            
                            rospy.logwarn("Bogey Nearby.")
                            self.Alert= True
                            
                        self.check_collision()

                        if self.collision==False:
                            # Publish the waypoint
                            self.publish_waypoint(waypoint)

                            if self.current_position:
                                distance = self.distance_to_waypoint(waypoint)
                                if distance < 0.5:
                                    rospy.loginfo(f"Reached waypoint: {waypoint}")
                                    break
                        
                    else:
                        self.Alert=False

                        # Publish the waypoint
                        self.publish_waypoint(waypoint)

                        if self.current_position:
                            distance = self.distance_to_waypoint(waypoint)
                            if distance < 0.5:
                                rospy.loginfo(f"Reached waypoint: {waypoint}")
                                break

                self.rate.sleep()
            time.sleep(1)

        self.land()

if __name__ == '__main__':
    agent_id = 1  # Change this to your agent ID
    csv_file = '/home/martian/ak_ws/src/collision_avoidance/src/head_on_traj.csv'  # Change this to your CSV file path
    waypoint_follower = WaypointFollower(agent_id, csv_file)
    waypoint_follower.run()
