#!/usr/bin/env python

import rospy
import csv
from datetime import datetime

# Import ROS Messages
from geometry_msgs.msg import Twist, TwistStamped # Added TwistStamped for /mavros/local_position/velocity_body
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix # For Global Position
from std_msgs.msg import Float64      # For Yaw from /tf_simple/yaw
from sensor_msgs.msg import Imu       # For Roll, Pitch, Angular Velocities, Linear Acceleration

# For quaternion to Euler conversion and matrix operations
try:
    import tf.transformations as tf_trans
    import numpy as np
except ImportError:
    rospy.logwarn("tf.transformations or numpy module not found. IMU orientation/velocity transformations cannot be performed. Install 'tf' and 'numpy' (e.g., ros-<distro>-tf, pip install numpy).")
    tf_trans = None
    np = None
import math

# Global variable to store the latest data from all topics
latest_data = {
    'timestamp': None,
    'rudder_angle': None,       # Commanded, from /autonomy/pathfollowing
    'thruster_%': None,         # Commanded, from /autonomy/pathfollowing
    'pos_NED_x': None,          # From /asv/odom
    'pos_NED_y': None,          # From /asv/odom
    # 'pos_NED_z': None,        # Removed, as it's hardcoded to 0 in /asv/odom by boat_tf.py
    'pos_GLB_lat': None,        # From /mavros/global_position/global
    'pos_GLB_long': None,       # From /mavros/global_position/global
    'pos_GLB_alt': None,        # From /mavros/global_position/global

    'acc_BDY_x': None,          # From IMU linear_acceleration
    'acc_BDY_y': None,          # From IMU linear_acceleration
    'acc_BDY_z': None,          # From IMU linear_acceleration

    'vel_BDY_x': None,          # From /mavros/local_position/velocity_body
    'vel_BDY_y': None,          # From /mavros/local_position/velocity_body
    'vel_BDY_z': None,          # From /mavros/local_position/velocity_body

    'vel_NED_x': None,          # Calculated from vel_BDY and attitude
    'vel_NED_y': None,          # Calculated from vel_BDY and attitude
    'vel_NED_z': None,          # Calculated from vel_BDY and attitude

    'vel_GLB_ground_speed': None, # Calculated from vel_NED
    'vel_GLB_course_over_ground': None, # Calculated from vel_NED (radians)
    'vel_GLB_vertical_speed': None, # Calculated as -vel_NED_z

    'att_NED_roll': None,       # From /mavros/imu/data (orientation quaternion)
    'att_NED_pitch': None,      # From /mavros/imu/data (orientation quaternion)
    'att_NED_yaw': None,        # From /tf_simple/yaw (preferred for simplicity as it's processed)

    'attvel_BDY_rollrate': None,# From IMU angular_velocity (body frame roll rate)
    'attvel_BDY_pitchrate': None,# From IMU angular_velocity (body frame pitch rate)
    'attvel_BDY_yawrate': None, # From IMU angular_velocity (body frame yaw rate)
}

# Callback functions
def command_callback(msg):
    """ Handles commanded rudder (angular.z) and thruster (linear.x) """
    latest_data['rudder_angle'] = msg.angular.z  # Assuming this is rudder angle command
    latest_data['thruster_%'] = msg.linear.x   # Assuming this is thruster % command

def odom_callback(msg):
    """ Handles NED position from Odometry message """
    latest_data['pos_NED_x'] = msg.pose.pose.position.x
    latest_data['pos_NED_y'] = msg.pose.pose.position.y
    # latest_data['pos_NED_z'] is not logged as it's hardcoded to 0 by boat_tf.py
    # msg.pose.pose.orientation contains quaternion for roll, pitch, yaw.
    # boat_tf.py sets roll=0, pitch=0 for this odom message. Yaw is derived from compass.
    # msg.twist.twist contains linear and angular velocities.
    # boat_tf.py does not populate these, so they will be 0.
    # If these were populated, we could get:
    # latest_data['vel_NED_x'] = msg.twist.twist.linear.x
    # latest_data['attvel_NED_yawrate'] = msg.twist.twist.angular.z

def global_pos_callback(msg):
    """ Handles Global position from NavSatFix message """
    latest_data['pos_GLB_lat'] = msg.latitude
    latest_data['pos_GLB_long'] = msg.longitude
    latest_data['pos_GLB_alt'] = msg.altitude
    # NavSatFix does not typically provide direct lat/long/alt velocities.
    # It might provide ground speed and course if the GPS unit supports it.

def yaw_ned_callback(msg):
    """ Handles NED Yaw from /tf_simple/yaw """
    latest_data['att_NED_yaw'] = msg.data # Assuming data is already in radians as per boat_tf.py

def imu_callback(msg):
    """ Handles IMU data for attitude (roll, pitch) and angular velocities """
    if tf_trans:
        q = msg.orientation
        euler = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        latest_data['att_NED_roll'] = euler[0]
        latest_data['att_NED_pitch'] = euler[1]
        # Yaw is taken from /tf_simple/yaw, so we don't overwrite it here:
        # latest_data['att_NED_yaw'] = euler[2]
    else:
        # If tf_trans is not available, roll and pitch cannot be calculated easily
        latest_data['att_NED_roll'] = None
        latest_data['att_NED_pitch'] = None

    # Angular velocity (body frame rates)
    latest_data['attvel_BDY_rollrate'] = msg.angular_velocity.x
    latest_data['attvel_BDY_pitchrate'] = msg.angular_velocity.y
    latest_data['attvel_BDY_yawrate'] = msg.angular_velocity.z

    # Linear acceleration (body frame)
    latest_data['acc_BDY_x'] = msg.linear_acceleration.x
    latest_data['acc_BDY_y'] = msg.linear_acceleration.y
    latest_data['acc_BDY_z'] = msg.linear_acceleration.z

def body_vel_callback(msg):
    """ Handles body frame velocities from /mavros/local_position/velocity_body """
    latest_data['vel_BDY_x'] = msg.twist.linear.x
    latest_data['vel_BDY_y'] = msg.twist.linear.y
    latest_data['vel_BDY_z'] = msg.twist.linear.z
    # Angular velocities from this topic could also be used, but IMU is typically the primary source.
    # latest_data['attvel_BDY_rollrate'] = msg.twist.angular.x

    # Attempt to transform body velocities to NED velocities
    if tf_trans and np and \
       latest_data['att_NED_roll'] is not None and \
       latest_data['att_NED_pitch'] is not None and \
       latest_data['att_NED_yaw'] is not None:

        roll = latest_data['att_NED_roll']
        pitch = latest_data['att_NED_pitch']
        yaw = latest_data['att_NED_yaw']

        # Create rotation matrix from Euler angles (body to NED)
        # Standard ZYX rotation sequence for aerospace
        R_body_to_ned = tf_trans.euler_matrix(roll, pitch, yaw, 'sxyz')[:3,:3]
        # Check default 'sxyz' if it matches your convention, or use 'sxyz' for roll,pitch,yaw respectively around fixed axes.
        # Or, more commonly for vehicle dynamics body to NED: ZYX / yaw-pitch-roll
        # R_body_to_ned = tf_trans.euler_matrix(yaw, pitch, roll, 'rzyx')[:3,:3] # This is a common convention
        # For safety, let's use a known standard: yaw, pitch, roll (ZYX) for body to world
        # Using tf_trans.quaternion_from_euler to be sure about the rotation order if euler_matrix is ambiguous
        q_body_to_ned = tf_trans.quaternion_from_euler(roll, pitch, yaw, 'sxyz') # roll,pitch,yaw intrinsic
        R_body_to_ned = tf_trans.quaternion_matrix(q_body_to_ned)[:3,:3]


        vel_body_vector = np.array([latest_data['vel_BDY_x'],
                                    latest_data['vel_BDY_y'],
                                    latest_data['vel_BDY_z']])

        vel_ned_vector = R_body_to_ned.dot(vel_body_vector)

        latest_data['vel_NED_x'] = vel_ned_vector[0]
        latest_data['vel_NED_y'] = vel_ned_vector[1]
        latest_data['vel_NED_z'] = vel_ned_vector[2]

        # Calculate GLB speed and course
        latest_data['vel_GLB_ground_speed'] = math.sqrt(vel_ned_vector[0]**2 + vel_ned_vector[1]**2)
        latest_data['vel_GLB_course_over_ground'] = math.atan2(vel_ned_vector[1], vel_ned_vector[0]) # Radians
        latest_data['vel_GLB_vertical_speed'] = -vel_ned_vector[2] # NED Z is down
    else:
        latest_data['vel_NED_x'] = None
        latest_data['vel_NED_y'] = None
        latest_data['vel_NED_z'] = None
        latest_data['vel_GLB_ground_speed'] = None
        latest_data['vel_GLB_course_over_ground'] = None
        latest_data['vel_GLB_vertical_speed'] = None

def data_logger():
    rospy.init_node('ship_data_logger', anonymous=True)

    output_file_name = rospy.get_param('~output_file', 'ship_data_logged.csv')
    log_rate_hz = rospy.get_param('~log_rate_hz', 1.0) # Default 1 Hz

    rospy.loginfo("Ship Data Logger started. Outputting to: %s at %s Hz", output_file_name, log_rate_hz)

    # Subscribers
    rospy.Subscriber('/autonomy/pathfollowing', Twist, command_callback)
    rospy.Subscriber('/asv/odom', Odometry, odom_callback)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, global_pos_callback)
    rospy.Subscriber('/tf_simple/yaw', Float64, yaw_ned_callback)
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, body_vel_callback)


    rate = rospy.Rate(log_rate_hz)

    # Define the fieldnames for the CSV file based on available data
    fieldnames = [
        'timestamp', 'rudder_angle', 'thruster_%',
        'pos_NED_x', 'pos_NED_y',
        'pos_GLB_lat', 'pos_GLB_long', 'pos_GLB_alt',
        'acc_BDY_x', 'acc_BDY_y', 'acc_BDY_z',
        'vel_BDY_x', 'vel_BDY_y', 'vel_BDY_z',
        'vel_NED_x', 'vel_NED_y', 'vel_NED_z',
        'vel_GLB_ground_speed', 'vel_GLB_course_over_ground', 'vel_GLB_vertical_speed',
        'att_NED_roll', 'att_NED_pitch', 'att_NED_yaw',
        'attvel_BDY_rollrate', 'attvel_BDY_pitchrate', 'attvel_BDY_yawrate'
    ]
    # Note: Original vel_GLB_lat, vel_GLB_long, vel_GLB_alt are replaced by
    # vel_GLB_ground_speed, vel_GLB_course_over_ground, vel_GLB_vertical_speed for practicality.

    # Filter latest_data keys to only include those in the final fieldnames
    # This ensures that if a key was in latest_data for development but removed from fieldnames,
    # it doesn't cause an error with DictWriter.
    # However, it's cleaner to define latest_data with only the keys that will be used.
    # For this iteration, we'll keep latest_data as is, and DictWriter will only use specified fieldnames.

    rospy.loginfo("Logging the following fields: %s", ", ".join(fieldnames))

    try:
        # Adding newline='' for better cross-platform CSV compatibility
        with open(output_file_name, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            rospy.loginfo("CSV file '%s' created and header written.", output_file_name)

            while not rospy.is_shutdown():
                current_time = datetime.now().isoformat()
                latest_data['timestamp'] = current_time

                # Create a snapshot of the data to write for this row
                row_to_write = {key: latest_data.get(key) for key in fieldnames}

                writer.writerow(row_to_write)
                # rospy.loginfo_throttle(5, "Writing data to CSV: %s", row_to_write) # For debugging
                rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted. Shutting down.")
    except IOError as e:
        rospy.logerr("IOError writing to %s: %s", output_file_name, e)
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", e)
    finally:
        rospy.loginfo("Logger stopped.")

if __name__ == '__main__':
    try:
        data_logger()
    except Exception as e:
        rospy.logerr("Unhandled exception in main scope: %s", e)
