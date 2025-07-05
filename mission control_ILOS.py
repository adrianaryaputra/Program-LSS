import numpy as np
import rospy
from utils.marker_publisher import MarkerPublisher
from utils.basic_pid import BasicPID  # Traditional PID controller
from utils.fuzzy_pid import FuzzyPID  # Fuzzy PID controller
from utils.boat_tf import BoatTF
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from utils.utility import get_angle, rad_to_deg

class PathFollower:
    def __init__(self, controller_type='traditional') -> None:
        self.objects = []
        
        # Hardcoded controller type
        self.controller_type = 'traditional'  # Change this to 'fuzzy' to use the fuzzy PID
        self.set_controller(self.controller_type)

        self.main_path = Path()
        self.current_position = NavSatFix()

        self.path_idx = 1
        self.marker_publisher = MarkerPublisher()
        self.boat_tf = BoatTF()

        # Initialize integral errors
        self.integral_angle_error = 0.0
        self.integral_distance_error = 0.0
        self.integral_time_step = 0.1  # Set your desired time step for integration

    def set_controller(self, controller_type):
        """Set the control algorithm based on the selected type."""
        if controller_type == 'traditional':
            self.controller = BasicPID(0.1, 0.01, 0.05)  # P, I, D gains for traditional PID
        elif controller_type == 'fuzzy':
            self.controller = FuzzyPID()  # Assuming a fuzzy PID instance
            self.controller.p = [0.02, 0.03]  # Tune fuzzy parameters as needed
        else:
            raise ValueError(f"Invalid controller type: {controller_type}")

    def update(self):
        # Existing update logic remains the same...
        self.boat_tf.listen_TF()
        if not self.boat_tf.is_tf_available:
            return 0
        
        if self.is_arrived_last_WP():
            print("ARRIVED TO LAST WP")
            return 0
        
        if len(self.main_path.poses) < self.path_idx + 1:
            print("NO WAYPOINT LEFT")
            return 0
        
        if self.is_arrived_path():
            print("ARRIVED ", self.path_idx)
            self.path_idx += 1
        
        self.marker_publisher.set_marker_color()
        self.marker_publisher.publish_marker(
            self.boat_tf.pos, 
            self.main_path.poses[self.path_idx].pose.position, 
            self.main_path.poses[self.path_idx - 1].pose.position
        )
        
        # ILOS Control Logic
        target_wp = self.main_path.poses[self.path_idx].pose.position
        previous_wp = self.main_path.poses[self.path_idx - 1].pose.position
        
        # Calculate angle to the target waypoint
        angle_to_target = get_angle(previous_wp, target_wp)
        path_error_angle = rad_to_deg(angle_to_target - self.boat_tf.yaw)

        if path_error_angle < -180:
            path_error_angle += 360
        elif path_error_angle > 180:
            path_error_angle -= 360

        # Calculate distance to the target waypoint
        path_error_dist = np.sqrt(
            (target_wp.x - self.boat_tf.pos.x) ** 2 + 
            (target_wp.y - self.boat_tf.pos.y) ** 2
        )

        # Update integral errors
        self.integral_angle_error += path_error_angle * self.integral_time_step
        self.integral_distance_error += path_error_dist * self.integral_time_step

        # Use selected controller to get control outputs
        path_control_angle_out = self.controller.update(path_error_angle, self.integral_angle_error)
        path_control_distance_out = self.controller.update(path_error_dist, self.integral_distance_error)

        # Combine control outputs
        result = path_control_angle_out + path_control_distance_out
        
        return result

    def is_arrived_path(self):
        if len(self.main_path.poses) - 1 < self.path_idx:
            return True
        controlled_path_point = self.main_path.poses[self.path_idx].pose.position
        controlled_path_point_before = self.main_path.poses[self.path_idx - 1].pose.position
        
        angle = rad_to_deg(get_angle(self.boat_tf.pos, controlled_path_point) - get_angle(controlled_path_point_before, controlled_path_point))
        
        delta_x = self.boat_tf.pos.x - controlled_path_point.x
        delta_y = self.boat_tf.pos.y - controlled_path_point.y

        return not (abs(angle) < 60 and (delta_x ** 2 + delta_y ** 2) > (2.0 ** 2))
    
    def is_arrived_last_WP(self):
        return self.path_idx >= len(self.main_path.poses) - 1 and self.is_arrived_path()
    
    def set_path(self, path: Path):
        self.main_path = path
        self.path_idx = 1

    # Add any additional methods as needed...

# Main script to run the program
if __name__ == "__main__":
    # Create the PathFollower instance with the hardcoded controller
    path_follower = PathFollower()

    # Continue with the rest of your ROS setup and main loop...
    rospy.init_node('path_follower_node')

    # Your main loop
    while not rospy.is_shutdown():
        control_signal = path_follower.update()
        # Implement the control signal in your robot's movement logic
