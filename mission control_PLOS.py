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
    def __init__(self, controller_type='traditional', los_distance=5.0) -> None:
        self.objects = []
        
        # Hardcoded controller type
        self.controller_type = controller_type  # 'traditional' or 'fuzzy'
        self.set_controller(self.controller_type)

        self.los_distance = los_distance  # Lookahead distance for PLOS
        self.main_path = Path()
        self.current_position = NavSatFix()

        self.path_idx = 1
        self.marker_publisher = MarkerPublisher()
        self.boat_tf = BoatTF()

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
        # Update the boat's position using the TF listener
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

        # Proportional Line of Sight (PLOS) Control Logic
        los_target_wp = self.compute_los_target_wp(self.los_distance)
        los_angle = get_angle(self.boat_tf.pos, los_target_wp)
        path_error_angle = rad_to_deg(los_angle - self.boat_tf.yaw)

        # Adjust angle for wraparound between -180 and 180 degrees
        if path_error_angle < -180:
            path_error_angle += 360
        elif path_error_angle > 180:
            path_error_angle -= 360

        # Calculate cross-track error
        path_error_dist = self.compute_cross_track_error()

        # Use selected controller to get control outputs
        control_angle_out = self.controller.update(path_error_angle)
        control_distance_out = self.controller.update(path_error_dist)

        # Combine control outputs
        result = control_angle_out + control_distance_out
        return result

    def compute_los_target_wp(self, los_distance):
        """Compute the Line of Sight (LOS) target waypoint based on the current position and lookahead distance."""
        current_wp = self.main_path.poses[self.path_idx - 1].pose.position
        next_wp = self.main_path.poses[self.path_idx].pose.position

        path_vector = np.array([next_wp.x - current_wp.x, next_wp.y - current_wp.y])
        path_length = np.linalg.norm(path_vector)

        if path_length == 0:
            return next_wp

        # Normalize path vector
        unit_path_vector = path_vector / path_length

        # Compute LOS target position by moving forward along the path by `los_distance`
        boat_to_wp_vector = np.array([self.boat_tf.pos.x - current_wp.x, self.boat_tf.pos.y - current_wp.y])
        along_track_dist = np.dot(boat_to_wp_vector, unit_path_vector)
        los_target = Point()

        if along_track_dist + los_distance < path_length:
            los_target.x = current_wp.x + (along_track_dist + los_distance) * unit_path_vector[0]
            los_target.y = current_wp.y + (along_track_dist + los_distance) * unit_path_vector[1]
        else:
            los_target = next_wp

        return los_target

    def compute_cross_track_error(self):
        """Compute the cross-track error based on the boat's position and the current path segment."""
        current_wp = self.main_path.poses[self.path_idx - 1].pose.position
        next_wp = self.main_path.poses[self.path_idx].pose.position

        path_vector = np.array([next_wp.x - current_wp.x, next_wp.y - current_wp.y])
        boat_vector = np.array([self.boat_tf.pos.x - current_wp.x, self.boat_tf.pos.y - current_wp.y])

        path_length = np.linalg.norm(path_vector)
        if path_length == 0:
            return 0.0

        # Normalize path vector
        unit_path_vector = path_vector / path_length

        # Compute the cross-track error using the perpendicular distance formula
        cross_track_error = np.cross(unit_path_vector, boat_vector)
        return cross_track_error

    def is_arrived_path(self):
        """Check if the boat has arrived at the current waypoint."""
        if len(self.main_path.poses) - 1 < self.path_idx:
            return True

        current_wp = self.main_path.poses[self.path_idx].pose.position
        delta_x = self.boat_tf.pos.x - current_wp.x
        delta_y = self.boat_tf.pos.y - current_wp.y

        return np.sqrt(delta_x**2 + delta_y**2) < 2.0  # Assuming 2m threshold for arrival
    
    def is_arrived_last_WP(self):
        """Check if the boat has arrived at the last waypoint."""
        return self.path_idx >= len(self.main_path.poses) - 1 and self.is_arrived_path()
    
    def set_path(self, path: Path):
        """Set the path for the boat to follow."""
        self.main_path = path
        self.path_idx = 1

# Main script to run the program
if __name__ == "__main__":
    # Create the PathFollower instance with the hardcoded controller
    path_follower = PathFollower(controller_type='traditional')  # Change to 'fuzzy' if needed

    # ROS setup and main loop
    rospy.init_node('path_follower_node')

    # Main control loop
    while not rospy.is_shutdown():
        control_signal = path_follower.update()
        # Implement the control signal in your robot's movement logic
