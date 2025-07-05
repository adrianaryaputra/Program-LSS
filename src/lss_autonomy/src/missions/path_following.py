import numpy as np
import rospy
from utils.marker_publisher import MarkerPublisher
from utils.basic_pid import BasicPID
from utils.fuzzy_pid import FuzzyPID
from utils.boat_tf import BoatTF
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from nav_msgs.msg  import Path
from utils.utility import get_angle, rad_to_deg, distance_line_to_point

class PathFollower:
    def __init__(self) -> None:

        self.objects = []
        self.pid_angle = FuzzyPID()
        self.pid_angle.p = [0.02, 0.03]

        self.pid_distance = FuzzyPID()
        self.pid_distance.p = [0.4, 0.6]
        self.pid_distance.upper_bound = 1.0
        self.pid_distance.lower_bound = -1.0

        self.error_angle = 0.0
        self.error_distance = 0.0
        self.path_angle_limit = 60
        self.path_keep_distance = 2.0

        self.main_path = Path()
        self.current_position = NavSatFix()

        self.path_idx = 1
        self.path_idx_before = 0
        self.marker_publisher = MarkerPublisher()
        self.boat_tf = BoatTF()

    
    def update(self):
        self.boat_tf.listen_TF()
        if not self.boat_tf.is_tf_available:
            return 0
        
        if self.is_arrived_last_WP():
            print("ARRIVED TO LAST WP")
            return 0
        
        if(len(self.main_path.poses) < self.path_idx + 1):
            print("NO WAYPOINT LEFT")
            return 0

        if self.is_arrived_path():
            print("ARRIVED ",self.path_idx)
            self.path_idx += 1
        
        self.marker_publisher.set_marker_color()
        self.marker_publisher.publish_marker(self.boat_tf.pos, 
                                             self.main_path.poses[self.path_idx].pose.position, 
                                             self.main_path.poses[self.path_idx - 1].pose.position)
        
        angle_path = get_angle(self.main_path.poses[self.path_idx - 1].pose.position,
                                self.main_path.poses[self.path_idx].pose.position)
        path_error_angle = rad_to_deg(angle_path - self.boat_tf.yaw)

        if ( path_error_angle < -180 ):
            path_error_angle += 360
        elif (path_error_angle > 180):
            path_error_angle -= 360

        path_error_dist = distance_line_to_point(self.main_path.poses[self.path_idx -1].pose.position, 
                                                 self.main_path.poses[self.path_idx].pose.position, self.boat_tf.pos)
        if path_error_dist > 4:
            path_error_dist = 0
            angle_to_first_wp = get_angle(self.boat_tf.pos,self.main_path.poses[self.path_idx-1].pose.position)
            path_error_angle = rad_to_deg(angle_to_first_wp - self.boat_tf.yaw)
            if ( path_error_angle < -180 ):
                path_error_angle += 360
            elif (path_error_angle > 180):
                path_error_angle -= 360
        
        path_control_angle_out = self.pid_angle.update(path_error_angle)
        path_control_distance_out = self.pid_distance.update(path_error_dist)
        result = path_control_angle_out + path_control_distance_out
        # if len(self.objects) > 0:
        #     result2 = self.obstacle_avoidance()
        # if result2 < 0 and result < 0:
        #     result = np.min([result,result2])
        # elif result2 > 0 and result > 0:
        #     result = np.max([result,result2])
        return result
    
    
    def is_arrived_path(self):
        if len(self.main_path.poses)-1 < self.path_idx:
            return True
        controlled_path_point = self.main_path.poses[self.path_idx].pose.position
        controlled_path_point_before = self.main_path.poses[self.path_idx - 1].pose.position
        
        angle = rad_to_deg(get_angle(self.boat_tf.pos, controlled_path_point) - get_angle(controlled_path_point_before, controlled_path_point))
        
        delta_x = self.boat_tf.pos.x - controlled_path_point.x
        delta_y = self.boat_tf.pos.y - controlled_path_point.y

        if np.abs(angle) >= 180:
            angle = 360 - np.abs(angle)
        return not(abs(angle) < self.path_angle_limit and pow(delta_x, 2) + pow(delta_y, 2) > pow(self.path_keep_distance,2))
    
    def is_arrived_last_WP(self):
        if self.path_idx >= len(self.main_path.poses)-1:
            if self.is_arrived_path():
                return True
        return False
    
    def get_path_angle(self):
        if len(self.main_path.poses) <= 1:
            print("WARNING NOT ENOUGH WAYPOINTS")
            return

        angle_path = get_angle(self.main_path.poses[self.path_idx - 1].pose.position,
                                        self.main_path.poses[self.path_idx].pose.position)
        return angle_path
    
    def set_path(self, path : Path):
        self.main_path = path
        self.path_idx = 1
        self.path_idx_before = 0
        pass 
