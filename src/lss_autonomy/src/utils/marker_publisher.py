import rospy
from visualization_msgs.msg import Marker

class MarkerPublisher:
    def __init__(self) -> None:
        self.marker_pub = rospy.Publisher('/path_following/target_marker',Marker,queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.id = 22
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.ns = 'liness'
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.pose.orientation.w = 1
    
    def publish_marker(self, pos1, pos2, pos3):
        self.marker.points.clear()
        self.marker.header.stamp = rospy.Time.now()
        self.marker.points.append(pos1)
        self.marker.points.append(pos2)
        self.marker.points.append(pos3)
        self.marker_pub.publish(self.marker)
    
    def set_marker_color(self):
        self.marker.color.a = 1
        self.marker.color.r = 0.0
        self.marker.color.g = 1
        self.marker.color.b = 0.2
