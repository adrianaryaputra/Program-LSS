import rospy 
import tf2_ros
import tf
from geometry_msgs.msg import Point

class BoatTF:
    def __init__(self):
        self.pos = Point()
        self.pos.x = 0
        self.pos.y = 0
        self.yaw = 0.0
        self.is_tf_available = False
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def listen_TF(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'asv/base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        quaternion = (
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        self.pos.x = trans.transform.translation.x
        self.pos.y = trans.transform.translation.y
        self.yaw = yaw
        self.is_tf_available = True