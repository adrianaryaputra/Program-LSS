#!/usr/bin/python3
import rospy
from missions.path_following import PathFollower
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from lss_srvs.srv import StringService, StringServiceResponse, StringServiceRequest

class MissionManagerNode:
    def __init__(self):
        self.path_subscriber = rospy.Subscriber('/waypoints/mission_1/in',Path,self.cb_path)
        self.is_started = False
        self.speed = 0.6
        self.rrate = rospy.Rate(15)
        self.path_follower = PathFollower()
        def start_mission_srv(a : StringServiceRequest):
            ret = StringServiceResponse()
            if a.data == 'start':
                self.is_started = True
                ret.success = True
            elif a.data == 'stop':
                self.is_started = False
                ret.success = True
            elif a.data == 'restart':
                self.restart()
                ret.success = True
            else:
                ret.success= False
            return ret
        self.err_pub = rospy.Publisher('/autonomy/mission_manager/error',Float64,queue_size=1)
        self.trg_pub = rospy.Publisher('/autonomy/mission_manager/target',Float64,queue_size=1)
        self.service = rospy.Service('/autonomy/mission_manager/cmd',StringService,start_mission_srv)
        self.pub = rospy.Publisher('/autonomy/pathfollowing',Twist,queue_size=1)
    
    def restart(self):
        self.path_follower.path_idx = 1
        self.path_follower.path_idx_before = 0
        print("Mission Restart")
    
    def update(self):
        if not self.is_started:
            print('Mission Not Started')
            return
        if self.path_follower.is_arrived_last_WP():
            self.status = 'finished'
            print('Mission Finished')
        else:
            self.error = self.path_follower.update()
            msg = Twist()
            msg.linear.x = self.speed
            msg.angular.z = self.error
            err = Float64()
            print(self.error)
            err.data = self.error
            self.err_pub.publish(err)
            err.data = 0.0
            self.trg_pub.publish(err)
            self.pub.publish(msg)
            print('Mission Running')
    
    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rrate.sleep()

    def cb_path(self,msg : Path):
        self.path_follower.set_path(msg)
        self.path_follower.path_idx_before = 0
        for i in range(100):
            print('Buoy Avoidance Gate Path')
        print(msg)

def main():
    rospy.init_node('mission_manager_node')
    m = MissionManagerNode()
    m.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")