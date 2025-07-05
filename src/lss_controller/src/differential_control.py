import rospy
import numpy as np
from typing import Tuple
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import serial
import time




class Steering():
    def __init__(self):
        self.radius = np.pi/2
        self.multiplier = 1.0/((np.pi/2) / self.radius + 1.0)
        self.multiplier = 0.5
        self.control_sub = rospy.Subscriber('/autonomy/pathfollowing',Twist,self.get_actuator_input) 
    
    def percent_to_pwm(self,thrust_percentage) -> int:
        clamp = lambda x : max(min(x,2000),1000)
        if abs(thrust_percentage) < 0.001:
            return 1500
        if thrust_percentage > 0:
            return clamp(int(1528 + (1832-1528)*thrust_percentage))
        else:
            return clamp(int(1472 + (1472-1100)*thrust_percentage))
    
    def rad_to_pwm(self,rad) -> int:
        clamp = lambda x : max(min(x,2000),1000)
        return  clamp(1500+int(-rad * 500 / (np.pi)))

    def get_actuator_input(self,target : Twist) -> Tuple:
        
        speed = target.linear.x
        angle = target.angular.z
        speed = self.percent_to_pwm(speed)
        angle = self.rad_to_pwm(angle)
        print(speed)
        print(angle)
        ser = serial.Serial('/dev/ttyUSB0',
                    baudrate=115200)
        
        to_send_data = b'its'
        to_send_data += speed.to_bytes(2, byteorder='little', signed=False)
        to_send_data += angle.to_bytes(2, byteorder='little', signed=False)
        print(to_send_data)
        ser.write(to_send_data)

if __name__ == '__main__':
    rospy.init_node('control')
    ds = Steering()
    rospy.spin()