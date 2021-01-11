#!/usr/bin/python
import rospy
import math
import time
from std_msgs.msg import Float64

class ManipulatorPub(object):

    def __init__(self):
        
        self.base_to_link_1_position = rospy.Publisher(
            '/mp/base_to_link_1_position_controller/command', Float64, queue_size=1)
        self.link_1_to_link_2_position = rospy.Publisher(
            '/mp/link_1_to_link_2_position_controller/command', Float64, queue_size=1)
        self.link_2_to_link_3_position = rospy.Publisher(
            '/mp/link_2_to_link_3_position_controller/command', Float64, queue_size=1)
        
        rospy.loginfo("Starting ManipulatorPub...")

    def xyz_move(self, x, y, z):
        l2 = 3
        l3 = 3

        base_to_link_1_msg = Float64()
        base_to_link_1_angle = math.atan2(y, x)
        base_to_link_1_msg.data = base_to_link_1_angle
        cos1 = math.cos(base_to_link_1_angle)
        sin1 = math.sin(base_to_link_1_angle)

        cos3 = (x**2 + y**2 + z**2 - l2*l2 - l3*l3)/(2*l2*l3)
        sin3 = math.sqrt(1 - cos3**2)
        link_2_to_link_3_msg = Float64()
        link_2_to_link_3_msg.data = math.atan2(sin3, cos3)

        alpha = math.atan2(z, math.sqrt(x**2 + y**2))
        beta = math.atan2(l3*sin3, l2+l3*cos3)
        link_1_to_link_2_msg = Float64()
        link_1_to_link_2_msg.data = alpha - beta

        rospy.loginfo("calculated x: " + str(cos1*(l2*math.cos(link_1_to_link_2_msg.data)+l3*math.cos(link_2_to_link_3_msg.data+link_1_to_link_2_msg.data))))
        rospy.loginfo("calculated y: " + str(sin1*(l2*math.cos(link_1_to_link_2_msg.data)+l3*math.cos(link_2_to_link_3_msg.data+link_1_to_link_2_msg.data))))
        rospy.loginfo("calculated z: " + str(l2*math.sin(link_1_to_link_2_msg.data)+l3*math.sin(link_2_to_link_3_msg.data+link_1_to_link_2_msg.data)))
    
        link_1_to_link_2_msg.data = link_1_to_link_2_msg.data - math.pi/2  

        self.base_to_link_1_position.publish(base_to_link_1_msg)
        self.link_1_to_link_2_position.publish(link_1_to_link_2_msg)
        self.link_2_to_link_3_position.publish(link_2_to_link_3_msg)

        rospy.loginfo("Sending base_to_link_1 msg: " + str(base_to_link_1_msg))
        rospy.loginfo("Sending link_1_to_link_2 msg: " + str(link_1_to_link_2_msg))
        rospy.loginfo("Sending link_2_to_link_3 msg: " + str(link_2_to_link_3_msg))

def xyz_input_test():
    manipulatorpub_obj = ManipulatorPub()

    while not rospy.is_shutdown():
        print(">>>>>>  Input  >>>>>>\n")
        x = float(raw_input("Input x=>"))
        y = float(raw_input("Input y=>"))
        z = float(raw_input("Input z=>"))
        manipulatorpub_obj.xyz_move(x=x, y=y, z=z)
        print(">>>>>>@@@@@@@@@>>>>>>>>>>\n")

if __name__ == "__main__":
    rospy.init_node('manipulator_pub')
    xyz_input_test()