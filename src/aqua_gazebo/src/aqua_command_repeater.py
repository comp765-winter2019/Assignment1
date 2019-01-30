#!/usr/bin/env python
import rospy
from aquacore.msg import Command,PeriodicLegCommand

if __name__=='__main__':
    rospy.init_node('aqua_command_repeater')
    
    src_cmd_topic = rospy.get_param('src_cmd_topic','/aqua/command')
    dst_cmd_topic = rospy.get_param('dst_cmd_topic','/simulator/aqua/command')
    src_plc_topic = rospy.get_param('src_plc_topic','/aqua/periodic_leg_command')
    dst_plc_topic = rospy.get_param('dst_plc_topic','/simulator/aqua/periodic_leg_command')
    
    cmd_pub = rospy.Publisher(dst_cmd_topic, Command, queue_size=1)   
    plc_pub = rospy.Publisher(dst_plc_topic, PeriodicLegCommand, queue_size=1)

    def cmd_repeater(msg):
        rospy.loginfo(src_cmd_topic)
        cmd_pub.publish(msg)

    def plc_repeater(msg):
        plc_pub.publish(msg)
    
    rospy.loginfo(src_cmd_topic)
    rospy.loginfo(src_plc_topic)
    cmd_sub = rospy.Subscriber(src_cmd_topic, Command, cmd_repeater)
    plc_sub = rospy.Subscriber(src_plc_topic, PeriodicLegCommand, plc_repeater)

    rospy.spin()
