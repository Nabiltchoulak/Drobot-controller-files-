#!/usr/bin/env python
# license removed for brevity
import rospy
from mav_msgs.msg import RateThrust
import ControlCommand.msg


def ORSinterface():
   seq=0
   rate=rospy.Rate(1000)
   while not rospy.is_shutdown():
      #hello_str = "hello world %s" % rospy.get_time()
      #rospy.loginfo(hello_str)
      
      rospy.Subscriber("/autopilot/control_command_input", ControlCommand.msg, callback)
      #rate.sleep()

def clean(control):
    cleaned=RateThrust()
    cleaned.head=control.head
    cleaned.angular_rates=control.bodyrates
    cleaned.thrust=control.collective_thrust
    return cleaned

def callback(data):
   
   cleaned_data=clean(data)
   pub.publish(cleaned_data)



if __name__ == '__main__':
    rospy.init_node('FG_autopilot_thrust', anonymous=True)
    pub = rospy.Publisher('/uav/input/rateThrust',RateThrust ,queue_size=1)

    try:
      ORSinterface()

   except rospy.ROSInterruptException:
      pass
