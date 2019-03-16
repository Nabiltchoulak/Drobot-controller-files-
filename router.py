#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from tf2_msgs.msg import TFMessage


position=10 # irouh l'position x=4 en principe ... hadi hya denya
altitude=2 # irouh l'l'atitude z=2
rospy.init_node('FG_autopilot', anonymous=True)
pub = rospy.Publisher('/uav/input/rateThrust',RateThrust ,queue_size=1)
#rate_data=RateThrust
#print(rate_data)
rate=rospy.Rate(1)
def rate_data():
   rospy.Subscriber("/tf", Tf, callback)


def callback(data):
   print(data.transforms.transform.translation.z)

if __name__ == '__main__':
   try:
      rate_data()

   except rospy.ROSInterruptException:
      pass

"""
while(1):
    
    x=mesurer(x) # forward displacement (distance forward :) ) à partir de l'IMU
    pitch=mesurer(pitch)#mesurer pitch à partir de l'IMU
    roll=mesurer(roll)
    yaw=mesurer(roll)
    altitude=mesurer(altitude)
    err_x_integral= err_x_integral + (position-x)
    u= 0.5 * err_x_integral 
    err_pitch_rate= u-pitch
    pitch_rate= 5*err_pitch_rate # le pitch_rate à injecter 
    
    roll_rate=-5*roll
    yaw_rate=-5*yaw
    thrust=5*(2-altitude)"""
