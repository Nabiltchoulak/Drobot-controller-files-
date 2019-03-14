#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from sensor_msgs.msg import Imu,RateThrust

position=10 # irouh l'position x=4 en principe ... hadi hya denya
altitude=2 # irouh l'l'atitude z=2
seq=0# Séquence du header
rate=rospy.rate(1)
rate_data=RateThrust()
rospy.init_node('take_off', anonymous=True)
pub = rospy.Publisher('/uav/input/rateThrust', Image,queue_size=1)
def rate_data():
    for i in range(2):
        
        rate_data.header.stamp = rospy.Time.now()
        rate_data.header.frame_id = "world"
        rate_data.header.seq = seq
        rate_data.angular_rates.x=np.float64(0.0)
        rate_data.angular_rates.y=np.float64(0.0)
        rate_data.angular_rates.z=np.float64(0.0)
        rate_data.thrust.x=np.float64(0.0)
        rate_data.thrust.x=np.float64(0.0)
        rate_data.thrust.z=np.float64(float(i+1 % 2))
        pub.publish(rate_data)
        rate.sleep()
        seq+=1
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

