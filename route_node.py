#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image

rospy.init_node('router', anonymous=True)

pub= rospy.Publisher('/camera/rgb/image_raw', Image,queue_size=10)
def ORSinterface():
   
   
   
   rate = rospy.Rate(60) # 10hz
   while not rospy.is_shutdown():
      #hello_str = "hello world %s" % rospy.get_time()
      #rospy.loginfo(hello_str)
      
      rospy.Subscriber("/uav/camera/left/image_rect_color", Image, callback)
      #rate.sleep()



def callback(data):
   pub.publish(data)

   #rospy.loginfo("I heard %s",data.data)
   

if __name__ == '__main__':
   try:
      ORSinterface()

   except rospy.ROSInterruptException:
      pass
