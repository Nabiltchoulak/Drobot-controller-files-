#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
def callback(data):
   pub.publish(data)
   rospy.loginfo("I heard %s",data.data)

def ORSinterface():
   pub = rospy.Publisher('image_left/image_color_rect', Image,queue_size=10)
   
   rospy.init_node('router', anonymous=True)
   rate = rospy.Rate(60) # 10hz
   while not rospy.is_shutdown():
      #hello_str = "hello world %s" % rospy.get_time()
      #rospy.loginfo(hello_str)
      rospy.Subscriber("/uav/camera/left/image_rect_color", Image, callback)
      rate.sleep()
   
   

if __name__ == '__main__':
   try:
      ORSinterface()

   except rospy.ROSInterruptException:
      pass
