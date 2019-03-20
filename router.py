#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import nav_msgs.msg 
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage


position=10 # irouh l'position x=4 en principe ... hadi hya denya
altitude=2 # irouh l'l'atitude z=2
)
#rate_data=RateThrust
#print(rate_data)

def angle_transf(angle):
    # transform angle in radians to range [-pi,pi]
    
    angle=angle%(2*pi)
    if angle > pi:
        angle=angle-2*pi
    return angle

def to_twist(velocity_x,velocity_y,velocity_z,p,q,r):
   sender=geometry_msgs.msg.TwistWithCovariance()
   ######## linear
   sender.linear.x=velocity_x
   sender.linear.y=velocity_y
   sender.linear.z=velocity_z
   ######## angular 
   sender.angular.x=q
   sender.angular.y=p
   sender.angular.z=r

   return sender

def toEulerAngle(i,j,k,one,*args):
    
    #code found here: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    #handle singularities here: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
    
    q_w,q_x,q_y,q_z=one,i,j,k #modified to match quaternion ordrer given by flightGoggle
    
    #roll (x-axis rotation)
    sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z);
    cosr_cosp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
    roll = atan2(sinr_cosp, cosr_cosp);

    #pitch (y-axis rotation)
    sinp = +2.0 * (q_w * q_y - q_z * q_x);
    if (fabs(sinp) >= 1):
        pitch = copysign(pi / 2, sinp); # use 90 degrees if out of range
    else:
        pitch = asin(sinp);

    #yaw (z-axis rotation)
    siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
    cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
    yaw = atan2(siny_cosp, cosy_cosp);
    
    return (pitch,roll,yaw)

def uav_groundtruth_pose(tf_data):
    
    x=tf_data.transform.translation.x
    y=tf_data.transform.translation.y
    z=tf_data.transform.translation.z
    q1=tf_data.transform.rotation.x
    q2=tf_data.transform.rotation.y
    q3=tf_data.transform.rotation.z
    q4=tf_data.transform.rotation.w
    
    return (x,y,z,q1,q2,q3,q4)

def phidot_thetadot_psidot_to_pqr(phi_dot,theta_dot,psi_dot,theta,phi):
    p=phi_dot - sin(theta)*psi_dot
    q=cos(phi)*theta_dot + sin(phi)*cos(theta)*psi_dot
    r=-sin(phi)*theta_dot + cos(phi)*cos(theta)*psi_dot

def get_pose(tarns):
   pose=geometry_msgs.msg.PoseWithCovariance()
   pose.pose.orientation=trans.rotation
   pose.pose.position.x=trans.translation.x
   pose.pose.position.y=trans.translation.y
   pose.pose.position.z=trans.translation.z
   return pose

class Derivate:
   def __init__(self,x=0,y=0,z=1,phi=0,theta=0,psi=0,*args)
      self.last_x=x
      self.last_y=y
      self.last_y=z
      self.last_pitch= phi % (2*pi)
      self.last_theta=theta % (2*pi)
      self.last_psi=psi % (2*pi)
      self.velocity=[]

   
   def calculate(trans,delta_t,*args)
      
      ######### linear velocities
      velocity_x=(trans.translation.x-self.last_x)/delta_t
      velocity_y=(trans.translation.y-self.last_y)/delta_t
      velocity_z=(trans.translation.z-self.last_z)/delta_t

      ################# for the next calculate 
      self.last_x=trans.translation.x
      self.last_y=trans.translation.y
      self.last_z=trans.translation.z


      ######### Angular velocities 
      pitch,roll,yaw=toEulerAngle(trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w)
      pitch,roll,yaw=angle_transf(pitch),angle_transf(roll),angle_transf(yaw)
      phi,theta,psi=roll,pitch,yaw

      phi_dot=(phi-self.last_phi)/delta_t
      theta_dot=(theta-self.last_theta)/delta_t
      psi_dot=(psi-self.last_psi)/delta_t

      self.last_phi=phi
      self.last_theta=theta
      self.last_psi=psi

      p, q, r = phidot_thetadot_psidot_to_pqr(phi_dot,theta_dot,psi_dot,theta, phi)

      return(to_twist(velocity_x,velocity_y,velocity_z,p,q,r))##### roll pitch yaw 
      
      

      

      




if __name__ == '__main__':
   ##### Node initiate 
   rospy.init_node('FG_autopilot', anonymous=True)
   pub = rospy.Publisher('/odom',RateThrust ,queue_size=1
   #rate=rospy.Rate(1)
   init_pos=rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")######## initial positions in Quaretnion frame
   initial_pos= init_pos[0:3]
   initial_pos.append(toEulerAngle(init_pos[3:7]))
   ######## initiate tf
   tfBuffer = tf2_ros.Buffer()
   listener = tf2_ros.TransformListener(tfBuffer)

   Velocity= Derivate(initial_pos)

   seq=0
   rate=rospy.Rate(1000)
   delta_t=0.001
   while not rospy.is_shutdown():
        
        ######### lookup for tf data 
        try:

            trans = tfBuffer.lookup_transform("world", 'uav/imu', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
      odom=nav_msgs.msg.Odometry()
      ########### Header
      odom.header.stamp=rospy.Time.now()
      odom.header.seq=seq
      odom.header.frame_id = "odom"
      odom.child_frame_id = "uav/imu"
      ############## Pose and twist 
      odom.pose=get_pose(trans)
      odom.twist=Derivate.calculate(trans,delta_t)

      
      pub.publish(odom)

