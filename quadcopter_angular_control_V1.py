#!/usr/bin/env python
# coding: utf-8
# license removed for brevity
# In[1]:

## This structure is based on the ros tf2 listener provided by tf2 class 
### link : http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29


#############################################       Dependencies       ###################################################
from math import *
from time import time,sleep
import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
from mav_msgs.msg import RateThrust
from tf2_msgs.msg import TFMessage
##############################################      Hyperparameters    ###################################################

##### Nodes handler


##### Constants
g=9.80655 # gravity
m=1 # vehicle mass = 1kg

##### Quadcopter pitch_roll_yaw-rates limitations

max_pitch_rate= pi/4 # rad/s
min_pitch_rate=-max_pitch_rate
max_theta_rate= max_pitch_rate
min_theta_rate=-max_theta_rate


max_roll_rate= pi/4 # rad/s
min_roll_rate=-max_roll_rate
max_phi_rate= max_roll_rate
min_phi_rate=-max_phi_rate


max_yaw_rate= pi/2 # rad/s
min_yaw_rate=-max_yaw_rate
max_psi_rate= max_yaw_rate
min_psi_rate=-max_psi_rate

max_thrust= 37 # 4*thrust_coeff*max_popr_speed**2
min_thrust= 0



##### PIDS
#PID theta_rate
kp_theta_rate,kd_theta_rate,ki_theta_rate=(100,10,1)

#PID phi_rate
kp_phi_rate,kd_phi_rate,ki_phi_rate=(100,10,1)

#PID psi_rate
kp_psi_rate,kd_psi_rate,ki_psi_rate=(100,5,0)

#PID thrust
kp_thrust,kd_thrust,ki_thrust=(28,12,31)



############################################      Utils functions     #####################################################

def pqr_to_phidot_thetadot_psidot(p,q,r,theta,phi):
    
    phi_dot= p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
    theta_dot= q*cos(phi) - r*sin(phi)
    psi_dot= q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta) 
    
    return(phi_dot,theta_dot,psi_dot)

def toEulerAngle(i,j,k,one):
    
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

def phidot_thetadot_psidot_to_pqr(phi_dot,theta_dot,psi_dot,theta,phi):
    p=phi_dot - sin(theta)*psi_dot
    q=cos(phi)*theta_dot + sin(phi)*cos(theta)*psi_dot
    r=-sin(phi)*theta_dot + cos(phi)*cos(theta)*psi_dot
    
    return(p,q,r)

def angle_transf(angle):
    # transform angle in radians to range [-pi,pi]
    
    angle=angle%(2*pi)
    if angle > pi:
        angle=angle-2*pi
    return angle
        
        



############################################ Quadcopter Controller class #################################################
class AnglesController:
    
    def __init__(self,phi0=0,theta0=0,psi0=0,z0=1):
        
        #phi : roll , theta : pitch , yaw : psi
        self.phi=phi0 % (2*pi)
        self.theta=theta0 % (2*pi)
        self.psi=psi0 % (2*pi)
        self.phi_dot=0 
        self.theta_dot=0 
        self.psi_dot=0 
        
        self.z=z0
        self.z_dot=0
        
        #phi_rate PID errors
        self.error_phi_integral=0
        self.max_phi_integral=max_phi_rate #anti wind-up
        self.last_phi=0 #use last_pitch_rate instead last_error_pitch_rate to avoid derivative kick
        
        #theta_rate PID errors
        self.error_theta_integral=0
        self.max_theta_integral=max_theta_rate #anti wind-up
        self.last_theta=0 #use last_theta_rate instead last_error_theta_rate to avoid derivative kick
        
        #psi_rate PID errors
        self.error_psi_integral=0
        self.max_psi_integral=max_psi_rate #anti wind-up
        self.last_psi=0 #use last_thrust instead last_error_thrust to avoid derivative kick
        
        #thrust PID errors
        self.error_z_integral=0
        self.max_z_integral=max_thrust #anti wind-up
        self.last_z=0 #use last_thrust instead last_error_thrust to avoid derivative kick
        
        
        
    def __repr__(self):
        #string representation of controller class
        return "phi:{} , theta:{} , psi:{} , z:{} ".format(self.phi,self.theta,self.psi,self.z)
    
    def move(self,pitch_rate,roll_rate,yaw_rate,thrust,delta_t):
        # Quadcopter dynamics using Euler's integration
        # Assuming that angular_velocities p,q and r are perfectly controlled
        # p,q,r : angular velocities in the quadcopter frame
        # p,q,r = roll_rate,pitch_rate,yaw_rate 
        
        #z : altitude 
        
        p,q,r = roll_rate,pitch_rate,yaw_rate
         
        self.phi_dot,self.theta_dot,self.psi_dot=pqr_to_phidot_thetadot_psidot(p,q,r,self.theta,self.phi) 
        
        self.phi += delta_t * self.phi_dot + normal(0,a_n)
        self.theta += delta_t * self.theta_dot + normal(0,a_n)
        self.psi+= delta_t * self.psi_dot + normal(0,a_n)
        
        self.phi,self.theta,self.psi=angle_transf(self.phi),angle_transf(self.theta),angle_transf(self.psi)
        
        self.z_dot += delta_t*(thrust/m*cos(self.phi)*cos(self.theta) - g ) + normal(0,l_n)
        self.z += delta_t*self.z_dot + normal(0,l_n)
        
        
 
    def compute_phi_rate(self,desired_phi,actual_phi,delta_t):

        error_phi=desired_phi-actual_phi
        
        self.error_phi_integral+=error_phi*delta_t
        self.error_phi_integral=max(-self.max_phi_integral,min(self.error_phi_integral,self.max_phi_integral))

        phi_derivative= (self.last_phi-actual_phi)/delta_t #avoid derivative_kick
        self.last_phi=actual_phi

        phi_rate= kp_phi_rate*error_phi + ki_phi_rate*self.error_phi_integral + kd_phi_rate*phi_derivative

        return max(min_phi_rate, min(phi_rate,max_phi_rate))
        
    def compute_theta_rate(self,desired_theta,actual_theta,delta_t):
        
        error_theta=desired_theta-actual_theta
        
        self.error_theta_integral+=error_theta*delta_t
        self.error_theta_integral=max(-self.max_theta_integral,min(self.error_theta_integral,self.max_theta_integral))
        
        theta_derivative= (self.last_theta-actual_theta)/delta_t #avoid derivative_kick
        self.last_theta=actual_theta
        
        theta_rate= kp_theta_rate*error_theta + ki_theta_rate*self.error_theta_integral + kd_theta_rate*theta_derivative
        
        return max(min_theta_rate, min(theta_rate,max_theta_rate))
    
    def compute_psi_rate(self,desired_psi,actual_psi,delta_t):
        
        error_psi=desired_psi-actual_psi
        
        self.error_psi_integral+=error_psi*delta_t
        self.error_psi_integral=max(-self.max_psi_integral,min(self.error_psi_integral,self.max_psi_integral))
        
        psi_derivative= (self.last_psi-actual_psi)/delta_t #avoid derivative_kick
        self.last_psi=actual_psi
        
        psi_rate= kp_psi_rate*error_psi + ki_psi_rate*self.error_psi_integral + kd_psi_rate*psi_derivative
        
        return max(min_psi_rate, min(psi_rate,max_psi_rate))
    
    
    
    def compute_thrust(self,desired_z,actual_z,delta_t):
        
        error_z=desired_z-actual_z
        
        self.error_z_integral+=error_z*delta_t
        self.error_z_integral=max(-self.max_z_integral,min(self.error_z_integral,self.max_z_integral))
        
        z_derivative= (self.last_z-actual_z)/delta_t #avoid derivative_kick
        self.last_z=actual_z
        
        thrust= kp_thrust*error_z + ki_thrust*self.error_z_integral + kd_thrust*z_derivative
        
        return max(min_thrust, min(thrust,max_thrust))
    
    
    
    
    
    
        



#################################################  ROSNode Function  ##########################################################

def Publish_rateThrust(Thrust,roll_rate,pitch_rate,yaw_rate):
    rate_data=RateThrust()
      
    rate_data.header.stamp = rospy.Time.now()
    rate_data.header.frame_id = "uav/imu"
    rate_data.angular_rates.x=np.float64(roll_rate)
    rate_data.angular_rates.y=np.float64(pitch_rate)
    rate_data.angular_rates.z=np.float64(yaw_rate)
    rate_data.thrust.x=np.float64(0.0)
    rate_data.thrust.x=np.float64(0.0)
    rate_data.thrust.z=np.float64(Thrust)
    
    pub.publish(rate_data)

def uav_groundtruth_pose(tf_data,fused_data):
    
    x=tf_data.transform.translation.x
    y=tf_data.transform.translation.y
    z=tf_data.transform.translation.z
    q1=fused_data.transform.rotation.x
    q2=fused_data.transform.rotation.y
    q3=fused_data.transform.rotation.z
    q4=fused_data.transform.rotation.w
    
    return (x,y,z,q1,q2,q3,q4)


#################################################  ROSNode Main  ##########################################################
if __name__ == '__main__':
    ##### initiate the node and the publisher
    rospy.init_node('angular_control', anonymous=True)
    pub = rospy.Publisher('/uav/input/rateThrust',RateThrust ,queue_size=1)
    ####### initiate tf buffer 
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    
    desired_pose = [pi/9 , 0 , 0 , 2] #pitch =pi/9 , roll= 0 , yaw= 0 , z=2
    desired_phi,desired_theta,desired_psi,desired_z = desired_pose[1],desired_pose[0],desired_pose[2],desired_pose[3]
    controller = AnglesController() # phi_initial=theta_initial=psi_initial=0, z_initial=1

    ##### delta_t parameers
    rate=rospy.Rate(1000)
    delta_t=0.001
    while not rospy.is_shutdown():
        
        ######### lookup for tf data 
        try:

            trans = tfBuffer.lookup_transform("world", 'uav/imu', rospy.Time())
            fused_transform = tfBuffer.lookup_transform("world", 'data_fusion', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        ########### get the tf data
        x,y,z,q1,q2,q3,q4=uav_groundtruth_pose(trans)# *** A faire par Nabil 
        
        ###### transform tf to euler frame
        pitch,roll,yaw=toEulerAngle(q1,q2,q3,q4)
        pitch,roll,yaw=angle_transf(pitch),angle_transf(roll),angle_transf(yaw)
        phi,theta,psi=roll,pitch,yaw
    
        ########### compute controller 
        phi_rate = controller.compute_phi_rate(desired_phi, phi, delta_t)
        theta_rate = controller.compute_theta_rate(desired_theta, theta, delta_t)
        psi_rate = controller.compute_psi_rate(desired_psi, psi, delta_t)
        thrust= controller.compute_thrust(desired_z, z, delta_t)
    
        ############ Convert 
        p, q, r = phidot_thetadot_psidot_to_pqr(phi_rate, theta_rate, psi_rate, theta, phi)
        roll_rate, pitch_rate, yaw_rate = p, q, r

        ############ Publish 
        Publish_rateThrust(thrust,roll_rate,pitch_rate,yaw_rate)

        rate.sleep()
