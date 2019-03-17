#!/usr/bin/env python
# coding: utf-8
# license removed for brevity
# In[1]:


#############################################       Dependencies       ###################################################
from math import *
from time import time,sleep
import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
##############################################      Hyperparameters    ###################################################
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


max_yaw_rate= pi/4 # rad/s
min_yaw_rate=-max_yaw_rate
max_psi_rate= max_yaw_rate
min_psi_rate=-max_psi_rate

max_thrust= 37 # 4*thrust_coeff*max_popr_speed**2
min_thrust= 0



##### PIDS
#PID theta_rate
kp_theta_rate,kd_theta_rate,ki_theta_rate=(100,0,0.1)

#PID phi_rate
kp_phi_rate,kd_phi_rate,ki_phi_rate=(100,0,0.1)

#PID psi_rate
kp_psi_rate,kd_psi_rate,ki_psi_rate=(1,0,0.1)

#PID thrust
kp_thrust,kd_thrust,ki_thrust=(50,10,10)

##### Nodes handler
rospy.init_node('/control/angular_control', anonymous=True)
pub = rospy.Publisher('/uav/input/rateThrust',RateThrust ,queue_size=1)

############################################      Utils functions     #####################################################

def pqr_to_phidot_thetadot_psidot(p,q,r,theta,phi):
    
    phi_dot= p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
    theta_dot= q*cos(phi) - r*sin(phi)
    psi_dot= q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta) 
    
    return(phi_dot,theta_dot,psi_dot)

def toEulerAngle(q1, q2, q3, q4):
    
    roll = atan2(2*(q1*q2+q3*q4), 1-2*(q2**2+q3**2))
    pitch = asin(2*(q1*q3-q2*q4))
    yaw = atan2(2*(q1*q4+q2*q3), 1-2*(q3**2+q4**2))
    
    return (roll, pitch, yaw)

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
        
        


# In[114]:


############################################ Quadcopter Controller class #################################################
class AnglesController:
    
    def __init__(self):

        
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
    
 
    def compute_phi_rate(self,desired_phi,actual_phi,delta_t):

        error_phi=desired_phi-actual_phi
        
        self.error_phi_integral+=error_phi*delta_t
        self.error_phi_integral=min(self.error_phi_integral,self.max_phi_integral)

        phi_derivative= (self.last_phi-actual_phi)/delta_t #avoid derivative_kick
        self.last_phi=actual_phi

        phi_rate= kp_phi_rate*error_phi + ki_phi_rate*self.error_phi_integral + kd_phi_rate*phi_derivative

        return max(min_phi_rate, min(phi_rate,max_phi_rate))
        
    def compute_theta_rate(self,desired_theta,actual_theta,delta_t):
        
        error_theta=desired_theta-actual_theta
        
        self.error_theta_integral+=error_theta*delta_t
        self.error_theta_integral=min(self.error_theta_integral,self.max_theta_integral)
        
        theta_derivative= (self.last_theta-actual_theta)/delta_t #avoid derivative_kick
        self.last_theta=actual_theta
        
        theta_rate= kp_theta_rate*error_theta + ki_theta_rate*self.error_theta_integral + kd_theta_rate*theta_derivative
        
        return max(min_theta_rate, min(theta_rate,max_theta_rate))
    
    def compute_psi_rate(self,desired_psi,actual_psi,delta_t):
        
        error_psi=desired_psi-actual_psi
        
        self.error_psi_integral+=error_psi*delta_t
        self.error_psi_integral=min(self.error_psi_integral,self.max_psi_integral)
        
        psi_derivative= (self.last_psi-actual_psi)/delta_t #avoid derivative_kick
        self.last_psi=actual_psi
        
        psi_rate= kp_psi_rate*error_psi + ki_psi_rate*self.error_psi_integral + kd_psi_rate*psi_derivative
        
        return max(min_psi_rate, min(psi_rate,max_psi_rate))
    
    
    
    def compute_thrust(self,desired_z,actual_z,delta_t):
        
        error_z=desired_z-actual_z
        
        self.error_z_integral+=error_z*delta_t
        self.error_z_integral=min(self.error_z_integral,self.max_z_integral)
        
        z_derivative= (self.last_z-actual_z)/delta_t #avoid derivative_kick
        self.last_z=actual_z
        
        thrust= kp_thrust*error_z + ki_thrust*self.error_z_integral + kd_thrust*z_derivative
        
        return max(min_thrust, min(thrust,max_thrust))
    
    
    
    
    
    
        


# In[ ]:


#################################################  ROSNode Function  ##########################################################
def angular_control():
    desired_pose = [pi/9 , 0 , 0 , 2] #pitch =pi/9 , roll= 0 , yaw= 0 , z=2
    desired_phi,desired_theta,desired_psi,desired_z = desired_pose[1],desired_pose[0],desired_pose[2],desired_pose[3]


    controller = AnglesController() # phi_initial=theta_initial=psi_initial=0, z_initial=1

    tic=time()
    sleep(1) #to avoid zero_division_error in the first loop

    while not rospy.is_shutdown():
        rospy.Subscriber("/tf", TFMessage, callback)


def callback(data):## Function that get pose as data 
    x,y,z,q1,q2,q3,q4=uav_groundtruth_pose()# *** A faire par Nabil 
    
    pitch,roll,yaw=toEulerAngle(q1,q2,q3,q4)
    pitch,roll,yaw=angle_transf(pitch),angle_transf(roll),angle_transf(yaw)
    phi,theta,psi=roll,pitch,yaw
    
    toc=time()
    delta_t=toc-tic
    
    phi_rate = controller.compute_phi_rate(desired_phi, phi, delta_t)
    theta_rate = controller.compute_theta_rate(desired_theta, theta, delta_t)
    psi_rate = controller.compute_psi_rate(desired_psi, psi, delta_t)
    thrust= controller.compute_thrust(desired_z, z, delta_t)
    
    tic=time()

    p, q, r = phidot_thetadot_psidot_to_pqr(phi_rate, theta_rate, psi_rate, theta, phi)
    roll_rate, pitch_rate, yaw_rate = p, q, r

    ############Publish 
    Publish_RateThrust_msg(Thrust,roll_rate,pitch_rate,yaw_rate) # *** A faire par Nabil
        

#################################################  ROSNode Main  ##########################################################
if __name__ == '__main__':
   try:
      angular_control()

   except rospy.ROSInterruptException:
      pass


# In[ ]:





# In[ ]:





# In[ ]:




