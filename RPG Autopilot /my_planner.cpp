#include <gtest/gtest.h>
#include <vector>

#include <autopilot/autopilot_states.h>
#include <Eigen/Dense>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

#include <autopilot/autopilot_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/ros.h>

#include <gtest/gtest.h>


int main(int argc, char **argv)
{
    static constexpr double kExecLoopRate_ = 50.0;
    ros::init(argc, argv, "my_planner");
    ros::NodeHandle nh;

    ros::Publisher arm_pub_;
    arm_pub_ = nh.advertise<std_msgs::Bool>("bridge/arm", 1);

    autopilot_helper::AutoPilotHelper autopilot_helper_;

      // Make sure everything is up and running
      // Wait for Autopilot feedback with assert
    //ASSERT_TRUE(autopilot_helper_.waitForAutopilotFeedback(10.0, kExecLoopRate_)) 
    //    << "Did not receive autopilot feedback within 10 seconds.";
        autopilot_helper_.waitForAutopilotFeedback(10.0, kExecLoopRate_);
    ros::Duration(3.0).sleep();

    // Arm bridge
    std_msgs::Bool arm_msg;
    arm_msg.data = true;
    arm_pub_.publish(arm_msg);

    ///////////////
    // Check off command
    ///////////////

     // Takeoff
    autopilot_helper_.sendStart();

      // Wait for autopilot to go to start
    //EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::START, 0.5,
    //kExecLoopRate_))
    //  << "Autopilot did not switch to start after sending start command.";
    autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::START, 0.5, kExecLoopRate_);

  // Abort start and send off
  autopilot_helper_.sendOff();

  // Wait for autopilot to go to off
  //EXPECT_TRUE(
    //  autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::OFF, 0.1,
    //      kExecLoopRate_))
    //  << "Autopilot could not be forced to off during take off.";

  ///////////////
  // Check take off
  ///////////////

  // Takeoff for real
  autopilot_helper_.sendStart();

  // Wait for autopilot to go to hover
  //EXPECT_TRUE(
    //  autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0,
    //      kExecLoopRate_))
    //  << "Autopilot did not switch to hover after take off.";
    autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0, kExecLoopRate_);
  ///////////////
  // Check go to pose
  ///////////////
 while (ros::ok())
 {
  // Send pose command
  const Eigen::Vector3d position_cmd = Eigen::Vector3d(0.0, 0.0, 1.0);
  const double heading_cmd = 0.0;

  autopilot_helper_.sendPoseCommand(position_cmd, heading_cmd);
  
  // Wait for autopilot to go to got to pose state
  //EXPECT_TRUE(
    //  autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::TRAJECTORY_CONTROL, 2.0,
    //      kExecLoopRate_))
    //  << "Autopilot did not switch to trajectory control because of go to pose action correctly.";
        autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_);

  // Wait for autopilot to go back to hover
  //EXPECT_TRUE(
    //  autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0,
    //      kExecLoopRate_))
    //  << "Autopilot did not switch back to hover correctly.";
        autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0, kExecLoopRate_);

        ros::Duration(1.0).sleep();
 } 

return 0;
}