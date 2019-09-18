/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 *  joint_module.h
 *
 *  Created on: September 12, 2019
 *      Author: Yoshimaru
 */

#ifndef THORMANG3_JOINT_MODULE_JOINT_MODULE_H_
#define THORMANG3_JOINT_MODULE_JOINT_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
// #include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/JointCtrlModule.h" // necessary?
#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h" // MAX_JOINT_ID

/* action */
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace thormang3
{

class JointModule: public robotis_framework::MotionModule,
                   public robotis_framework::Singleton<JointModule>
{
public:
  JointModule();
  virtual ~JointModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  void publishStatusMsg(unsigned int type, std::string msg);

  void followJointTrajectoryActionPreemptCallback();
  void followJointTrajectoryActionGoalCallback();
  void trajectoryCommandCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg);
  void onJointTrajectory(trajectory_msgs::JointTrajectory trajectory);

  /* Parameter */
  // KinematicsDynamics *robotis_;

private:
  void queueThread();

  void parseData(const std::string &path);

  bool arm_angle_display_;

  double          control_cycle_sec_;
  boost::thread   queue_thread_;
  boost::thread   *traj_generate_thread_;

  // boost::mutex process_mutex_; // added

  std_msgs::String movement_done_msg_;

  ros::Publisher  status_msg_pub_;
  ros::Publisher  movement_done_pub_;

  /* action */
  boost::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> >follow_joint_trajectory_action_server_;
  control_msgs::FollowJointTrajectoryActionFeedback feedback;
  control_msgs::FollowJointTrajectoryActionResult result;

  /* joint state */
  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd goal_joint_position_;
  Eigen::VectorXd init_joint_position_;

  /* trajectory */
  bool    is_moving_;
  double  mov_time_;
  int     cnt_;
  int     all_time_steps_;

  Eigen::MatrixXd goal_joint_tra_;

  std::map<std::string, int> joint_name_to_id_;
};

}

#endif /* THORMANG3_JOINT_MODULE_JOINT_MODULE_H_ */
