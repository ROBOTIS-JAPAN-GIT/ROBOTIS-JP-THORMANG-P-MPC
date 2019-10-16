/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
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
 *  joint_module.cpp
 *
 *  Created on: September 12, 2019
 *      Author: Yoshimaru
 */

#include "thormang3_joint_module/joint_module.h"

using namespace thormang3;

JointModule::JointModule()
  : control_cycle_sec_(0.008),
    is_moving_(false),
    arm_angle_display_(false),
    is_dxl_read_(false),
    cnt_(0)
{
  enable_       = false;
  module_name_  = "joint_module";
  control_mode_ = robotis_framework::PositionControl;

  /* arm */
  result_["r_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["r_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"]   = new robotis_framework::DynamixelState();
  result_["torso_y"]      = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id_["r_arm_sh_p1"] = 1;
  joint_name_to_id_["l_arm_sh_p1"] = 2;
  joint_name_to_id_["r_arm_sh_r"]  = 3;
  joint_name_to_id_["l_arm_sh_r"]  = 4;
  joint_name_to_id_["r_arm_sh_p2"] = 5;
  joint_name_to_id_["l_arm_sh_p2"] = 6;
  joint_name_to_id_["r_arm_el_y"]  = 7;
  joint_name_to_id_["l_arm_el_y"]  = 8;
  joint_name_to_id_["r_arm_wr_r"]  = 9;
  joint_name_to_id_["l_arm_wr_r"]  = 10;
  joint_name_to_id_["r_arm_wr_y"]  = 11;
  joint_name_to_id_["l_arm_wr_y"]  = 12;
  joint_name_to_id_["r_arm_wr_p"]  = 13;
  joint_name_to_id_["l_arm_wr_p"]  = 14;
  joint_name_to_id_["torso_y"]     = 27;

  /* etc */
  joint_name_to_id_["r_arm_end"]   = 35;
  joint_name_to_id_["l_arm_end"]   = 34;

  /* parameter */
  present_joint_position_   = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  init_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

}

JointModule::~JointModule()
{
  queue_thread_.join();
}

void JointModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_      = boost::thread(boost::bind(&JointModule::queueThread, this));

  ros::NodeHandle ros_node;

  /* publish topics */

  // for gui
  status_msg_pub_     = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  movement_done_pub_  = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);

}

void JointModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* action server */
  follow_joint_trajectory_action_server_.reset(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(ros_node, "thormang3/right_arm_controller/follow_joint_trajectory", false));
  follow_joint_trajectory_action_server_->registerGoalCallback(boost::bind(&JointModule::followJointTrajectoryActionGoalCallback, this));
  follow_joint_trajectory_action_server_->registerPreemptCallback(boost::bind(&JointModule::followJointTrajectoryActionPreemptCallback, this));
  follow_joint_trajectory_action_server_->start();

  /* subscribe trajectory command topic */
  ros::Subscriber traj_command_sub = ros_node.subscribe("thormang3/trajectory_command", 5,
                                                        &JointModule::trajectoryCommandCallback, this);

  /* ROS Loop */
  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void JointModule::followJointTrajectoryActionPreemptCallback()
{
  follow_joint_trajectory_action_server_->setPreempted();
}

void JointModule::followJointTrajectoryActionGoalCallback()
{
  control_msgs::FollowJointTrajectoryGoalConstPtr goal = follow_joint_trajectory_action_server_->acceptNewGoal();
  if (is_moving_ == false)
    {
      // traj_generate_thread_ = new boost::thread(boost::bind(&JointModule::onJointTrajectory, this, goal->trajectory));
      traj_generate_thread_ = new boost::thread(boost::bind(&JointModule::onFollowJointTrajectory, this, goal->trajectory));
      delete traj_generate_thread_;
    }
  else
    ROS_INFO("previous task is alive");
}

void JointModule::trajectoryCommandCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  if (is_moving_ == false)
    {
      // traj_generate_thread_ = new boost::thread(boost::bind(&JointModule::onJointTrajectory, this, *msg));
      traj_generate_thread_ = new boost::thread(boost::bind(&JointModule::onFollowJointTrajectory, this, *msg));
      delete traj_generate_thread_;
    }
  else
    ROS_INFO("previous task is alive");
}

void JointModule::onFollowJointTrajectory(trajectory_msgs::JointTrajectory trajectory)
{
  /* Initialize */
  all_time_steps_ = 0;
  mov_time_ = 0.0;
  int i = 0, n = 0;
  double prev_time = 0;

  /* Loop */
  while ( i < trajectory.points.size() )
    {
      if (is_moving_ == false)
        {
          trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
          double time_from_start = point.time_from_start.sec + point.time_from_start.nsec * 1e-9;

          double mov_time = time_from_start - prev_time;

          if (mov_time <= 0)
            {
              mov_time = control_cycle_sec_;
            }

          for (int j = 1; j <= MAX_JOINT_ID; j++)
            {
              double ini_pos;
              double tar_pos;

              if (i == 0)
                {
                  // refer from the dxl state
                  ini_pos = goal_joint_position_(j);
                  tar_pos = goal_joint_position_(j);
                }
              else
                {
                  // refer the last value
                  ini_pos = goal_joint_tra_(n-1, j);
                  tar_pos = goal_joint_tra_(n-1, j);
                }
              for (std::vector<std::string>::iterator iter = trajectory.joint_names.begin(); iter != trajectory.joint_names.end(); iter++)
                {
                  if (joint_name_to_id_[iter->c_str()] == j)
                    {
                      size_t iter_index = std::distance(trajectory.joint_names.begin(), iter);
                      tar_pos = double(point.positions[iter_index]);
                    }
                }

              // TODO: use velocity and acceleration from the follow joint trajectory msg.
              Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_pos, 0.0, 0.0,
                                                                          tar_pos, 0.0, 0.0,
                                                                          control_cycle_sec_,
                                                                          mov_time);

              if (j == 1)
                {
                  all_time_steps_ += tra.rows();
                  goal_joint_tra_.conservativeResize(all_time_steps_, MAX_JOINT_ID + 1);
                }
              // ROS_INFO("n : %d, j : %d, tra.rows() : %d, tra.cols() : %d", n, j, tra.rows(), tra.cols());
              goal_joint_tra_.block(n, j, tra.rows(), tra.cols()) = tra;

            }

          n = goal_joint_tra_.rows();
          prev_time = time_from_start;
          i++;

        }
    }

  cnt_ = 0;
  is_moving_ = true;

}

void JointModule::onJointTrajectory(trajectory_msgs::JointTrajectory trajectory)
{
  // process_mutex_.lock();

  ROS_INFO("received trajectory command.");

  // local param
  int i = 0;
  double prev_time = 0;
  all_time_steps_ = 0;
  mov_time_ = 0;

  ROS_INFO("MAX_JOINT_ID : %d", MAX_JOINT_ID);
  // while ( enable_ && i < trajectory.points.size() ) // check life of module && ros::ok()

  while ( i < trajectory.points.size() ) // check life of module && ros::ok()
    {
      // ROS_INFO("start of while # : %d", i);
      // if (is_moving_ == false && cnt_ == 0)
      // if (is_moving_ == false && std::isnan(goal_joint_position_(1)) == false)
      // if (is_moving_ == false)
      if (is_moving_ == false)
        {
          // ROS_INFO("Entered if.");

          trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
          double time_from_start = point.time_from_start.sec + point.time_from_start.nsec * 1e-9;
          ROS_INFO("time_from_start : %f, prev_time : %f", time_from_start, prev_time);

          if (time_from_start <= 0.0)
            {
              mov_time_ = 0; // minimum
              all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
            }
          else
            {
              mov_time_ = time_from_start - prev_time;
              all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
            }

          ROS_INFO("resize.");
          goal_joint_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);

          for (int j = 1; j <= MAX_JOINT_ID; j++)
            {
              // ROS_INFO("for # : %d", j);
              double ini_pos = goal_joint_position_(j); // if failed, resize also fails
              double tar_pos = goal_joint_position_(j);
              ROS_INFO("goal_joint_position_ : %f", tar_pos);
              for (std::vector<std::string>::iterator iter = trajectory.joint_names.begin(); iter != trajectory.joint_names.end(); iter++)
                {
                  // ROS_INFO("joint name : %s", iter->c_str());
                  if (joint_name_to_id_[iter->c_str()] == j)
                    {
                      size_t iter_index = std::distance(trajectory.joint_names.begin(), iter);
                      tar_pos = double(point.positions[iter_index]);
                      ROS_INFO("joint name : %s, value : %f", iter->c_str(), tar_pos);
                    }
                }

              Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_pos, 0.0, 0.0,
                                                                          tar_pos, 0.0, 0.0,
                                                                          control_cycle_sec_,
                                                                          mov_time_);
              ROS_INFO("No. %d", j);
              goal_joint_tra_.block(0, j, all_time_steps_, 1) = tra; // error
              // ROS_INFO("Inserted.");
            }

          ROS_INFO("end of if : %d", i);
          is_dxl_read_ = false;
          is_moving_ = true;
          cnt_ = 0;

          prev_time = time_from_start;
          i++;

        }
      else
        continue;
    }

  ROS_INFO("Sending trajectory DONE.");

  // process_mutex_.unlock();
}

void JointModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                          std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    present_joint_position_(joint_name_to_id_[joint_name]) = joint_curr_position;
    goal_joint_position_(joint_name_to_id_[joint_name]) = joint_goal_position;

  }

  /* ----- send trajectory ----- */
  if (is_moving_ == true)
    {
      for (int id = 1; id <= MAX_JOINT_ID; id++)
        goal_joint_position_(id) = goal_joint_tra_(cnt_, id);

      cnt_++;
    }

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
  }

  /* TODO: action feedback */
  ros::Time tm_on_execute = ros::Time::now();
  // trajectory_msgs::JointTrajectoryPoint commanded_joint_trajectory_point, error_joint_trajectory_point;
  // if ( follow_joint_trajectory_action_server_->isActive() ) {
  //   control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_action_feedback;
  //   follow_joint_trajectory_action_feedback.header.stamp = tm_on_execute;
  //   follow_joint_trajectory_action_feedback.joint_names = joint_list;
  //   follow_joint_trajectory_action_feedback.desired = commanded_joint_trajectory_point;
  //   follow_joint_trajectory_action_feedback.actual  = commanded_joint_trajectory_point;
  //   follow_joint_trajectory_action_feedback.error   = error_joint_trajectory_point;
  //   follow_joint_trajectory_action_server_->publishFeedback(follow_joint_trajectory_action_feedback);
  // }

  /*---------- initialize count number ----------*/
  if (is_moving_ == true)
  {
    if (cnt_ >= all_time_steps_)
    {
      ROS_INFO("[end] send trajectory");

      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      is_moving_  = false;
      cnt_        = 0;

      movement_done_pub_.publish(movement_done_msg_);
      movement_done_msg_.data = "";

      /* action result */
      if ( follow_joint_trajectory_action_server_->isActive() )
        {
          control_msgs::FollowJointTrajectoryResult result;
          result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
          follow_joint_trajectory_action_server_->setSucceeded(result);
        }

      if (arm_angle_display_ == true)
      {
        ROS_INFO("l_arm_sh_p1 : %f", goal_joint_position_(joint_name_to_id_["l_arm_sh_p1"]) * RADIAN2DEGREE );
        ROS_INFO("l_arm_sh_r  : %f", goal_joint_position_(joint_name_to_id_["l_arm_sh_r"])  * RADIAN2DEGREE );
        ROS_INFO("l_arm_sh_p2 : %f", goal_joint_position_(joint_name_to_id_["l_arm_sh_p2"]) * RADIAN2DEGREE );
        ROS_INFO("l_arm_el_y  : %f", goal_joint_position_(joint_name_to_id_["l_arm_el_y"])  * RADIAN2DEGREE );
        ROS_INFO("l_arm_wr_r  : %f", goal_joint_position_(joint_name_to_id_["l_arm_wr_r"])  * RADIAN2DEGREE );
        ROS_INFO("l_arm_wr_y  : %f", goal_joint_position_(joint_name_to_id_["l_arm_wr_y"])  * RADIAN2DEGREE );
        ROS_INFO("l_arm_wr_p  : %f", goal_joint_position_(joint_name_to_id_["l_arm_wr_p"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_sh_p1 : %f", goal_joint_position_(joint_name_to_id_["r_arm_sh_p1"]) * RADIAN2DEGREE );
        ROS_INFO("r_arm_sh_r  : %f", goal_joint_position_(joint_name_to_id_["r_arm_sh_r"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_sh_p2 : %f", goal_joint_position_(joint_name_to_id_["r_arm_sh_p2"]) * RADIAN2DEGREE );
        ROS_INFO("r_arm_el_y  : %f", goal_joint_position_(joint_name_to_id_["r_arm_el_y"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_wr_r  : %f", goal_joint_position_(joint_name_to_id_["r_arm_wr_r"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_wr_y  : %f", goal_joint_position_(joint_name_to_id_["r_arm_wr_y"])  * RADIAN2DEGREE );
        ROS_INFO("r_arm_wr_p  : %f", goal_joint_position_(joint_name_to_id_["r_arm_wr_p"])  * RADIAN2DEGREE );
      }

    }
  }

}

void JointModule::stop()
{
  is_moving_  = false;
  cnt_        = 0;

  return;
}

bool JointModule::isRunning()
{
  return is_moving_;
}

void JointModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Joint";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}
