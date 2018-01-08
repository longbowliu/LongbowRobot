/*
 * RobotTrajectoryFollower.cpp
 *
 *  Created on: Jan 7, 2018
 *      Author: llb
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include <map>
#include "nogrp_moveit_config/joint_msg.h"

using namespace std ;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class RobotTrajectoryFollower
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
 // actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;

  ros::Publisher joint_pub ;
  ros::Publisher joint_motor_pub ;
  sensor_msgs::JointState joint_state;
  nogrp_moveit_config::joint_msg joint_motor_msg;

  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryActionGoal agoal_;
  control_msgs::FollowJointTrajectoryActionFeedback afeedback_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
public:
  map< string, int > MotoName_Id;
  RobotTrajectoryFollower(std::string name) :
    nh_("~"),
    as_(nh_, name,boost::bind(&RobotTrajectoryFollower::goalCB, this, _1),  false),
    action_name_(name)
  {
    /*<!--
         - arm_base_to_arm_round_joint_stevo0
      - shoulder_2_to_arm_joint_stevo1
      - big_arm_round_to_joint_stevo2
      - arm_joint_stevo2_to_arm_joint_stevo3
      - wrist_to_arm_joint_stevo4
      - arm_joint_stevo4_to_arm_joint_stevo5

     -->*/
    nh_.param("arm_base_to_arm_round_joint_stevo0", MotoName_Id["arm_base_to_arm_round_joint_stevo0"], 0);
    nh_.param("shoulder_2_to_arm_joint_stevo1", MotoName_Id["shoulder_2_to_arm_joint_stevo1"], 0);
    nh_.param("big_arm_round_to_joint_stevo2", MotoName_Id["big_arm_round_to_joint_stevo2"], 0);
    nh_.param("arm_joint_stevo2_to_arm_joint_stevo3", MotoName_Id["arm_joint_stevo2_to_arm_joint_stevo3"], 0);
    nh_.param("wrist_to_arm_joint_stevo4", MotoName_Id["wrist_to_arm_joint_stevo4"], 0);
    nh_.param("arm_joint_stevo4_to_arm_joint_stevo5", MotoName_Id["arm_joint_stevo4_to_arm_joint_stevo5"], 10);

    joint_pub = nh_.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);

    joint_motor_pub = nh_.advertise<nogrp_moveit_config::joint_msg>("/arm_motors", 1000);

    //Register callback functions:
    //as_.registerGoalCallback(boost::bind(&RobotTrajectoryFollower::goalCB, this,_1));
    as_.registerPreemptCallback(boost::bind(&RobotTrajectoryFollower::preemptCB, this));

    as_.start();
  }

  ~RobotTrajectoryFollower(void)//Destructor
  {
  }
//
  void setJointStateName(std::vector<std::string> joint_names){
    joint_state.name.resize(joint_names.size());
    joint_state.name.assign(joint_names.begin(), joint_names.end());
    //joint_state.name[0] ="arm_1_to_arm_base";
    //std::vector<std::string>::iterator it;
    //for ( it = joint_state.name.begin(); it != joint_state.name.end(); it++){
    //  cout <<(*it) <<endl;
    //}
    //cout <<endl ;
  }
  void setJointStatePosition(std::vector<double> joint_posi){
    joint_state.position.resize(joint_posi.size());
    joint_state.position.assign(joint_posi.begin(), joint_posi.end());
    //joint_state.position[0] = base_arm;
    //std::vector<double>::iterator it;
    //for ( it = joint_state.position.begin(); it != joint_state.position.end(); it++){
    //  cout <<(*it) <<endl;
    //}
    //cout <<endl ;
  }
  void publishJointState(){
    joint_state.header.stamp = ros::Time::now();
    joint_pub.publish(joint_state);
  }
  void publishMotorState(int ids[],std::vector<double> joint_posi){


    std::vector<double>::iterator it;
    int i=0;
    for ( it = joint_posi.begin(); it != joint_posi.end(); it++,i++){
      joint_motor_msg.id=ids[i];
      joint_motor_msg.r=(*it) ;
      joint_motor_pub.publish(joint_motor_msg);
      cout <<joint_motor_msg <<endl;

    }


  }
  void goalCB(const control_msgs::FollowJointTrajectoryGoalConstPtr msg)
  {
    //cout<<((*msg))<<endl;
    std::vector<std::string> joint_names=(*msg).trajectory.joint_names;
    //
    setJointStateName( joint_names);

    std::vector<std::string>::iterator it;
    int ids [joint_names.size()];
    int i=0;
    for ( it = joint_names.begin(); it != joint_names.end(); it++,i++){
      ids[i]=MotoName_Id[(*it)];
      cout <<MotoName_Id[(*it)] <<endl;
    }

    //goal=(*msg);goal.trajectory.points;//c++ how to use this style??

    std::vector<trajectory_msgs::JointTrajectoryPoint> points = (*msg).trajectory.points;
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator pointsit;
    ros::Rate rate(10);//10hz
    size_t t=points.size();
    ROS_INFO("%s: goalCB ", action_name_.c_str());
    for ( pointsit = points.begin(); pointsit != points.end(); pointsit++){
      //cout<<(*pointsit)<<endl;
      //cout <<endl ;
      //here send datamsg to hardware node,command motors run.
      //
      publishMotorState(ids,(*pointsit).positions);
      //wait
      rate.sleep();
      //then update joinstates an publish
      setJointStatePosition((*pointsit).positions);
      publishJointState();
      //feedback_.
      //as_.publishFeedback(feedback_);
      ROS_INFO("left position :%d", (int)t);
      t--;
    }

    // accept the new goal
    //as_.acceptNewGoal();
    if(as_.isActive())as_.setSucceeded();
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted

    if(as_.isActive()){
        as_.setPreempted();
    }
  }
  Server as_;
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "jointcontroller");
  RobotTrajectoryFollower RobotTrajectoryFollower("/arm_controller/follow_joint_trajectory");

;
  ROS_INFO("-------------zzz joint controller is running .");
  ros::spin();

  return 0;
}

