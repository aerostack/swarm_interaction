/*!*******************************************************************************************
 *  \file       behavior_inform_position_to_robots.cpp
 *  \brief      Behavior inform position to robots implementation file.
 *  \details    This file implements the BehaviorInformRobots class.
 *  \authors    Javier Cabrera Marugan
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All Rights Reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "../include/behavior_inform_position_to_robots.h"
#include <pluginlib/class_list_macros.h>

namespace swarm_interaction
{
BehaviorInformPositionToRobots::BehaviorInformPositionToRobots() : BehaviorExecutionController() 
{ 
  setName("inform_position_to_robots"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorInformPositionToRobots::~BehaviorInformPositionToRobots() {}

void BehaviorInformPositionToRobots::onConfigure()
{ 
  //nspace = getNamespace(); 
  nh = getNodeHandle();
  nh.param<std::string>("shared_robot_positions_channel_topic", shared_robot_positions_channel_str,
                                 "shared_robot_positions_channel");
  nh.param<std::string>("pose_topic", pose_topic, "self_localization/pose");
 

  pose_subscriber = nh.subscribe("/" + getNamespace() + "/"+ pose_topic, 1, &BehaviorInformPositionToRobots::poseCallback, this);
  shared_robot_positions_channel_pub = nh.advertise<aerostack_msgs::SharedRobotPosition>('/' + shared_robot_positions_channel_str, 1000);
  numberOfStepsBeforePublished=0;

}

void BehaviorInformPositionToRobots::onActivate()
{
 
}

void BehaviorInformPositionToRobots::onDeactivate()
{
 //social_communication_channel_pub.shutdown();
}

void BehaviorInformPositionToRobots::poseCallback(const geometry_msgs::PoseStamped& pos)
{
  x=pos.pose.position.x;
  y=pos.pose.position.y;
  z=pos.pose.position.z;
  time=pos.header.stamp.sec;
}

void BehaviorInformPositionToRobots::onExecute()
{
  numberOfStepsBeforePublished=numberOfStepsBeforePublished+1;
  if (numberOfStepsBeforePublished>=40){
    geometry_msgs::Point position;
    position.x=x;
    position.y=y;
    position.z=z;
    
    aerostack_msgs::SharedRobotPosition message;
    message.sender = getNamespace();
    message.position = position;
    message.time = time;
    
    shared_robot_positions_channel_pub.publish(message);
    numberOfStepsBeforePublished=0;

  }

}

bool BehaviorInformPositionToRobots::checkSituation()
{
  return true;
}

void BehaviorInformPositionToRobots::checkGoal()
{
  
}

void BehaviorInformPositionToRobots::checkProgress() 
{ 
 
}


void BehaviorInformPositionToRobots::checkProcesses() 
{ 
 
}


}
PLUGINLIB_EXPORT_CLASS(swarm_interaction::BehaviorInformPositionToRobots, nodelet::Nodelet)
