/*!*******************************************************************************************
 *  \file       behavior_pay_attention_to_robot_messages.cpp
 *  \brief      Behavior Pay Attention To Robot Messages implementation file.
 *  \details    This file implements the BehaviorPayAttentionToRobotMessages class.
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

#include "../include/behavior_pay_attention_to_robot_messages.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace swarm_interaction
{
BehaviorPayAttentionToRobotMessages::BehaviorPayAttentionToRobotMessages() : BehaviorExecutionController() {
 setName("pay_attention_to_robot_messages"); 
 setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorPayAttentionToRobotMessages::~BehaviorPayAttentionToRobotMessages() {}

bool BehaviorPayAttentionToRobotMessages::checkSituation() 
{ 
 return true; 
}

void BehaviorPayAttentionToRobotMessages::checkGoal() 
{ 

}

void BehaviorPayAttentionToRobotMessages::checkProgress() 
{ 
 
}

void BehaviorPayAttentionToRobotMessages::checkProcesses() 
{ 
 
}

void BehaviorPayAttentionToRobotMessages::onConfigure()
{
  nh = getNodeHandle();
  nh.param<std::string>("social_communication_channel_topic", social_communication_channel_str,
                                 "social_communication_channel");
  nh.param<std::string>("message_from_robot", message_from_robot,
                                 "message_from_robot");

      // Activate communications
  social_communication_channel_sub =
      nh.subscribe('/' + social_communication_channel_str, 500,
                            &BehaviorPayAttentionToRobotMessages::socialCommunicationChannelCallback, this);


  message_from_robot_pub=nh.advertise<aerostack_msgs::SocialCommunicationStatement>('/' + message_from_robot, 1000);


}

void BehaviorPayAttentionToRobotMessages::onActivate()
{

}

void BehaviorPayAttentionToRobotMessages::onDeactivate()
{
 //social_communication_channel_sub.shutdown();
 //add_belief_srv.shutdown();
}

void BehaviorPayAttentionToRobotMessages::onExecute()
{
  
}

void BehaviorPayAttentionToRobotMessages::socialCommunicationChannelCallback(
    const aerostack_msgs::SocialCommunicationStatement &message)
{
	
  if ((message.sender != getNamespace()) && (message.receiver == getNamespace()))
  {
     aerostack_msgs::SocialCommunicationStatement msg;
     msg.sender=message.sender;
     msg.receiver=message.receiver;
     msg.content=message.content;
     message_from_robot_pub.publish(msg);
  }
  

}



}
PLUGINLIB_EXPORT_CLASS(swarm_interaction::BehaviorPayAttentionToRobotMessages, nodelet::Nodelet)
