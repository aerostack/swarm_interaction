/*!*******************************************************************************************
 *  \file       behavior_inform_robots.cpp
 *  \brief      Behavior inform robots implementation file.
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

#include "../include/behavior_inform_robots.h"
#include <pluginlib/class_list_macros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorInformRobots behavior;
  behavior.start();
  return 0;
}

BehaviorInformRobots::BehaviorInformRobots() : BehaviorExecutionManager() 
{ 
  setName("inform_robots"); 
}

BehaviorInformRobots::~BehaviorInformRobots() {}

void BehaviorInformRobots::onConfigure()
{ 
  nspace = getNamespace(); 
  nh = getNodeHandle();
  nh.param<std::string>("social_communication_channel_topic", social_communication_channel_str,
                                 "social_communication_channel");
  social_communication_channel_pub = nh.advertise<aerostack_msgs::SocialCommunicationStatement>('/'
   + social_communication_channel_str, 1000);
}

void BehaviorInformRobots::onActivate()
{
  // Behavior implementation
  std::string args = getParameters();

  YAML::Node node = YAML::Load(args);

  std::string destination = node["receiver"].as<std::string>();
  std::string text = node["message"].as<std::string>();

  aerostack_msgs::SocialCommunicationStatement message;
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "TEXT";
  out << YAML::Value << text;
  out << YAML::EndMap;

  message.sender = getNamespace();
  message.receiver = destination;
  message.content = out.c_str();

  social_communication_channel_pub.publish(message);
  BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
}

void BehaviorInformRobots::onDeactivate()
{
 //social_communication_channel_pub.shutdown();
}

void BehaviorInformRobots::onExecute()
{
  

}

bool BehaviorInformRobots::checkSituation()
{
  return true;
}

void BehaviorInformRobots::checkGoal()
{
  
}

void BehaviorInformRobots::checkProgress() 
{ 
 
}


void BehaviorInformRobots::checkProcesses() 
{ 
 
}
