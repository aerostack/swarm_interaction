/*!*******************************************************************************************
 *  \file       behavior_inform_robots.h
 *  \brief      behavior inform robots definition file.
 *  \details     This file contains the BehaviorTrakeOff declaration. To obtain more information about
 *              it's definition consult the behavior_rotate.cpp file.
 *  \authors    Alberto Camporredondo
 *  \copyright  Copyright (c) 2018 Universidad Politecnica de Madrid
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
*********************************************************************************************/

#ifndef BEHAVIOR_INFORM_ROBOTS_H
#define BEHAVIOR_INFORM_ROBOTS_H

// System
#include <string>
#include <thread>
#include <tuple>
// ROS
#include "std_srvs/Empty.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// Aerostack msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/SocialCommunicationStatement.h>
// Aerostack libraries
#include <behavior_execution_controller.h>
#include <yaml-cpp/yaml.h>
namespace swarm_interaction
{
class BehaviorInformRobots : public BehaviorExecutionController
{
  // Constructor
public:
  BehaviorInformRobots();
  ~BehaviorInformRobots();

 struct Behavior
  {
    std::string name;
    std::string arguments;
  };

private:


  // Congfig variables
  std::string social_communication_channel_str;
  std::string nspace; 
  ros::NodeHandle nh;

  // Communication variables
  ros::Publisher social_communication_channel_pub;


private:
  // BehaviorExecutionController
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

};
}

#endif
