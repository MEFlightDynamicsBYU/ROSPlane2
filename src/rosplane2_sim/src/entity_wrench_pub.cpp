/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
 *
*/

#include <iostream>
#include <string>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/wrench.pb.h>
#include <ignition/msgs/entity_wrench.pb.h>
#include <ignition/transport.hh>


//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const ignition::msgs::Wrench &wrench_)
{

  ignition::transport::Node publisher;
  std::string topic_pub = "/world/rosplane2/wrench";

  auto pub = publisher.Advertise<ignition::msgs::EntityWrench>(topic_pub);

  if (!pub)
  {
    std::cerr << "Error advertising topic [" << topic_pub << "]" << std::endl;
    return;
  }

  ignition::msgs::Entity en;
  std::string link_name = "fixedwing::link";
  en.set_name(link_name);
  en.set_type(ignition::msgs::Entity_Type_LINK);


  ignition::msgs::Wrench wrench = wrench_;
  
  ignition::msgs::EntityWrench ew;
  ew.set_allocated_entity(&en);
  ew.set_allocated_wrench(&wrench);


  pub.Publish(ew);
  std::cout <<std::endl<< std::endl<<"HERE!!!" << std::endl<< std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Create a transport node and advertise a topic.
  gz::transport::Node subscriber;
  const std::string topic_sub = "/forces_moments";
  

  // Subscribe to a topic by registering a callback.
  if (!subscriber.Subscribe(topic_sub, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }

  // Zzzzzz.
  gz::transport::waitForShutdown();

  return 0;
}