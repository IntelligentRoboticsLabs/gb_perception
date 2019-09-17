/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: David Vargas dvargasfr@gmail.com */

#include "qr_perception.h"

QR_perception::QR_perception(const ros::NodeHandle& nh) : nh_(nh)
{
  qr_reader_sub_ = nh_.subscribe("/barcode", 1, &QR_perception::qrCallback, this);
  timer_recognized = nh_.createTimer(ros::Duration(2.0), &QR_perception::timerCallback2order, this, true);
  timer_order = nh_.createTimer(ros::Duration(3.0), &QR_perception::timerCallback2standby, this, true);
  timer_recognized.stop();
  timer_order.stop();
  ROS_INFO("########### state_ = STAND_BY");
  state_ = STAND_BY;
}

void QR_perception::qrCallback(const std_msgs::String::ConstPtr& qr_msg)
{
  switch(state_){
    case STAND_BY:
      QRonStandByState(qr_msg);
      break;
    case RECOGNIZED:
      QRonRecogState(qr_msg);
      break;
    case ORDER:
      QRonOrderState(qr_msg);
      break;
  }
}

void QR_perception::QRonStandByState(const std_msgs::String::ConstPtr& qr_msg)
{
  ROS_INFO("########### state_ = RECOGNIZED");
  state_ = RECOGNIZED;
  timer_recognized.start();
  last_response_ = qr_msg->data.c_str();
  ROS_INFO("########### I saw a new msg! [%s]", qr_msg->data.c_str());
  order.push_back(last_response_);
}

void QR_perception::QRonRecogState(const std_msgs::String::ConstPtr& qr_msg)
{
  timer_recognized.stop();
  if (qr_msg->data.c_str() != last_response_)
  {
    last_response_ = qr_msg->data.c_str();
    ROS_INFO("########### I saw a new msg! [%s]", qr_msg->data.c_str());
    order.push_back(last_response_);
  }
  timer_recognized.start();
}

void QR_perception::QRonOrderState(const std_msgs::String::ConstPtr& qr_msg)
{
  timer_order.stop();
  ROS_INFO("########### state_ = RECOGNIZED");
  state_ = RECOGNIZED;
  last_response_ = qr_msg->data.c_str();
  ROS_INFO("########### I saw a new msg! [%s]", qr_msg->data.c_str());
  order.push_back(last_response_);
}

void QR_perception::timerCallback2order(const ros::TimerEvent&)
{
  ROS_INFO("########### state_ = ORDER");
  state_ = ORDER;
  timer_order.start();
}

void QR_perception::timerCallback2standby(const ros::TimerEvent&)
{
  ROS_INFO("########### state_ = STAND_BY");
  state_ = STAND_BY;

  std::vector<bica_graph::StringEdge> ask_edges = graph_.get_string_edges_by_data("ask: [[:alnum:]_[:punct:]]*");
  for (auto edge : ask_edges)
  {
    ROS_INFO("I found ASK edge");
    robot_id_ = edge.get_source();
    graph_.remove_edge(edge.get_source(), edge.get(), edge.get_target());
  }
  std::string order_list_str = "response: ";
  int word_cnt = 0;
  for (auto it = order.begin(); it != order.end(); ++it)
  {
    word_cnt++;
    if (word_cnt == order.size() - 1)
      order_list_str += *it + ", and ";
    else if (word_cnt < order.size())
      order_list_str += *it + ", ";
    else
      order_list_str += *it + ".";
  }
  graph_.add_edge(robot_id_, order_list_str, robot_id_);
  robot_id_ = "";
  last_response_ = "";
  order.clear();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "qr_perception_node");
  ros::NodeHandle nh("~");
  QR_perception qr_perception(nh);

  ros::spin();

  return 0;

}
