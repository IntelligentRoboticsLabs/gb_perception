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
}

void QR_perception::qrCallback(const std_msgs::String::ConstPtr& qr_msg)
{
  if (qr_msg->data.c_str() != last_response_ && qr_msg->data.c_str() != "stop")
  {
    last_response_ = qr_msg->data.c_str();
    ROS_INFO("########### I saw a new msg! [%s]", qr_msg->data.c_str());

    std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
    {
      std::string edge = it->get();
      if (edge.find("ask") != std::string::npos)
      {
        robot_id_ = it->get_source();
        graph_.remove_edge(*it);
      }
    }
    std::string response = "response: " + last_response_;
    graph_.add_edge(robot_id_, response, robot_id_);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "qr_perception_node");
  ros::NodeHandle nh("~");
  QR_perception qr_perception(nh);

  ros::spin();

  return 0;

}
