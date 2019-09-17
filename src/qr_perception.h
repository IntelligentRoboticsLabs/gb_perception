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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bica_graph/graph_client.h>
#include <string>
#include <iostream>
#include <boost/foreach.hpp>

#ifndef QR_PERCEPTION_H
#define QR_PERCEPTION_H

class QR_perception
{
public:
    explicit QR_perception(const ros::NodeHandle& nh);
protected:
    void QRonStandByState(const std_msgs::String::ConstPtr& qr_msg);
    void QRonRecogState(const std_msgs::String::ConstPtr& qr_msg);
    void QRonOrderState(const std_msgs::String::ConstPtr& qr_msg);
    void qrCallback(const std_msgs::String::ConstPtr& qr_msg);
    void timerCallback2order(const ros::TimerEvent&);
    void timerCallback2standby(const ros::TimerEvent&);
private:
    ros::NodeHandle nh_;
    ros::Subscriber qr_reader_sub_;
    bica_graph::GraphClient graph_;
    std::string last_response_, robot_id_;
    std::vector<std::string> order;
    ros::Timer timer_recognized;
    ros::Timer timer_order;
    ros::Time last_time_;
    int state_;
    static const int STAND_BY = 0;
  	static const int RECOGNIZED = 1;
  	static const int ORDER = 2;
};
#endif  // QR_PERCEPTION_H
