/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_client.h>
#include <interactive_marker_proxy/GetInit.h>

using namespace interactive_markers;

class Proxy
{
public:
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  interactive_markers::InteractiveMarkerClient client_;
  ros::Publisher pub_;
  ros::Timer timer_;
  unsigned subscribers_;
  std::string topic_ns_;
  std::string target_frame_;
  ros::ServiceServer service_;
  std::string status_text_;

  std::map<std::string, visualization_msgs::InteractiveMarker> markers_;
  std::map<std::string, geometry_msgs::PoseStamped> frame_locked_poses_;

  Proxy(std::string target_frame, std::string topic_ns) :
      client_(tf_, target_frame, topic_ns), subscribers_(0), topic_ns_(topic_ns), target_frame_(target_frame)
  {
    ROS_INFO_STREAM("Subscribing to " << topic_ns);
    ROS_INFO_STREAM("Target frame set to " << target_frame);

    client_.setInitCb(boost::bind(&Proxy::initCb, this, _1));
    client_.setUpdateCb(boost::bind(&Proxy::updateCb, this, _1));
    client_.setResetCb(boost::bind(&Proxy::resetCb, this, _1));
    client_.setStatusCb(boost::bind(&Proxy::statusCb, this, _1, _2, _3));

    client_.subscribe(topic_ns_);

    pub_ = nh_.advertise<visualization_msgs::InteractiveMarkerUpdate>(topic_ns_ + "/tunneled/update", 1000);

    timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Proxy::timerCb, this, _1));

    service_ = nh_.advertiseService(topic_ns_ + "/tunneled/get_init", &Proxy::getInit, this);
  }

  typedef visualization_msgs::InteractiveMarkerInitConstPtr InitConstPtr;
  typedef visualization_msgs::InteractiveMarkerUpdateConstPtr UpdateConstPtr;
  typedef visualization_msgs::InteractiveMarkerUpdatePtr UpdatePtr;

  bool getInit(interactive_marker_proxy::GetInit::Request& request,
               interactive_marker_proxy::GetInit::Response& response)
  {
    ROS_INFO("Init requested.");
    std::vector< visualization_msgs::InteractiveMarker > markers;
    std::map<std::string, visualization_msgs::InteractiveMarker>::iterator it;
    for( it = markers_.begin(); it!=markers_.end(); it++ )
    {
      response.msg.markers.push_back(it->second);

    }
    return true;
  }

  void timerCb(const ros::TimerEvent&)
  {
    client_.update();

    // send pose updates for frame-locked IMs
    visualization_msgs::InteractiveMarkerUpdate up_msg;
    up_msg.poses.reserve(frame_locked_poses_.size());

    std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
    for (it = frame_locked_poses_.begin(); it != frame_locked_poses_.end(); it++)
    {
      try
      {
        std_msgs::Header& header = it->second.header;
        tf::StampedTransform transform;
        tf_.lookupTransform(target_frame_, header.frame_id, header.stamp, transform);

        tf::Pose pose;
        tf::poseMsgToTF(it->second.pose, pose);
        pose = transform * pose;

        // store transformed pose in update message
        visualization_msgs::InteractiveMarkerPose imp;
        imp.name = it->first;
        tf::poseTFToMsg(pose, imp.pose);
        imp.header.frame_id = target_frame_;
        up_msg.poses.push_back(imp);
      }
      catch (...)
      {
      }
    }
    pub_.publish(up_msg);
  }

  void updateCb(const UpdateConstPtr& up_msg)
  {
    const visualization_msgs::InteractiveMarkerUpdate::_erases_type& erases = up_msg->erases;
    for (unsigned i = 0; i < erases.size(); i++)
    {
      frame_locked_poses_.erase(erases[i]);
      markers_.erase(erases[i]);
    }

    std::vector<visualization_msgs::InteractiveMarkerPose> fixed_poses;

    const visualization_msgs::InteractiveMarkerUpdate::_poses_type& poses = up_msg->poses;
    for (unsigned i = 0; i < poses.size(); i++)
    {
      // if the message is in the target frame, we will not send
      // pose updates anymore, so send it now
      if (poses[i].header.frame_id == target_frame_)
      {
        frame_locked_poses_.erase(poses[i].name);
        fixed_poses.push_back(poses[i]);
      }
      else
      {
        geometry_msgs::PoseStamped p;
        p.header = poses[i].header;
        p.pose = poses[i].pose;
        frame_locked_poses_[poses[i].name] = p;
      }
    }

    const visualization_msgs::InteractiveMarkerUpdate::_markers_type& markers = up_msg->markers;
    for (unsigned i = 0; i < markers.size(); i++)
    {
      if (markers[i].header.frame_id == target_frame_)
      {
        frame_locked_poses_.erase(markers[i].name);
      }
      else
      {
        geometry_msgs::PoseStamped p;
        p.header = markers[i].header;
        p.pose = markers[i].pose;
        frame_locked_poses_[markers[i].name] = p;
      }
      markers_[markers[i].name] = markers[i];
    }

    visualization_msgs::InteractiveMarkerUpdate up_msg2 = *up_msg;
    up_msg2.poses = fixed_poses;

    pub_.publish(up_msg2);
  }

  void initCb(const InitConstPtr& init_msg)
  {
    UpdatePtr update(new visualization_msgs::InteractiveMarkerUpdate());
    update->markers = init_msg->markers;
    update->seq_num = init_msg->seq_num;
    update->server_id = init_msg->server_id;
    updateCb(update);
  }

  void statusCb(InteractiveMarkerClient::StatusT status, const std::string& server_id, const std::string& status_text)
  {
    if ( status_text_ == status_text ) return;
    status_text_ = status;
    std::string status_string[] = {"INFO", "WARN", "ERROR"};
    ROS_INFO_STREAM( "(" << status_string[(unsigned)status] << ") " << server_id << ": " << status_text);
  }

  void resetCb(const std::string& server_id)
  {
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interactive_marker_proxy");
  {
    ros::NodeHandle nh;
    Proxy proxy(nh.resolveName("target_frame"), nh.resolveName("topic_ns"));
    ros::spin();
  }
}
