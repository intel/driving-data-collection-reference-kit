/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#include <iostream>
#include <exception>
#include <stdexcept>

#include <ros/ros.h>
#include <ros/console.h>
#include <can_node/can_frame.h>
#include <can_node/can_frame_array.h>
#include "can.h"

namespace can_node
{
  class canNode
  {
    private:
      // private ROS node handle
      ros::NodeHandle nh_;
      ros::NodeHandle priv_nh_;
      double cmd_frequency_;
      double status_frequency_;
      int channel_no_;
      int serial_no_;
      float timer_duration;
      string can_interface_type_;
      string can_sdk_type_;
      int baud_rate_;
      canHWInterface *can_hw_iface_;

      ros::Publisher can_frame_publisher_;
      
    public:

      /*
      canNode:  Constructor for can node, which initializes all the launch parameters and decides
                which CAN hardware interface to be used.

      @param:
       nh [in]     : Node handle
       priv_nh[in] : Private node handle
      */
      canNode(ros::NodeHandle nh, ros::NodeHandle priv_nh):
                                          nh_(nh), priv_nh_(priv_nh)
      {

        if(!(priv_nh_.getParam("status_freq", status_frequency_)))
          throw " Car status_ frequency is not defined ";
        if(!(priv_nh_.getParam("can_interface_type", can_interface_type_)))
          throw " Can interface type is not defined ";
        if(!(priv_nh_.getParam("baud_rate", baud_rate_)))
          throw "Baud rate is not defined ";

        /* If we are using directly the can driver to send can messages we dont need any SDK but if its not
         * driver we need to use some SDK
        */
        if(can_interface_type_ != "driver")
        {
          if(!(priv_nh_.getParam("can_sdk_type", can_sdk_type_)))
            throw "Can SDK interface type is not defined ";
        }


        if(can_interface_type_ == "SDK" && can_sdk_type_ == "kvaser")
        {
          if(!(priv_nh_.getParam("serial_no", serial_no_)))
            throw " serial_no_ is not defined ";
          if(!(priv_nh_.getParam("channel_no", channel_no_)))
            throw " channel_no_ is not defined ";

          can_hw_iface_ = new canInterfaceKvaser(can_interface_type_, can_sdk_type_, baud_rate_, canNode::getRxMsg, this, serial_no_, channel_no_);
        }
        else
        {
          throw std::runtime_error( can_interface_type_ + "  can interface and " +  can_sdk_type_ + " can sdk not supported ");
        }
        if (can_hw_iface_->initCanInterface())
        {
          delete can_hw_iface_;
          throw "Failed to initialize Can hardware interface ";
        }
        if(can_hw_iface_->configCanInterface())
        {
          delete can_hw_iface_;
          throw "Configuration of the CAN HW interface failed ";
        }

        can_frame_publisher_ = nh_.advertise<can_node::can_frame_array>("can_frames", 1);
      }

      /*
        ~canNode: Destructor for the CAN node for de intialization of CAN device.
      */
      ~canNode()
      {
        can_hw_iface_->destroyCanInterface();
        delete(can_hw_iface_);
      }
      
      /*
        getRxMsg: A static member function which is a callback function to recieve the can message.

        @Parameters:
        can_msg [in] : A vector of can messages read from the device
        data [in]    : A pointer to the canNode class itself

        @Returns nothing

      */
      static void getRxMsg(const std::vector<can_msg *> msg, void *data)
      {
        //canNode *node = static_cast<canNode *>(data);
        canNode *node = (canNode *)data;

        
        unsigned char d[8] = { '\0' };
        uint32_t id;
        uint32_t len;
        
        can_node::can_frame_array array_of_frames;
        array_of_frames.header.frame_id = "can_frames";

        can_msg *can_message = NULL;
        for (auto it = msg.begin(); it != msg.end(); it++)
        {
          can_message = *it;

          id = can_message->id;
          len = can_message->len;

          memcpy(d, (unsigned char *)can_message->data, len);
          
          can_node::can_frame frame;
          
          array_of_frames.header.stamp = ros::Time::now();
          frame.id = id;
          frame.len = len;
          for (uint8_t i =0; i < len; i++)
            frame.message.push_back(d[i]);
          frame.msg_timestamp = can_message->time_stamp;

          array_of_frames.frame_array.push_back(frame);

          delete (char *)can_message->data; // once you are done with deep copy, delete the pointer here itself
          delete can_message;
        }

          node->can_frame_publisher_.publish(array_of_frames);
      }
      
      /*
      getStatusFreq : returns the frequency in which the loop is supposed to run
      @return: returns a integer value.
      */
      int getStatusFreq()
      {
        return status_frequency_;
      }
  };
};


int main(int argc, char **argv)
{
  int status_freq;
  ros::init(argc, argv, "can");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  try
  {
    can_node::canNode can_node(nh, priv_nh);
    status_freq = can_node.getStatusFreq();
    ros::Rate loop_rate(status_freq);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
  }
  catch (const char *exp)
  {
    ROS_FATAL(" EXCEPTION CAUGHT : %s \n", exp);
  }

  
  ROS_DEBUG("Out of ROS Loop, Exiting CAN node \n");
  return 0;
}

