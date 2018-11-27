/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#include <string>
#include <canlib.h>

using namespace std;

namespace can_node
{

struct can_msg
{
  int id;
  int len;
  void *data;
  ros::Time time_stamp;
};

class canHWInterface
{
protected:
  string interface_type_;
  string sdk_type_;
  /* Baud rate in kbps */
  int baud_rate_;

public:
  canHWInterface(string interface_type, string sdk_type, int baud_rate,  void (*ptr)(const std::vector<can_msg *> msg, void *data),
                 void *data)
    : interface_type_(interface_type), sdk_type_(sdk_type), baud_rate_(baud_rate), getRxMsg(ptr)
  {
    data_ = data;
  }
  virtual int initCanInterface() = 0;
  virtual void destroyCanInterface() = 0;
  virtual int configCanInterface() = 0;
  void (*getRxMsg)(const std::vector<can_msg *> msg, void *data);
  void *data_;

  virtual ~canHWInterface()
  {

  } 
};


/* Class which suppport kvaser SDK for transmitting can commands. Needs to implement separate class
 * if your board is using some other can device like vector
 */
class canInterfaceKvaser : public canHWInterface
{
private:
  canHandle handle_;
  unsigned int serial_no_;
  int channel_no_;
  bool can_bus_on;

public:
  canInterfaceKvaser(string interface_type, string sdk_type, int baud_rate, void (*ptr)(const std::vector<can_msg *> msg, void *data),
                     void *data, unsigned int serial_no, int channel_no);
  int initCanInterface();
  int configCanInterface();
  void destroyCanInterface();
};

};
