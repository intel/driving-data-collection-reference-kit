/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/
#include <ros/ros.h>
#include <ros/console.h>

#include <canlib.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "can.h"
#ifdef __cplusplus
}
#endif

using namespace can_node;
canInterfaceKvaser *gkvaserPtr;

/* canInterfaceKvaser: constructor for kvaser interface.
  @params:
     interface_type [in] : interface type for kvaser
     sdk_type [in]       : sdk type for kvaser
     baud_rate [in]      : can baud rate [ preferrably 500K ]
     ptr [in]            : A function pointer to callback when can recieves a message.
     data [in]           : Pointer for the can node itself.
     serial_no [in]      : serial no of the CAN device.
     channel_no [in]     : channel no of the CAN.
 
*/
canInterfaceKvaser::canInterfaceKvaser(string interface_type, string sdk_type, int baud_rate,
                                       void (*ptr)(const std::vector<can_msg *> msg, void *data), void *data, unsigned int serial_no,
                                       int channel_no)
  : canHWInterface(interface_type, sdk_type, baud_rate, ptr, data)
  , handle_(-1)
  , serial_no_(serial_no)
  , channel_no_(channel_no)
{
  gkvaserPtr = this;
  can_bus_on = false;
}

/* check: Error checking of KVASER CAN api functions. converts status to Error String
   @params 
   id [in]   : string represents the api name which return condition to be checked.
   stat [in] : status 

   @returns nothing.  
*/
static void check(char const* id, canStatus stat)
{
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    ROS_ERROR("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }
}

/* notifyCallback : callback function for events (status, error, tx, rx, etc..)
  
   @params:
      data  [in]   : Data which is passed when registering the callback.

   @returns nothing.

*/
void notifyCallback(canNotifyData *data)
{
  long id = 0;
  unsigned char d[8] = { '\0' };
  unsigned int dlc = 0;
  unsigned int flags = 0;
  unsigned long time = 0;
  canStatus status;
  std::vector<can_msg *> msg_vector;

  switch (data->eventType)
  {
    case canEVENT_STATUS:
      ROS_INFO("CAN Status Event ");
      break;

    case canEVENT_ERROR:
      ROS_INFO("CAN Error Event\n");
      break;

    case canEVENT_TX:
      ROS_INFO("CAN Tx Event\n");
      break;

    case canEVENT_RX:
    {
      ROS_INFO("CAN Rx Event\n");
      
      do
      {
        status = canRead((*((int *)(data->tag))), &id, &d, &dlc, &flags, &time);
        if (status == canOK)
        {
          can_msg *can_message = new can_msg;
          can_message->len = dlc;
          can_message->id = id;
          can_message->data = new char[dlc];
          can_message->time_stamp = ros::Time::now();
          memcpy(can_message->data, d, dlc);
          msg_vector.push_back(can_message);

          if ((flags & canMSG_RTR) == 0)
          {
            for (unsigned int j = 0; j < dlc; j++)
            {
              ROS_DEBUG("%02x ", d[j]);
            }
          }

          ROS_DEBUG("RxMsg: Ch:%d ID:%08lx DLC:%u Flg:%02x T:%08lx \n", *((int *)(data->tag)), id, dlc, flags, time);
        }
        else if ((status != canERR_NOMSG) && status < 0)
        {
          check("Read", status);
          ROS_ERROR(" canRead failed \n");
        }

      } while (status != canERR_NOMSG);
      gkvaserPtr->getRxMsg(msg_vector, gkvaserPtr->data_);

      break;
    }
    default:
      ROS_ERROR(" Unsupported \n");
  }
}

/* getBaudRateEnum : get the can baud rate.
   @params:
      baud_rate  [in] : baud rate to be get.

    @returns can baud rate to be set.
*/

static int getBaudRateEnum(int baud_rate)
{
  switch (baud_rate)
  {
    case 10:
      /* 10 KBPS */
      return canBITRATE_10K;
    case 83:
      return canBITRATE_83K;
    case 50:
      return canBITRATE_50K;
    case 62:
      return canBITRATE_62K;
    case 100:
      return canBITRATE_100K;
    case 125:
      return canBITRATE_125K;
    case 250:
      return canBITRATE_250K;
    case 500:
      return canBITRATE_500K;
    case 1000:
      return canBITRATE_1M;
    default:
      return 0;
  }
}

/* initCanInterface : initialize the can interface.

   @params, @returns : nothings
*/
int canInterfaceKvaser::initCanInterface()
{
  unsigned int fw[2] = { '\0' };
  int chanCount = 0;
  unsigned int serial[2];
  int channel = 0;

  if (canOK != canGetNumberOfChannels(&chanCount))
  {
    ROS_ERROR(" failed to get no of channels ");
    return -1;
  }
  ROS_INFO(" no of channels - %d \n", chanCount);

  for (int i = 0; i < chanCount; i++)
  {
    canStatus stat = canGetChannelData(i, canCHANNELDATA_CARD_SERIAL_NO, &serial, sizeof(serial));
    if (stat != canOK)
    {
      check("canGetChannelData", stat);
      return -1;
    }

    if (serial_no_ == serial[0])
    {
      if (channel_no_ == 0)
      {
        channel = i;
      }
      else
      {
        channel = i + 1;
      }
      ROS_INFO(" serial no if found at %d and channel is %d", i, channel);
      break;
    }
  }

  // Initialize CANLIB.
  canInitializeLibrary();

  handle_ = canOpenChannel(channel, canOPEN_ACCEPT_VIRTUAL);
  if (handle_ < 0)
  {
    ROS_ERROR("canOpenChannel failed - %d \n", handle_);
    return -1;
  }

  if (canGetChannelData(channel, canCHANNELDATA_CARD_FIRMWARE_REV, &fw, sizeof(fw)) != canOK)
  {
    ROS_ERROR("Failed to get Firmware revision \n");
  }
  else
  {
    ROS_INFO("firmware version : %u.%u.%u.%u \n", fw[1] >> 16, fw[1] & 0xffff, fw[0] >> 16, fw[0] & 0xffff);
  }

  return 0;
}

/* configCanInterface : configure the can interface.

   @params, @returns : nothings
*/

int canInterfaceKvaser::configCanInterface()
{
  const unsigned int drivertype = canDRIVER_NORMAL;
  long freq;
  unsigned int tseg1, tseg2, sjw, noSamp, syncmode;
  int baudRate;

  canStatus stat;
  if ((stat = canSetBusOutputControl(handle_, drivertype)) != canOK )
  {
    check("canSetBusOutputControl", stat);
    return -1;
  }
  else
  {
    ROS_INFO(" Set normal mode \n");
  }

  if (canOK == canGetBusParams(handle_, &freq, &tseg1, &tseg2, &sjw, &noSamp, &syncmode))
  {
    printf("freq %ld, tseg1 %u, tseg2 %u, sjw %u, noSamp %u, syncmode %u\n ", freq, tseg1, tseg2, sjw, noSamp,
           syncmode);
  }
  else
  {
    ROS_ERROR(" Failed to fetch the baud Rate \n");
  }

  baudRate = getBaudRateEnum(baud_rate_);

  if ( (stat = canSetBusParams(handle_, baudRate, 0, 0, 0, 0, 0)) != canOK)
  {
    check("canSetBusParams", stat);
    return -1;
  }
  else
  {
    ROS_INFO("baud Rate set to baudRate - %d \n", baud_rate_);
  }

  if ( (stat = canBusOn(handle_)) != canOK)
  {
    check("canSetBusParams", stat);    
    return -1;
  }
  else
  {
    can_bus_on = true;
    ROS_INFO("Can bus on");
  }

  // Register the notify callback and pass the can handle to read the messages on that handle.
  if ( (stat = canSetNotify(handle_, notifyCallback,
               canNOTIFY_RX | canNOTIFY_TX | canNOTIFY_ERROR | canNOTIFY_STATUS | canNOTIFY_ENVVAR, &handle_)) != canOK)
  {
    check("canSetNotify", stat);    
    return -1;
  }
  return 0;
}

/* destroyCanInterface : destroy the can interface.

   @params, @returns : nothings
*/

void canInterfaceKvaser::destroyCanInterface()
{
  if (!can_bus_on)
  {
    canStatus stat;
    if ((stat = canBusOff(handle_)) != canOK)
    {
      check("canBusOff", stat);
      ROS_ERROR("ERROR canBusOff() FAILED Err= %d. <line: %d>\n", stat, __LINE__);
      return;
    }

    if ((stat = canClose(handle_)) != canOK)
    {
      check("canClose", stat);
      ROS_ERROR("ERROR canClose() in Cleanup() FAILED Err= %d. <line: %d>\n", stat, __LINE__);
      return;
    }
  }
}




