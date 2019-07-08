/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include "datainfile/datafile.h"

const std::string slashChar(".");
const std::string dumpPath("/dump");
const std::string fileExt(".dat");

extern int errno;

using namespace std;

  void DataFile::GenerateFileMetadata(datainfile::DataFile *msg)
  {
    msg->filepath = file_to_open.replace(0,mount_paths[current_path_index].length(), slashChar);
    msg->uuid = device_uuid[current_path_index];
    msg->host = host_name;
    msg->file_offset_begin = frame_offset_begin;
    msg->file_offset_end = frame_offset_end;
  }

  datainfile::DataFile DataFile::offload (char *data, int size, bool append_file)
  {
    static uint long long count = 0;
    datainfile::DataFile msg;
    current_path_index = count % paths.size();
   
    //Open with the current index
    file_to_open = paths[current_path_index] + dumpPath + std::to_string(count) + fileExt;

    if(append_file)
    {
      
      current_path_fd = open(file_to_open.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
      //Write
      if (current_path_fd ==-1)
      {
        ROS_ERROR("Failed");        
        perror("FileDump");                 
      }

      write (current_path_fd, (char *)data, size);
      close (current_path_fd);
      frame_offset_begin = frame_offset_end + 1;
      frame_offset_end = frame_offset_begin + size - 1;

      GenerateFileMetadata(&msg);
    }
    else
    {
      //Open with the current index
      current_path_fd = open(file_to_open.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
      
      //Write
      if (current_path_fd ==-1)
      {
        ROS_ERROR("Failed"); 
        perror("FileDump");                 
      }

      write (current_path_fd, (char *)data, size);
      close (current_path_fd);
      frame_offset_begin = 0;
      frame_offset_end = size - 1;

      GenerateFileMetadata(&msg);
      //Move to the next index if needed.
      count++;    
    }
    return msg;
  }
