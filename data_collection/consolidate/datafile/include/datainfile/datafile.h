/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#ifndef DATAFILE_H_
#define DATAFILE_H_

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <fcntl.h>
#include <linux/falloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <mntent.h>

#include <ros/ros.h>

#include <vector>
#include <string>

#include "datainfile/DataFile.h"

class DataFile
{
  //List of paths where we want the
  //data to be written
  std::vector<std::string> paths;
  //List of mounted filesystem paths
  std::vector<std::string> mount_paths;

  //List of device paths mounted. eg: "/dev/sdb1"
  std::vector<std::string> dev_paths;
  //List of device uuid(s), represents unique identity
  std::vector<std::string> device_uuid;
  // Holds the system hostname
  std::string host_name;

  //Holds the file absolute path 
  //with filename for each frame.
  std::string file_to_open;
  // to iterate over multiple paths
  int current_path_index;
  // holds the file discriptor
  int current_path_fd;
  //start offset of frame in the file
  uint long long frame_offset_begin;
  //end offset of frame in the file
  uint long long frame_offset_end;

  void GenerateFileMetadata(datainfile::DataFile *msg);

public:

  DataFile () {}

  DataFile (std::vector<std::string> pathlist)
  {
    std::string   temp_filename;
    std::string   uuid_dev_str;
    FILE*         fp;
    int           temp_fd;
    int           sz;
    struct stat   s;
    dev_t         dev;
    char          buf[256];
    char          hostname[1024];
    struct hostent* host_ptr;
    struct mntent mnt;
    size_t buflen = 256;

    paths = pathlist;
    frame_offset_begin = 0;
    frame_offset_end = -1;

    // Fetch the hostname of system code is running
    hostname[1023] = '\0';
    gethostname(hostname, 1023);

    host_ptr = gethostbyname(hostname);
    host_name = host_ptr->h_name;

    // Iterate over list of paths
    for(int count=0; count< paths.size();count++)
    {
      // Check directory path existence. 
      int rval = access (paths[count].c_str(), F_OK);
      if (rval == 0) 
      ROS_INFO("%s exists\n", paths[count].c_str());
      else 
      {
        if (errno == ENOENT) 
          ROS_ERROR("%s does not exist\n", paths[count].c_str());
        else if (errno == EACCES) 
          ROS_ERROR("%s is not accessible\n", paths[count].c_str());
      }

      // Check for file write operation
      temp_filename = paths[count] + "/sample.txt";
      temp_fd = open(temp_filename.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
      if (temp_fd == -1)
      {
        ROS_ERROR("Failed"); 
        perror("FileWriteValidation");                 
      }
      write (temp_fd, (char *)temp_filename.c_str(), temp_filename.size());
      close (temp_fd);      

      // Mount point extraction
      if (stat((char *)temp_filename.c_str(), &s) != 0) 
      {
        ROS_ERROR(" Problem in fetching status of file \n");
      }

      dev = s.st_dev;
      if ((fp = setmntent("/proc/mounts", "r")) == NULL) 
      {
        ROS_ERROR(" Problem in opening /proc/mounts file \n");    
      }
      while (getmntent_r(fp, &mnt, &buf[0], buflen)) 
      {
        if (stat((mnt.mnt_dir), &s) != 0) 
        {
            continue;
        }
        if (s.st_dev == dev) 
        {
            mount_paths.push_back(mnt.mnt_dir);
            dev_paths.push_back(mnt.mnt_fsname);
            endmntent(fp);
            break;
        }
      }

      // UUID Extraction
      const void * address = static_cast<const void*>(this);
      std::stringstream ss;
      ss << address;  
      std::string fname = ss.str();
      fname = fname + ".txt";
      std::string cmd = "lsblk -o mountpoint,uuid >" + fname;
      int ret = system(cmd.c_str());
      std::ifstream input( fname.c_str(),std::ifstream::in );
      for( std::string line; getline( input, line ); )
      {
          if(!(line.empty()))
          {
            std::string delimiter = " ";
            std::string mount_p = line.substr(0, line.find(delimiter));
            std::string uuid_p = line.substr(line.find(delimiter)+1);
            for(int itr=0; itr < mount_paths.size();itr++)
            {
              if(mount_p == mount_paths[itr])
              {
                if(!(uuid_p.empty()))
                  device_uuid.push_back(uuid_p);
                break;
              }
            }
          }
      }
      input.close();
      cmd = "rm -r " + fname;
      ret = system(cmd.c_str());
      for(int itr=0; itr < mount_paths.size();itr++)
      {
        printf("\n mount =%s uuid =%s \n",mount_paths[itr].c_str(),device_uuid[itr].c_str());
      }
    }
  }

  ~DataFile()
  {
  }
  
  // returns the datafile msg after dumping the frame
  datainfile::DataFile offload (char *data, int size, bool append_file);
  
};
#endif