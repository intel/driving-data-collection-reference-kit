#!/usr/bin/env python

"""
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
"""

import os
import sys, getopt
import rosbag_split
import multiprocessing
from multiprocessing import Process
from collections import namedtuple
import concurrent.futures

DiskUsage = namedtuple('DiskUsage', 'total used free')

def disk_usage(path):
    """Return disk usage statistics about the given path.

    Will return the namedtuple with attributes: 'total', 'used' and 'free',
    which are the amount of total, used and free space, in bytes.
    """
    st = os.statvfs(path)
    free = st.f_bavail * st.f_frsize
    total = st.f_blocks * st.f_frsize
    used = (st.f_blocks - st.f_bfree) * st.f_frsize
    return DiskUsage(total, used, free)

def multithreading(func, args, workers):
	with concurrent.futures.ProcessPoolExecutor(workers) as ex:
		res = ex.map(func, args)
	print ("Multi process completed .. ")
	return list(res)

def execute_cmd(val):
  print val
  os.system(val)

def split_rosbag(inputfile,dump_paths,userpaswd,num,time):
  args = []
  for path in dump_paths:
		Total, Used, Free = disk_usage(path)
		print path
		print Total, Used, Free
		inrosbag_size = os.path.getsize(inputfile)
		if inrosbag_size < Free:
			pos = inputfile.rfind("/")
			file_name = inputfile[pos+1:]
			file_prefix = file_name.rsplit(".")
			in_rbag = str(inputfile)
			cmd = "echo " + str(userpaswd) + " | sudo -S mkdir " + str(path) + "./split_rosbags/"
			print cmd
			os.system(cmd)
			cmd = "echo " + str(userpaswd) + " | sudo -S chmod -R 777 " + str(path) + "./split_rosbags"
			print cmd
			os.system(cmd)
			out_rbag = str(path) + "./split_rosbags/" + str(file_prefix[0]) + "_part_"
                        args.append([in_rbag,out_rbag,num,time])
			print args
			split_rosbag_list = multithreading(rosbag_split.split_rosbag,args,1)
			print split_rosbag_list[0]
			return split_rosbag_list[0]

def reconstruct_rosbag(splitted_bags,dump_paths,num_proc,image_size,userpaswd):
  args = []
  counter = 0
  conv_possible = False
  fetch_capacity = True
  for bag in splitted_bags:
    bag_size = os.path.getsize(bag)
    print "bag =" , bag_size
    print "image size =", image_size
    cmd = "rosbag info " + str(bag) + " | grep \"datainfile/ImageDataFile\" | grep \"image_raw\" | grep -Eo \'[0-9]{0,9}\' > image_count.txt"
    os.system(cmd)
    image_count = 0
    f=open('image_count.txt','r')
    for i in f.readlines():
      image_count += int(i)
    print "num of images =", image_count
    total_out_bag_size = bag_size + (image_count * image_size)
    print "Total = ",total_out_bag_size
    if(counter == len(dump_paths)):
      print "Fatal : Insufficient space to convert all data"
      break
    while counter < len(dump_paths):
      if fetch_capacity:
        Total, Used, Free = disk_usage(dump_paths[counter])
	cmd = "echo " + str(userpaswd) + " | sudo -S mkdir " + str(dump_paths[counter]) + "./RCrosbag/"
	print cmd
	os.system(cmd)
	cmd = "echo " + str(userpaswd) + " | sudo -S chmod -R 777 " + str(dump_paths[counter]) + "./RCrosbag/"
	print cmd
	os.system(cmd)
	 
      if Free > total_out_bag_size:
        fetch_capacity = False
        Free = Free - total_out_bag_size
        pos = bag.rfind("/")
        out_filename = bag[pos+1:]
        cmd = "./rosbag_reconstruct.py -i " + str(bag) + " -o " + str(dump_paths[counter]) + "./RCrosbag/" + out_filename
        print cmd
        args.append(cmd)
        break
      else:
        counter += 1
        fetch_capacity = True
  print "Proceeding ahead with below conversion :"
  counter = 0
  for val in args:
    counter += 1
    print counter, val
  if num_proc == 0:
    num_proc = multiprocessing.cpu_count()
  print "Maximum number of processes spawned ",num_proc
  val = multithreading(execute_cmd,args,num_proc)
  print val

def main(argv):
	dump_paths = []
	num = 0
	time = 60.0
        num_proc = 0
        im_height = 0
        im_width = 0
	try:
   		opts, args = getopt.getopt(argv,"hi:d:p:n:t:T:H:W:",["irosbag=","dirpath=","paswd=","numsplit=","timesplit=","numParallelProc=","imageHeight=","imageWidth="])
   	except getopt.GetoptError:
   		print 'consolidate.py -i <inputRosbag> -d <drivepath> -p <sudo user password> -n <number of bag splits> -t <time of bag split> -T <number of parallel proc> -H <Height of image> -W <Weigth of image>'
   		sys.exit(2)
   	for opt, arg in opts:
   		if opt == '-h':
         		print 'consolidate.py -i <inputRosbag> -d <drivepath> -p <sudo user password> -n <number of bag splits> -t <time of bag split> -T <number of parallel proc> -H <Height of image> -W <Weigth of image>'
   			sys.exit()
   		elif opt in ("-i", "--irosbag"):
   			if os.path.exists(arg) == False:
   				print arg
   				sys.exit(" Fatal : Input rosbag doesn't exists")
   			inputfile = arg

   		elif opt in ("-d", "--dirpath"):
   			if os.path.isdir(arg) == False:
   				print arg
   				sys.exit(" Fatal : Path doesn't exists")
   			dump_paths.append(arg)
   		elif opt in ("-p", "--paswd"):
   			userpaswd = arg
   		elif opt in ("-n", "--numsplit"):
   			num = int(arg)
   		elif opt in ("-t", "--timesplit"):
   			time = float(arg)
   		elif opt in ("-T", "--numParallelProc"):
   			num_proc = int(arg)
   		elif opt in ("-H", "--imageHeight"):
   			im_height = int(arg)
   		elif opt in ("-W", "--imageWidth"):
   			im_width = int(arg)
        if im_height == 0 or im_width == 0:
            im_height = int(raw_input("Image Height = "))
            im_width = int(raw_input("Image Width = "))
        
        yuyv_to_rgb = im_width * im_height * 3
   	print inputfile 
   	print dump_paths
   	print userpaswd

   	splitted_bags =split_rosbag(inputfile,dump_paths,userpaswd,num,time)
   	reconstruct_rosbag(splitted_bags,dump_paths,num_proc,yuyv_to_rgb,userpaswd)
    	print "Done !!!"

if __name__ == '__main__':
    main(sys.argv[1:])
