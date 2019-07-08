#!/usr/bin/env python
"""
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
"""
import rosgraph
import rosbag
import os
import image_conversion
import sys, getopt
import uuid
import rostopic


class Consolidator(object):

	def __init__(self,inputfile,outputfile):
		#self.in_rosbag_ = rospy.get_param('~input_rosbag')
		self.in_rosbag_ = inputfile
		#self.out_rosbag_ = rospy.get_param('~output_rosbag')
		self.out_rosbag_ = outputfile
		if not os.path.exists(self.in_rosbag_):
			raise rostopic.ROSTopicException("bag file [%s] does not exist"%self.in_rosbag_)
		self.input_bag_ = rosbag.Bag(self.in_rosbag_)
		self.output_bag_ = rosbag.Bag(self.out_rosbag_, 'w')
		unique_filename = str(uuid.uuid4())
		cmd = 'lsblk -o uuid,mountpoint > ' + str(unique_filename)
		os.system(cmd)
		self.mount_uuid_map = {}
		f=open(str(unique_filename),'r')
		for i in f.readlines():
		    uuid_list=i.split()
		    if(len(uuid_list)==2):
		            self.mount_uuid_map[uuid_list[0]]=uuid_list[1]
		f.close()
		cmd = 'rm -rf ' + str(unique_filename)
		os.system(cmd)

	def fetch_and_write(self):
		for topic, raw_msg, t in self.input_bag_.read_messages(raw=True):
			msg_type, serialized_bytes, md5sum, pos, pytype = raw_msg
			msg = pytype()
			msg.deserialize(serialized_bytes)
			if msg._type == 'datainfile/ImageDataFile':
#				print "found data image msg"
				"""
				Invoke the function that converts camera image to
				sensor_msgs/Image msg
				"""
				img_msg = image_conversion.read_from_file(msg,self.mount_uuid_map)

				"""
				Write the updated msg to the rosbag.
				"""
				self.output_bag_.write(topic,img_msg,t)
			else:
				if topic == '/fix':
					topic = '/gps/fix'
				if topic == '/imu':
					topic = '/imu/data'
				self.output_bag_.write(topic,msg,t)

		self.output_bag_.close()
		self.input_bag_.close()


def main(argv):
	inputfile = ''
   	outputfile = ''
   
   	try:
   		opts, args = getopt.getopt(argv,"hi:o:",["iRosbag=","oRosbag="])
   	except getopt.GetoptError:
   		print 'rosbag_reconstruct.py -i <inputRosbag> -o <outputRosbag>'
   		sys.exit(2)
   	for opt, arg in opts:
   		if opt == '-h':
   			print 'rosbag_reconstruct.py -i <inputRosbag> -o <outputRosbag>'
   			sys.exit()
   		elif opt in ("-i", "--iRosbag"):
   			inputfile = arg
   		elif opt in ("-o", "--oRosbag"):
   			outputfile = arg
   	print 'Input file is "', inputfile
   	print 'Output file is "', outputfile

   	''' Instantiate the class '''
   	node_ = Consolidator(inputfile,outputfile)
   	node_.fetch_and_write()


if __name__ == '__main__':
    main(sys.argv[1:])
