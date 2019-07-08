#!/usr/bin/env python

"""
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
"""
import os
import sys, getopt
import commands
import ruamel.yaml

def fetch_ports_map():
	q=commands.getstatusoutput('sudo v4l2-ctl --list-devices')
	w=q[1]
	res=w.split('\n\n')
	usb_id=[]
	cam_id=[]
	for val in res:
		cam_id.append(val.split('\t')[1].split('\n')[0].split('/')[2])
		usb_id.append(val.split('\t')[0].split('(')[1].split(')')[0])
	return usb_id, cam_id

def dump_yaml(usb_id,cam_id,wrkspace):
	port_to_dev_map = {}
	yaml_feature = {}
	for i in range(len(usb_id)):
		port_to_dev_map[cam_id[i]]=ruamel.yaml.scalarstring.DoubleQuotedScalarString(usb_id[i])
	yaml_feature["port_map"] = port_to_dev_map
	yaml_file = wrkspace + "camera_dev_path.yaml"
	with open(yaml_file, 'w') as outfile:
		ruamel.yaml.round_trip_dump(yaml_feature, outfile, explicit_start=True, version=(1, 2))

def main(argv):
	try:
   		opts, args = getopt.getopt(argv,"hi:d:",["yamlDumpDir="])
   	except getopt.GetoptError:
   		print 'camera_usb_port_mapper.py -d <yamlDumpDir> '
   		sys.exit(2)
   	for opt, arg in opts:
   		if opt == '-h':
         		print 'camera_usb_port_mapper.py -d <yamlDumpDir>'
   			sys.exit()
   		elif opt in ("-d", "--yamlDumpDir"):
   			if os.path.isdir(arg) == False:
   				print arg
   				sys.exit(" Fatal : Path doesn't exists")
   			wrkspace = arg

   	usb_id, cam_id = fetch_ports_map()
   	dump_yaml(usb_id,cam_id,wrkspace)
	print "Done !!!"

if __name__ == '__main__':
    main(sys.argv[1:])
