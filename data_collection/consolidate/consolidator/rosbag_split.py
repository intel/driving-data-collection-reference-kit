#!/usr/bin/env python
"""
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
"""
import os
import getpass
import sys
import multiprocessing
import consolidate

def split_rosbag(arg):
	in_rosbag= str(arg[0])
	out_rosbag = str(arg[1])
	number = int(arg[2])
	time = float(arg[3])
	print in_rosbag
	print out_rosbag
	print number
	print time
	rosbag_list = []
	args = []
	if number > 0:
		cmd = "rosbag info -y -k start " + str(in_rosbag) + " > start.txt"
		os.system(cmd)
		cmd = "rosbag info -y -k end " + str(in_rosbag) + " > end.txt"
		os.system(cmd)
		st_file = open("start.txt","r")
		end_file = open("end.txt","r")

		start_time = st_file.readline()
		start_time = start_time.split()

		end_time = end_file.readline()
		end_time = end_time.split()

		print start_time
		print end_time

		st_file.close()
		end_file.close()

		cmd = "rm -rf start.txt end.txt"
		os.system(cmd)

		duration = float(end_time[0]) - float(start_time[0])

		print duration

		step = duration / number
		print step

		for i in range(number):
			print i
			time_min = float(start_time[0]) + step * i
			time_max = float(start_time[0]) + (step * (i+1))
			if time_max >= float(end_time[0]):
				time_max = float(end_time[0])
			print time_min
			print time_max
			if i == 0:
				cmd = "rosbag filter " + str(in_rosbag) + " " + str(out_rosbag) + str(i+1) + ".bag \"t.secs >= " + str(time_min) +" and t.secs <= " + str(time_max) + "\""
				print cmd
				args.append(cmd)
			else:
				cmd = "rosbag filter " + str(in_rosbag) + " " + str(out_rosbag) + str(i+1) + ".bag \"t.secs > " + str(time_min) +" and t.secs <= " + str(time_max) + "\""
				print cmd
				args.append(cmd)
			outfile = str(out_rosbag) + str(i+1) + ".bag"
			rosbag_list.append(outfile)
	else:
		if time > 0.0:
			print time
			cmd = "rosbag info -y -k start " + str(in_rosbag) + " > start.txt"
			os.system(cmd)
			cmd = "rosbag info -y -k end " + str(in_rosbag) + " > end.txt"
			os.system(cmd)
			st_file = open("start.txt","r")
			end_file = open("end.txt","r")

			start_time = st_file.readline()
			start_time = start_time.split()

			end_time = end_file.readline()
			end_time = end_time.split()

			print start_time
			print end_time

			duration = float(end_time[0]) - float(start_time[0])

			print duration

			step = duration / time
			itr = int(step) + 1
			for i in range(itr):
				print i
				time_min = float(start_time[0]) + time * i
				if time_min >= float(end_time[0]):
					break					
				time_max = float(start_time[0]) + (time * (i+1))
				if time_max >= float(end_time[0]):
					time_max = float(end_time[0])
				print time_min
				print time_max
				if i == 0:
					cmd = "rosbag filter " + str(in_rosbag) + " " + str(out_rosbag) + str(i+1) + ".bag \"t.secs >= " + str(time_min) +" and t.secs <= " + str(time_max) + "\""
					print cmd
					args.append(cmd)
				else:
					cmd = "rosbag filter " + str(in_rosbag) + " " + str(out_rosbag) + str(i+1) + ".bag \"t.secs > " + str(time_min) +" and t.secs <= " + str(time_max) + "\""
					print cmd
					args.append(cmd)
				outfile = str(out_rosbag) + str(i+1) + ".bag"
				rosbag_list.append(outfile)
			
		else:
			sys.exit(" No operation to be performed ")
	num_proc = multiprocessing.cpu_count()
	print "Number of processes = ", num_proc
	val = consolidate.multithreading(consolidate.execute_cmd,args,num_proc)
	print val
	return rosbag_list

