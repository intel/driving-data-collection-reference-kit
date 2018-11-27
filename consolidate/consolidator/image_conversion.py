#!/usr/bin/env python
"""
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
"""
import numpy
import cv2
import sys

def read_from_file(msg,mount_uuid_map):
	img_msg = msg.image
	uuid_ = msg.datafile_.uuid
	if uuid_ in mount_uuid_map.keys():
		mount_path=mount_uuid_map[uuid_]
	else:
		sys.exit("required device not mounted")

	path = mount_path + "/" + msg.datafile_.filepath
	read_raw_image = numpy.fromfile(path,dtype='b')
	yuyv_img = numpy.reshape(read_raw_image,(msg.image.height,msg.image.width,2))
	yuyv_img = yuyv_img.astype(numpy.uint8)
	rgb_image = cv2.cvtColor(yuyv_img,cv2.COLOR_YUV2RGB_YUYV)
	rgb_flat= rgb_image.flatten()
	rgb_list=rgb_flat.tolist()
	img_msg.data = rgb_list
	img_msg.encoding ='rgb8'
	img_msg.step= img_msg.width * 3
	return img_msg
