/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#ifndef CAM_BASE_API_H_
#define CAM_BASE_API_H_


#include <iostream>
#include <vector>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "FlyCapture2.h"
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <fstream>
#include "TagDetector.h"
#include "opencv2/calib3d/calib3d.hpp"

using namespace FlyCapture2;
using namespace std;
using namespace cv;

typedef struct CamTestOptions {
    Mat map1;
    Mat map2;
    Mat map1R;
    Mat map2R;
    vector<Mat> K;
    Mat projR_to_L;
    Mat world_center;
    Mat proj_to_wrld;
    Mat proj_to_refcam;
} CamTestOptions;

class CAM_BASE {
	BusManager busMgr;
	CameraInfo *camInfo;
public:
	unsigned int usb_cam_connected;
	unsigned int ptgrey_connected;
	vector<string> cam_names;
	CAM_BASE();
	unsigned int check_connected_ptgrey();
	unsigned int check_connected_usb();
	void display();
	virtual void init_cam(const string& cam_choice){}
	virtual Mat retriveImg(void){}
	virtual Mat retriveRectifiedLeft(CamTestOptions opts){}
    virtual void retriveRectifiedImgs(vector<Mat>& img,CamTestOptions opts){}
    virtual void readconfig_ref(const string& serial,CamTestOptions& opts, const string & config_path){}
	virtual void save_params(CamTestOptions opts, const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const string& serial){}
	virtual void save_params_pair(CamTestOptions opts[], const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const vector<string>& serial){}
	virtual void closecamera(void){}

	virtual ~CAM_BASE()
	{

	}
};

#endif
