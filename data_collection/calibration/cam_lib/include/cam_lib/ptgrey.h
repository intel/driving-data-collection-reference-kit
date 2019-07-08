/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#ifndef PTGRY_API_H_
#define PTGRY_API_H_

#include "cam_base.h"
class PTGRY : public CAM_BASE{

	BusManager busMgr;
	GigECamera cam;
	Image rawImage;

	public:
	Mode camera_mode;
	void check_mode(int mode_value);
	Mat retriveImg(void);
    void init_cam(const string& cam_choice);	
    Mat retriveRectifiedLeft(CamTestOptions opts);
    void retriveRectifiedImgs(vector<Mat>& img,CamTestOptions opts);
    void readconfig_ref(const string& serial,CamTestOptions& opts, const string & config_path);
	void save_params(CamTestOptions opts, const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const string& serial);
	void save_params_pair(CamTestOptions opts[], const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const vector<string>& serial);
	void closecamera(void){}	
};

#endif
