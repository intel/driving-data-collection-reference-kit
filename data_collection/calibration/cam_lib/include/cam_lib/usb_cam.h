/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#ifndef USB_CAM_API_H_
#define USB_CAM_API_H_

#include "cam_base.h"
#define CLEAR(x) memset (&(x), 0, sizeof (x))

struct buffer {
        void * start;
        size_t length;
};

class USB_CAM: public CAM_BASE {
    struct buffer *buffers;
    int fd;
    unsigned int n_buffers;
    unsigned int width;
    unsigned int height;
    void open_device(const string& deviceName );
    void errno_exit(const char* s);
    int xioctl(int fd, int request, void* argp);
    void mmapInit(const string& deviceName);
    void init_device(const string& deviceName );
    void captureStart(void);
    Mat imageProcess(void* p);
    Mat mainLoop(void);
    void captureStop(void);
    void deviceUninit(void);
    void deviceClose(void);

public:
    int mono;
    Mat retriveImg(void);
    void init_cam(const string& serial);
    Mat retriveRectifiedLeft(CamTestOptions opts);
    void retriveRectifiedImgs(vector<Mat>& img,CamTestOptions opts);
    void readconfig_ref(const string& serial,CamTestOptions& opts, const string & config_path);
    void save_params(CamTestOptions opts, const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const string& serial);
    void save_params_pair(CamTestOptions opts[], const vector<Mat>& rot,const vector<Mat>& trans, const string& config_path, const vector<string>& serial);
    void closecamera(void);
};

#endif
