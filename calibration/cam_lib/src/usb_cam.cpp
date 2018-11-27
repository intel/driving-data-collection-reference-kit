/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#include "usb_cam.h"


void USB_CAM::open_device(const string& deviceName ){
  struct stat st;

  if (-1 == stat(deviceName.c_str(), &st)) {
    cout<<"Cannot identify "<<deviceName<<": "<<errno<<", "<<strerror(errno)<<endl;
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR (st.st_mode)) {
      cout<<deviceName<<" is not a character device"<<endl;
    exit(EXIT_FAILURE);
  }

  fd = open(deviceName.c_str(), O_RDWR | O_NONBLOCK, 0);

  if (-1 == fd) {
      cout<<"Cannot open "<<deviceName<<": "<<errno<<", "<<strerror(errno)<<endl;
    exit(EXIT_FAILURE);
  }
}

void USB_CAM::errno_exit(const char* s)
{
  cout<<s<<" error "<<errno<<", "<<strerror (errno)<<endl;
  exit(EXIT_FAILURE);
}


int USB_CAM::xioctl(int fd, int request, void* argp)
{
  int r;

  do r = ioctl(fd, request, argp);
  while (-1 == r && EINTR == errno);

  return r;
}

void USB_CAM::mmapInit(const string& deviceName)
{
  struct v4l2_requestbuffers req;

  CLEAR (req);

  req.count               = 4;
  req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory              = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      cout<<deviceName<<" does not support memory mapping"<<endl;
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  if (req.count < 2) {
    cout<<"Insufficient buffer memory on "<<deviceName<<endl;
    exit(EXIT_FAILURE);
  }

  buffers =static_cast<struct buffer *>(calloc(req.count, sizeof(*buffers)));

  if (!buffers) {
    cout<< "Out of memory"<<endl;
    exit(EXIT_FAILURE);
  }

  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;

    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap (NULL , buf.length, PROT_READ | PROT_WRITE , MAP_SHARED, fd, buf.m.offset);
    if (MAP_FAILED == buffers[n_buffers].start)
      errno_exit("mmap");
  }
}

void USB_CAM::init_device(const string& deviceName )
{
  struct v4l2_capability cap;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      cout<<deviceName<< " is no V4L2 device" <<endl;
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    cout<<deviceName<< " is no video capture device" <<endl;
    exit(EXIT_FAILURE);
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    cout<<deviceName<< " does not support streaming i/o" <<endl;
    exit(EXIT_FAILURE);
  }

  CLEAR (fmt);
  // v4l2_format
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = static_cast<unsigned int>(width);
  fmt.fmt.pix.height      = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

  if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
    errno_exit("VIDIOC_S_FMT");

  /* Note VIDIOC_S_FMT may change width and height. */
  if (width != fmt.fmt.pix.width) {
    cerr<<"Assertion failed in file " + string(__FILE__)+ ",at line " + to_string(__LINE__)+"\nGiven image width is not supported by camera\n";
    abort();
  }
  if (height != fmt.fmt.pix.height) {
    cerr<<"Assertion failed in file " + string(__FILE__)+ ",at line " + to_string(__LINE__)+"\nGiven image height is not supported by camera\n";
    abort();
  }

  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;

  mmapInit(deviceName);
}

void USB_CAM::captureStart(void)
{
  unsigned int i;
  enum v4l2_buf_type type;
  for (i = 0; i < n_buffers; ++i) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = i;

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
  }
           
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
    errno_exit("VIDIOC_STREAMON");
}

void USB_CAM::init_cam(const string& serial){
  buffers = NULL;
  fd=-1;
  n_buffers=0;
  string im_w, im_h;
  cout<<"provide image width which is supported by camera[default:2560]: "<<endl;
  getline(cin,im_w);
  if(im_w.empty())
      im_w="2560";
  width=stoi(im_w);
  cout<<"provide image height which is supported by camera[default:720]: "<<endl;
  getline(cin,im_h);
  if(im_h.empty())
      im_h="720";
  height=stoi(im_h);
  open_device(serial);
  init_device(serial);
  captureStart();
}

Mat USB_CAM::imageProcess(void* p)
{
  Mat dst = Mat(height,width, CV_8UC2,p);
  Mat rgb;
  cvtColor(dst,rgb,COLOR_YUV2BGR_YUYV );
  return rgb;
}

Mat USB_CAM::retriveImg()
{
  Mat img;
  for (;;) {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
        if (EINTR == errno)
          continue;

        errno_exit("select");
    }

    if (0 == r) {
        fprintf (stderr, "select timeout\n");
        exit(EXIT_FAILURE);
    }

    struct v4l2_buffer buf;
    CLEAR (buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    xioctl(fd, VIDIOC_DQBUF, &buf);
    assert (buf.index < n_buffers);

    img = imageProcess(buffers[buf.index].start);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        continue;
    break;
  }
  return img;
}

Mat USB_CAM::retriveRectifiedLeft(CamTestOptions opts){
  Mat original = retriveImg();
  Mat crop_l,rview_l;
  if(!mono){
    original(Rect(0,0,original.cols/2,original.rows)).copyTo(crop_l);
    remap(crop_l,rview_l,opts.map1,opts.map2,INTER_LINEAR);
  }
  else{
    remap(original,rview_l,opts.map1,opts.map2,INTER_LINEAR);
  }
  return rview_l;
}

void USB_CAM::retriveRectifiedImgs(vector<Mat>& img, CamTestOptions opts){
  img.clear();
  Mat original = retriveImg();
  Mat crop_l,rview_l, crop_r,rview_r;
  if(!mono){
    original(Rect(0,0,original.cols/2,original.rows)).copyTo(crop_l);
    original(Rect(original.cols/2,0,original.cols/2,original.rows)).copyTo(crop_r);
    remap(crop_l,rview_l,opts.map1,opts.map2,INTER_LINEAR);
    remap(crop_r,rview_r,opts.map1R,opts.map2R,INTER_LINEAR);
    img.push_back(rview_l);
    img.push_back(rview_r);
  }
  else{
    remap(original,rview_l,opts.map1,opts.map2,INTER_LINEAR);
    img.push_back(rview_l);
  }
}

void USB_CAM::readconfig_ref(const string& serial, CamTestOptions& opts, const string& config_path){
  FileStorage fs(config_path+serial+".yaml",FileStorage::READ);
  Mat intL,intR;
  if(!fs.isOpened())
    cout<<"unable to open param file"<<endl;

  fs["KL"] >> intL;
  fs["map1L"] >> opts.map1;
  fs[ "map2L"] >> opts.map2;
  fs["KR"] >> intR;
  fs["map1R"] >> opts.map1R;
  fs["map2R"] >> opts.map2R;
  fs["projR_to_L"] >> opts.projR_to_L;
  fs["world_center"] >> opts.world_center;
  fs["project_to_reference_cam"] >> opts.proj_to_refcam;
  fs["projection_to_world"] >> opts.proj_to_wrld;
  fs.release();
  opts.K.push_back(intL);
  opts.K.push_back(intR);
}

void USB_CAM::save_params(CamTestOptions opts, const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const string& serial){

    Mat proj1,proj2;
    rot[0](Rect(0,0,2,3)).copyTo(proj1);
    hconcat(proj1,trans[0],proj1);
    proj1 = opts.K[0]*proj1;
    invert(proj1,opts.proj_to_wrld,DECOMP_SVD);

    rot[1](Rect(0,0,2,3)).copyTo(proj2);
    hconcat(proj2,trans[1],proj2);
    proj2 = opts.K[1]*proj2;

    opts.world_center = -rot[0].t()*trans[0];
    opts.proj_to_refcam = Mat::eye(3,3,CV_64FC1);
    opts.projR_to_L = proj1*proj2.inv(DECOMP_SVD);

    FileStorage fs(config_path+"/"+serial+".yaml",FileStorage::WRITE);
    //Camera params.
    fs << "camera" << serial;
    fs << "KL" << opts.K[0];
    fs << "KR" << opts.K[1];
    fs << "map1L" << opts.map1;
    fs << "map2L" << opts.map2;
    fs << "map1R" << opts.map1R;
    fs << "map2R" << opts.map2R;
    fs << "projR_to_L" << opts.projR_to_L;
    fs << "world_center" << opts.world_center;
    fs << "rotation_wrt_wrld" << rot[0];
    fs << "translation_wrt_wrld" << trans[0];
    fs << "project_to_reference_cam" << opts.proj_to_refcam;
    fs << "projection_to_world" << opts.proj_to_wrld;
    fs.release();
}

void USB_CAM::save_params_pair(CamTestOptions opts[], const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const vector<string>& serial){

    Mat proj1,proj2;
    rot[0](Rect(0,0,2,3)).copyTo(proj1);
    hconcat(proj1,trans[0],proj1);
    proj1 = opts[0].K[0]*proj1;

    rot[1](Rect(0,0,2,3)).copyTo(proj2);
    hconcat(proj2,trans[1],proj2);
    proj2 = opts[1].K[0]*proj2;
    Mat proj_to_refcam = proj1*proj2.inv(DECOMP_SVD);
    proj_to_refcam=opts[0].proj_to_refcam*proj_to_refcam;

    FileStorage fs(config_path+"/"+serial[1]+".yaml",FileStorage::WRITE);
    //Camera params.
    fs << "camera" << serial;
    fs << "KL" << opts[1].K[0];
    fs << "KR" << opts[1].K[1];
    fs << "map1L" << opts[1].map1;
    fs << "map2L" << opts[1].map2;
    fs << "map1R" << opts[1].map1R;
    fs << "map2R" << opts[1].map2R;
    fs << "projR_to_L" << opts[1].projR_to_L;
    fs << "rotation_wrt_ref" <<rot[1];
    fs << "translation_wrt_ref" << trans[1];
    //Reference camera against which it is calibrated.
    fs << "reference_cam" << serial[0];
    fs << "reference_cam_K" << opts[0].K[0];
    fs << "ref_rotation" <<rot[0];
    fs << "ref_translation" << trans[0];
    fs << "project_to_reference_cam" <<proj_to_refcam;
    fs << "world_center" << opts[0].world_center;
    fs << "projection_to_world" << opts[0].proj_to_wrld;
    fs.release();
}

void USB_CAM::captureStop(void)
{
  enum v4l2_buf_type type;

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
    errno_exit("VIDIOC_STREAMOFF");
}

void USB_CAM::deviceUninit(void)
{
  unsigned int i;
  for (i = 0; i < n_buffers; ++i)
  if (-1 == munmap (buffers[i].start, buffers[i].length))
    errno_exit("munmap");

  free(buffers);
}

void USB_CAM::deviceClose(void)
{
  if (-1 == close (fd))
    errno_exit("close");

  fd = -1;
}

void USB_CAM::closecamera(void){
  captureStop();
  deviceUninit();
  deviceClose();
}
