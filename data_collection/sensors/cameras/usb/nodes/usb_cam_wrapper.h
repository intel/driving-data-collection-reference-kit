#ifndef USB_CAM_INCLUDE_USB_CAM_USBCAMWRAPPER_H_
#define USB_CAM_INCLUDE_USB_CAM_USBCAMWRAPPER_H_

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <pluginlib/class_loader.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include "datainfile/ImageDataFile.h"

namespace usb_cam {

enum TriggerFrequence {
    FPS_10HZ = 10,
    FPS_15HZ = 15,
    FPS_20HZ = 20,
    DEFAULT_FPS = 30
};

class UsbCamWrapper {
 public:
  UsbCamWrapper(ros::NodeHandle node, ros::NodeHandle private_nh);
  virtual ~UsbCamWrapper();
  bool service_start_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool service_stop_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  int take_and_send_image();
  int spin();

 private:
  // util method to split image into left and right
  void splitImage(const sensor_msgs::Image img, sensor_msgs::Image& imgl, sensor_msgs::Image& imgr);
  // shared image message
  datainfile::ImageDataFile img_data_file_;
  sensor_msgs::Image img_;
  sensor_msgs::Image imgL_, imgR_;
  sensor_msgs::CameraInfoPtr cam_info_ = nullptr;
  //image_transport::CameraPublisher image_pub_;
  DataFile *dataFileObj;
  image_transport::PubLoaderPtr pub_loader_;
  boost::shared_ptr<image_transport::PublisherPlugin> image_pub_plugin_; // default / left cam
  boost::shared_ptr<image_transport::PublisherPlugin> image_pub_plugin_R_; // right cam; used when split_image=true

  ros::Publisher cam_info_pub_;
  ros::Publisher cam_dump_pub_;

  // parameters
  std::string topic_name_;
  std::string video_device_name_; 
  std::string io_method_name_; 
  std::string dev_map_yml_;
  std::string pixel_format_name_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string usb_port_name_;
  //std::string dump_path_;
  std::vector<std::string> dump_path_;
  std::string cam_product_id_;
  std::string cam_vendor_id_;
  std::string cam_id_string_;
  bool dump_to_disk_;
  bool split_image_;
  
  //std::string start_service_name_, start_service_name_;
  //bool streaming_status_;
  int image_width_;
  int image_height_;
  int framerate_;
  int exposure_;
  int brightness_;
  int contrast_; 
  int saturation_;
  int sharpness_;
  int focus_;
  int white_balance_;
  int gain_;

  bool autofocus_;
  bool autoexposure_;
  bool auto_white_balance_;

  // usb will be reset when camera timeout
  int cam_timeout_;
  UsbCam cam_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  ros::ServiceServer service_start_; 
  ros::ServiceServer service_stop_; 

  // private ROS node handle
  ros::NodeHandle node_;
  ros::NodeHandle priv_node_;

  ros::Time last_stamp_;
  float frame_warning_interval_;
  float frame_drop_interval_;
  float spin_interval_;
  int error_code_;
};
}

#endif /* USB_CAM_INCLUDE_USB_CAM_USBCAMWRAPPER_H_ */
