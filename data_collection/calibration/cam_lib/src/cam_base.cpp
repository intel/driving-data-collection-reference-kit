/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

#include "cam_base.h"


CAM_BASE::CAM_BASE(){
  ptgrey_connected =10;
  usb_cam_connected =0;

}

unsigned int CAM_BASE::check_connected_ptgrey(){
	camInfo = new CameraInfo [ptgrey_connected];
    BusManager::DiscoverGigECameras(camInfo, &ptgrey_connected);
    return ptgrey_connected;
}

unsigned int CAM_BASE::check_connected_usb(){
    system("ls /dev/video* >  USB_CAM.txt 2>&1");
    cam_names.clear();
    string line;
    ifstream myfile ("USB_CAM.txt");
    if (myfile.is_open())
    {
      while ( getline (myfile,line) )
      {
        cam_names.push_back(line);
      }
      if(cam_names[0]!="/dev/video0"){
        cout << "No usb cameras"<<endl;
        return 0;
      } 
      myfile.close();
    }
    system("rm -rf USB_CAM.txt");
    usb_cam_connected = cam_names.size();
    return usb_cam_connected;
}

void CAM_BASE::display(){
	stringstream ss;
	for(int id = 0; id < ptgrey_connected; id++){
		string serial;
		cout<<"cam_id:"<< id <<endl;
		ostringstream ipAddress;
		ipAddress << (unsigned int)camInfo[id].ipAddress.octets[0] << "."
		    << (unsigned int)camInfo[id].ipAddress.octets[1] << "."
		    << (unsigned int)camInfo[id].ipAddress.octets[2] << "."
		    << (unsigned int)camInfo[id].ipAddress.octets[3];

		cout << "Camera model - " << camInfo[id].modelName << endl;
		cout << "Serial number - " << camInfo[id].serialNumber<<endl;
		cout<< "IP address - " << ipAddress.str()<<endl;
		cout<<"\n";
		ss<<camInfo[id].serialNumber;
		ss>>serial;
		cam_names.push_back(serial);
		ss.str("");
    }
    for(int i = ptgrey_connected; i< (ptgrey_connected+usb_cam_connected); i++){
      cout<<"cam_id:"<< i <<endl;
      cout<<"cam_name:"<<cam_names[i-ptgrey_connected]<<endl;
      cout<<"\n";
    }
}
