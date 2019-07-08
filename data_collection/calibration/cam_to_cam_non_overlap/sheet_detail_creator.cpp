/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/


#include <cam_lib/usb_cam.h>
#include <cam_lib/ptgrey.h>


static int print_help(){
	cout <<"Code asks user input to determine world coordinates and stores the details in yaml file at config_path. \n\n";
    cout <<"Usage:\n ./sheat_detail_creator\n  -help = <to display usage>\n  -tsize = <tag_size of apriltag>\n  -tspace = <spacing between tags>\n";
    cout <<"  -r = <no of rows tags arranged on sheet>\n  -c = <no of cols tags arranged on sheet>\n  -id_count = <No of tags present on the sheet>\n";
    cout <<"  -tagfamily = <family of apriltags used default=Tag36h11>\n  -config_path = <path to configuration files of camera>\n";
    return 0;
}

int main(int argc, char** argv) {
	CommandLineParser parser(argc, argv, "{tsize||}{tspace||}{r||}{c||}{id_count||}{help||}{tagfamily|Tag36h11|}{config_path|../config/|}");
	if (parser.has("help"))
		return print_help();

	float tag_size = parser.get<float>("tsize");
	float tag_space = parser.get<float>("tspace");
	int rows = parser.get<int>("r");
	int cols = parser.get<int>("c");
	int id_count = parser.get<int>("id_count");
	string config_path = parser.get<string>("config_path");
	string tagfamily = parser.get<string>("tagfamily");
	float spacing = tag_size + tag_space;
	if (!parser.check())
	{
	    parser.printErrors();
	    return -1;
	}

	TagFamily family(tagfamily);
    family.setErrorRecoveryFraction(1);
    TagDetector detector(family);
    TagDetectionArray detections;

	CAM_BASE *cam;
    cam = new CAM_BASE;
    unsigned int ptgrey_connected = cam->check_connected_ptgrey();
    unsigned int usb_cam_connected = cam->check_connected_usb();

    int total_cams =usb_cam_connected + ptgrey_connected;
    if(total_cams < 1){
      cout<<"provide total atleat 1 cam" <<endl;
      delete cam;
      return 0;
    }
 	cam->display();

 	string cam_choice;
    int camid;
    CamTestOptions opts;

    cout<<"Select the camera id (default id:0): "<<endl;
    getline(cin,cam_choice);
    if(cam_choice.empty())
        cam_choice="0";
    camid = stoi(cam_choice);
    if(!(cam_choice.find_first_not_of( "0123456789" ) == string::npos) || camid>=total_cams){
        cout<<"Not a proper id"<<endl;
        delete cam;
        exit(1);
    }
    CAM_BASE *cameras;
    string serial, camtype;
    if(camid < ptgrey_connected){
        PTGRY ptcam;
        cameras = new PTGRY;
        cameras->init_cam(cam_choice);
        cout<<"provide yaml config file name without extension: "<<endl;
        getline(cin,serial);
    }
    else{
        USB_CAM usbcam;
        cameras = new USB_CAM;
        cameras->init_cam(cam->cam_names[camid-ptgrey_connected]);
        cout<<"choose the cam type (0-streo,1-mono)[default: stereo]: "<<endl;
        getline(cin,camtype);
        if(camtype.empty())
            camtype="0";
        static_cast<USB_CAM*>(cameras)->mono = stoi(camtype);
        if(!(camtype.find_first_not_of( "01" ) == string::npos)) {
            cout<<"Not a proper id"<<endl;
            delete cameras;
            delete cam;
            exit(1);
        }
        cout<<"provide yaml config file name without extension: "<<endl;
        getline(cin,serial);
        
    }
    cameras->readconfig_ref(serial, opts, config_path);
    Mat distCoeffs=Mat::zeros(4,1,CV_64FC1);
    namedWindow(serial,WINDOW_AUTOSIZE);
    Mat images;

   while(1){
        images=cameras->retriveRectifiedLeft(opts);
        Mat K = opts.K[0].clone();
        detector.process(images, Point2d(K.at<double>(0,2),K.at<double>(1,2)), detections);
        Mat show=images.clone();
        vector <int> tagid;
	
        if(detections.size()!=0){
            
            for(size_t i=0;i<detections.size();i++){
                
                tagid.push_back(detections[i].id);
                putText(show, to_string(detections[i].id), detections[i].p[0],FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0xff, 0x99, 0), 1);
                circle( show,detections[i].p[0],1,Scalar( 0, 0, 255 ),-1,0);
                circle( show,detections[i].p[1],1,Scalar( 0, 255, 255 ),-1,0);
                circle( show,detections[i].p[2],1,Scalar( 255, 255, 0 ),-1,0);
                circle( show,detections[i].p[3],1,Scalar( 0, 255, 0 ),-1,0);
            }
            
        }

        imshow(serial,show);
        if(detections.size()==id_count){
        	cout<<"ids: ";
	        sort(tagid.begin(),tagid.end());
	        for(size_t i=0;i<tagid.size();i++){
	        	cout<<tagid[i]<<" ";
	        }
	        cout<<"\n";
           
            cout<<"Press esc to save tag details if all tags detected properly, else enter to continue..."<<endl;
            char key=waitKey(0);
            if(key==27){
                
                destroyWindow(serial);
            	vector<Point3d> wrld,shift;
            	for(int J=0;J<rows;J++){
			    	for(int K=0;K<cols;K++){
			    		shift.push_back(Point3d(K*spacing, -J*spacing, 0));
			    	}
			    }
			    wrld.push_back(Point3d(-0.5*tag_size, -0.5*tag_size, 0));
			    wrld.push_back(Point3d( 0.5*tag_size, -0.5*tag_size, 0));
			    wrld.push_back(Point3d( 0.5*tag_size,  0.5*tag_size, 0));
			    wrld.push_back(Point3d(-0.5*tag_size,  0.5*tag_size, 0));
            	FileStorage fs(config_path+"tagdetails.yaml",FileStorage::WRITE);
                fs << "tagfamily" << tagfamily;
            	fs << "tag_size" << tag_size;
            	fs << "tag_space" << tag_space;
                fs << "id_count" << id_count;
            	fs << "tagid" << tagid;
				fs << "wrldpt" << wrld;
				fs << "shift" << shift;
                fs.release();
                break;
            }
        }
        else
            waitKey(33);
    }
    cameras->closecamera();
    delete cameras;
    delete cam;
    return 0;
}
