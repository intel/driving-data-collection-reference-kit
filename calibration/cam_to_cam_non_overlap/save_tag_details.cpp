/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/


#include <cam_lib/usb_cam.h>
#include <cam_lib/ptgrey.h>

static int print_help(){
	cout <<"Code to perform extrinsic calibration of camera with respect to ground. \n\n";
    cout <<"Usage:\n ./single_cam\n  -help = <to display usage>\n  -config_path = <path to configuration files of camera>\n";
    cout <<"  -tagyaml1=<file name of sheet1 tagdetails default=tagdetails1.yaml\n  -tagyaml2=<file name of sheet2 tagdetails default=tagdetails2.yaml\n  -save_path=<to save the params in given path with same name as original yaml\n";
    return 0;
}

void drawRectangl(Mat& img,const Mat& points,int k){
  line(img, points.at<Point2d>(k,0),points.at<Point2d>(k+1,0), Scalar( 0, 0, 255 ),1);
  line(img, points.at<Point2d>(k+1,0),points.at<Point2d>(k+2,0), Scalar( 0, 255, 0 ),1);
  line(img, points.at<Point2d>(k+2,0),points.at<Point2d>(k+3,0), Scalar( 255, 0, 0 ),1);
  line(img, points.at<Point2d>(k+3,0),points.at<Point2d>(k,0), Scalar( 255, 0, 255 ),1);
}

void detection(CAM_BASE* obj,const CamTestOptions opts,const vector<Point3d> *src_real, const vector<Point3d> *shift,const vector<int> *tagids, const string *tagfamily, const string serial, const string save_path)
{
	TagFamily family1(tagfamily[0]);
	TagFamily family2(tagfamily[1]);
    family1.setErrorRecoveryFraction(1);
    TagDetector detector1(family1);
    family2.setErrorRecoveryFraction(1);
    TagDetector detector2(family2);
    TagDetectionArray detections[2];

    Mat distCoeffs=Mat::zeros(4,1,CV_64FC1);
    namedWindow("images",CV_WINDOW_NORMAL);
    vector<Point2d> fin_imgpt[2];
	vector<Point3d> fin_wrldpt[2];
	Mat rot[2],trans[2];

    FileStorage fs_save(save_path+"data.yaml", FileStorage::WRITE);
    fs_save << "K" <<opts.K[0].clone();
    int img_count;
    while(1){
    	Mat images;
    	images = obj->retriveRectifiedLeft(opts);
    	Mat show=images.clone();
    	
    	Mat K = opts.K[0].clone();
    	detector1.process(images, Point2d(K.at<double>(0,2),K.at<double>(1,2)), detections[0]);
    	detector2.process(images, Point2d(K.at<double>(0,2),K.at<double>(1,2)), detections[1]);
		

		for(int i=0;i<2;i++){
			Mat r, t;
			vector<Point2d> imgpt;
			vector<Point3d> wrldpt;
			if(detections[i].size()!=0){
		        for(size_t j=0;j<detections[i].size();j++){
		        	std::vector<int>::const_iterator it;
		        	it = find (tagids[i].begin(), tagids[i].end(), detections[i][j].id);
		            if (it!=tagids[i].end()){
		            	putText(show, to_string(detections[i][j].id), detections[i][j].p[0],FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0xff, 0x99, 0), 1); 
		                for(size_t k=0;k<4;k++){
	                        imgpt.push_back(detections[i][j].p[k]);
	                        wrldpt.push_back((src_real[i][k]+shift[i][it-tagids[i].begin()]));
	                    }
		            }  
		        }
		    }

	        if(imgpt.size()>0){
	            Mat imgpt_mat(imgpt);
	            Mat wrldpt_mat(wrldpt);
	            
	            solvePnP(wrldpt_mat, imgpt_mat, K, distCoeffs, r, t);
	            Rodrigues(r, r);

	            Point2d dst[imgpt_mat.rows];
	            Mat_<Point2d> dstmat(imgpt_mat.rows, 1, dst);

	            projectPoints(wrldpt_mat, r, t, K, distCoeffs, dstmat);

	            for (int a=0; a<imgpt_mat.rows; a=a+4){
	                drawRectangl(show, dstmat, a);
	            }

	            int tag_fin = imgpt.size()/4;

	            stringstream ss;
	            ss << tag_fin;
	            String text = "no_of_tags_detected in pattern"+to_string(i)+": "+ss.str();
	            int fontface = FONT_HERSHEY_SIMPLEX;
	            double fontscale = 0.5;
	            putText(show, text, Point(5,20*(i+1)),fontface, fontscale, Scalar(0xff, 0x99, 0), 1); 
	        }
	        fin_imgpt[i] = imgpt;
	        fin_wrldpt[i] = wrldpt;
	        rot[i]=r;
	        trans[i]=t;
	    }

	    imshow("images",show);
		cout<<"Press esc to save the calib params, else enter to continue..."<<endl;
		char key=waitKey(0);
		if(key=='s'){
			imwrite(save_path+serial+"_"+to_string(img_count)+".png",images);
			fs_save << ("pose"+to_string(img_count)) << "{";
			fs_save << "imgpt1" << fin_imgpt[0];
			fs_save << "wrldpt1" << fin_wrldpt[0];
			fs_save << "imgpt2" << fin_imgpt[1];
			fs_save << "wrldpt2" << fin_wrldpt[1];

			fs_save << "rot1" << rot[0];
			fs_save << "rot2" << rot[1];
			fs_save << "trans1" << trans[0];
			fs_save << "trans2" << trans[1];
			Mat p1 = rot[0](Rect(0,0,2,3)).clone();
            hconcat(p1, trans[0] , p1);
            Mat p2 = rot[1](Rect(0,0,2,3)).clone();
            hconcat(p2, trans[1] , p2);
            fs_save << "Proj1" << p1;
            fs_save << "proj2" << p2;
			fs_save << "}";
			cout<<"saved image "<<to_string(img_count)<<endl;
			img_count++;
		}
	    if(key==27){
	    	fs_save << "no_of_patterns" << img_count;
	    	fs_save.release();
	        destroyAllWindows();
	        break;
	    }
	}
}

int main(int argc, char** argv) {
	CommandLineParser parser(argc, argv, "{config_path|../config/|}{tagyaml1|tagdetails1.yaml|}{tagyaml2|tagdetails2.yaml|}{save_path|../images/|}{help||}");
	if (parser.has("help"))
		return print_help();

	string config_path = parser.get<string>("config_path");
	string tagyaml1 = parser.get<string>("tagyaml1");
	string tagyaml2 = parser.get<string>("tagyaml2");
	string save_path = parser.get<string>("save_path");
	if (!parser.check())
	{
	    parser.printErrors();
	    return -1;
	}
	string cmd = "mkdir -p "+save_path;
	system(cmd.c_str());

	vector<int>  tagids[2];
    vector<Point3d>  wrldpt[2], shift[2];
    string tagfamily[2];
	FileStorage tagdetail1(config_path+tagyaml1,FileStorage::READ);
	tagdetail1["tagid"]  >> tagids[0];
	tagdetail1["wrldpt"] >> wrldpt[0];
 	tagdetail1["shift"]  >> shift[0];
 	tagdetail1["tagfamily"]  >> tagfamily[0];
 	tagdetail1.release();

	FileStorage tagdetail2(config_path+tagyaml2,FileStorage::READ);
	tagdetail2["tagid"]  >> tagids[1];
	tagdetail2["wrldpt"] >> wrldpt[1];
 	tagdetail2["shift"]  >> shift[1];
 	tagdetail2["tagfamily"]  >> tagfamily[1];
 	tagdetail2.release();
 	

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

    CamTestOptions opts;
    string  camtype, filename;
    string serial;
    CAM_BASE *cameras;
    if(camid < ptgrey_connected){
        PTGRY ptcam;
        cameras = new PTGRY;
        cameras->init_cam(cam_choice);
        cout<<"provide yaml config file name without extension: "<<endl;
        getline(cin,filename);
        serial=filename;
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
        getline(cin,filename);
        serial=filename;   
    }
	cameras->readconfig_ref(serial, opts, config_path);
    detection(cameras, opts, wrldpt, shift, tagids, tagfamily, serial, save_path);
	delete cameras;
    delete cam;
    return 0;
}
