/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/


#include <cam_lib/usb_cam.h>
#include <cam_lib/ptgrey.h>

#define no_of_tags 25
int tagid[no_of_tags]= {0,1,2,3,4,24,25,26,27,28,48,49,50,51,52,72,73,74,75,76,96,97,98,99,100};

void drawRectangl(Mat& img,const Mat& points,int k){
  line(img, points.at<Point2d>(k,0),points.at<Point2d>(k+1,0), Scalar( 0, 0, 255 ),1);
  line(img, points.at<Point2d>(k+1,0),points.at<Point2d>(k+2,0), Scalar( 0, 255, 0 ),1);
  line(img, points.at<Point2d>(k+2,0),points.at<Point2d>(k+3,0), Scalar( 255, 0, 0 ),1);
  line(img, points.at<Point2d>(k+3,0),points.at<Point2d>(k,0), Scalar( 255, 0, 255 ),1);
}

void detection(CAM_BASE* obj,const CamTestOptions& opts, Point3d src_real[], Point3d shift[], TagDetector& detector, TagDetectionArray detections, const string& serial, const string save_path, vector<Mat>& rot, vector<Mat>& trans){
    Mat distCoeffs=Mat::zeros(4,1,CV_64FC1);
    namedWindow(serial,WINDOW_AUTOSIZE);
    size_t arysize=sizeof(tagid)/sizeof(int);
    int *end =tagid+arysize;

	while(1){
		int tag_count =0;
		vector<Mat> images;
		obj->retriveRectifiedImgs(images,opts);
		Mat displimg;
		rot.clear();
		trans.clear();

		for(int q=0;q<images.size();q++){
			Mat K = opts.K[q].clone();
			detector.process(images[q], Point2d(K.at<double>(0,2),K.at<double>(1,2)), detections);
			Mat show=images[q].clone();
			circle( show,Point(K.at<double>(0,2),K.at<double>(1,2)),1,Scalar( 0, 0, 255 ),-1,0);
			vector<Point2d> imgpt;
			vector<Point3d> wrldpt;

			if(detections.size()!=0){
		        bool tag_present =false;
		        for(size_t i=0;i<detections.size();i++){
		            int* loc=std::find(tagid, end, detections[i].id);
		            bool exists = loc != end;
		            if (exists){
		            	putText(show, to_string(detections[i].id), detections[i].p[0],FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0xff, 0x99, 0), 1); 
		                tag_present = true;
		                imgpt.push_back(detections[i].p[0]);
		                imgpt.push_back(detections[i].p[1]);
		                imgpt.push_back(detections[i].p[2]);
		                imgpt.push_back(detections[i].p[3]);

		                wrldpt.push_back(src_real[0]+shift[loc-tagid]);
		                wrldpt.push_back(src_real[1]+shift[loc-tagid]);
		                wrldpt.push_back(src_real[2]+shift[loc-tagid]);
		                wrldpt.push_back(src_real[3]+shift[loc-tagid]);
		            }  
		        }

		        if(tag_present){
		            Mat imgpt_mat(imgpt);
		            Mat wrldpt_mat(wrldpt);
		            Mat r, t;

		            solvePnP(wrldpt_mat, imgpt_mat, K, distCoeffs, r, t);
		            Rodrigues(r, r);

		            Point2d dst[imgpt_mat.rows];
		            Mat_<Point2d> dstmat(imgpt_mat.rows, 1, dst);
		            projectPoints(wrldpt_mat, r, t, K, distCoeffs, dstmat);


		            for (int a=0; a<imgpt_mat.rows; a=a+4){
		                drawRectangl(show, dstmat, a);
		            }

		            string campos;
		            if(q==0 & images.size()==2){
		            	campos = "usb_left_";
		            }
		            else if(q==1 & images.size()==2){
		            	campos = "usb_right_";
		            }
		            else{
		            	campos = "ptgrey_";
		            }
		            int tag_fin = imgpt.size()/4;
		            tag_count += tag_fin;

		            stringstream ss;
		            ss << tag_fin;
		            String text = campos+"no_of_tags_detected:"+ss.str();
		            cout<<ss.str()<<endl;
		            int fontface = FONT_HERSHEY_SIMPLEX;
		            double fontscale = 1.0;
		            putText(show, text, Point(0,25),fontface, fontscale, Scalar(0xff, 0x99, 0), 2); 
		            rot.push_back(r);
                	trans.push_back(t); 
		        }    
			} 
		    if(q==0){
            	displimg = show.clone();
            }
            else{
            	hconcat(displimg,show,displimg);
            }
	    }
	    imshow(serial,displimg);
    	cout<<"Press esc to save the calib params, else enter to continue..."<<endl;
    	char key=waitKey(0);
        if(key==27){
            imwrite(save_path+"/"+serial+".png",displimg);
            destroyAllWindows();
            break;
        }
    }
}

int main(){

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
	
    string tagfamily, config_path, save_path;
    double tag_size, tag_space;

    cout<<"Provide tag Family (default: Tag36h11): "<<endl;
    getline(cin,tagfamily);
    if(tagfamily.empty())
        tagfamily= "Tag36h11";

    cout<<"Provide tag size (default: 0.1706): "<<endl;
    string tag_siz;
    stringstream ts;
    getline(cin,tag_siz);
    if(tag_siz.empty())
        tag_siz="0.1706";
    ts << tag_siz;
    ts >> tag_size;

    cout<<"Provide tag spacing (default: 0.043): "<<endl;
    string tag_sp;
    stringstream tsp;
    getline(cin,tag_sp);
    if(tag_sp.empty())
        tag_sp="0.043";
    tsp << tag_sp;
    tsp >> tag_space;

    cout<<"Provide path to config files (default path:'../config/') : "<<endl;
    getline(cin,config_path);
    if(config_path.empty())
        config_path="../config/";

    cout<<"Provide path to save images(default path:'./calib_img/') : "<<endl;
    getline(cin,save_path);
    if(save_path.empty())
        save_path="./calib_img/";
    string cmd="mkdir -p "+save_path;
    system(cmd.c_str());

    Point3d src_real[4] = {Point3d(-0.5*tag_size, -0.5*tag_size, 0), Point3d( 0.5*tag_size, -0.5*tag_size, 0),
  	  Point3d( 0.5*tag_size,  0.5*tag_size, 0), Point3d(-0.5*tag_size,  0.5*tag_size, 0)};

	double spacing=tag_size+tag_space;

	Point3d shift[no_of_tags]={
	  Point3d(-2*(spacing), 2*(spacing), 0),Point3d(-spacing, 2*(spacing), 0),Point3d(0, 2*(spacing), 0),
	  Point3d(spacing, 2*(spacing), 0),Point3d(2*(spacing), 2*(spacing), 0),
	  Point3d(-2*(spacing), spacing, 0),Point3d(-spacing, spacing, 0),Point3d(0, spacing, 0),
	  Point3d(spacing, spacing, 0),Point3d(2*(spacing), spacing, 0),
	  Point3d(-2*(spacing), 0, 0),Point3d(-(spacing), 0, 0),Point3d(0, 0, 0),
	  Point3d(spacing, 0, 0),Point3d(2*(spacing), 0, 0),
	  Point3d(-2*(spacing), -spacing, 0),Point3d(-spacing, -spacing, 0),Point3d(0, -spacing, 0),
	  Point3d(spacing, -spacing, 0),Point3d(2*(spacing), -spacing, 0),
	  Point3d(-2*(spacing), -2*(spacing), 0),Point3d(-spacing, -2*(spacing), 0),Point3d(0, -2*(spacing), 0),
	  Point3d(spacing, -2*(spacing), 0),Point3d(2*(spacing), -2*(spacing), 0)};


	TagFamily family(tagfamily);
    family.setErrorRecoveryFraction(1);
    TagDetector detector(family);
    TagDetectionArray detections;

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
    string serial, filename;
    vector<Mat> rot, trans;
    string camtype;
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
	detection(cameras, opts, src_real, shift, detector, detections, serial, save_path,rot, trans);
    cameras->save_params(opts, rot, trans, config_path, serial);
    delete cameras;
    delete cam;
	return 0;
}