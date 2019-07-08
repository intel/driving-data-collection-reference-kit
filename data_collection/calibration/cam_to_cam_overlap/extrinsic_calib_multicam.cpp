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

void detection(CAM_BASE* obj[2],CamTestOptions opts[], Point3d src_real[],  Point3d shift[], TagDetector& detector, const vector<string>& serial, const string save_path, vector<Mat>& rot, vector<Mat>& trans){
    TagDetectionArray detections[2];
    Mat distCoeffs=Mat::zeros(4,1,CV_64FC1);
    namedWindow("images",CV_WINDOW_NORMAL);
    Mat rotation[2],translation[2];
    size_t arysize=sizeof(tagid)/sizeof(int);
    int *end =tagid+arysize;

    while(1){
        Mat images[2], display[2];
        for(int i=0;i< 2; i++){
            images[i] = obj[i]->retriveRectifiedLeft(opts[i]);
        }

        detector.process(images[0], Point2d(opts[0].K[0].at<double>(0,2),opts[0].K[0].at<double>(1,2)), detections[0]);
        detector.process(images[1], Point2d(opts[1].K[0].at<double>(0,2),opts[1].K[0].at<double>(1,2)), detections[1]);

        for(int i=0;i<2;i++){
            Mat show=images[i].clone();
            Mat K = opts[i].K[0].clone();
            Mat r, t;
            vector<Point2d> imgpt;
            vector<Point3d> wrldpt;
            if(detections[i].size()!=0){
                for(size_t j=0;j<detections[i].size();j++){
                    int* loc= find (tagid, end, detections[i][j].id);
                    if (loc != end){
                        putText(show, to_string(detections[i][j].id), detections[i][j].p[0],FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0xff, 0x99, 0), 1); 
                        for(size_t k=0;k<4;k++){
                            imgpt.push_back(detections[i][j].p[k]);
                            wrldpt.push_back(src_real[k]+shift[loc-tagid]);
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
                String text = "no_of_tags_detected: "+ss.str();
                int fontface = FONT_HERSHEY_SIMPLEX;
                double fontscale = 1.0;
                putText(show, text, Point(5*i,25),fontface, fontscale, Scalar(0xff, 0x99, 0), 2); 
            }
            rotation[i]=r;
            translation[i]=t;
            display[i]=show.clone();
        }

        Mat concat_img(display[0].rows,display[0].cols*2,CV_8UC3);
        display[0].copyTo(concat_img(Rect(0,0,display[0].cols,display[0].rows)));
        display[1].copyTo(concat_img(Rect(display[0].cols,0,display[0].cols,display[0].rows)));
        imshow("images",concat_img);
        cout<<"Press esc to save the calib params, else enter to continue..."<<endl;
        char key=waitKey(0);
        if(key==27){
            imwrite(save_path+"/"+serial[0]+".png",images[0]);
            imwrite(save_path+"/"+serial[1]+".png",images[1]);
            rot.push_back(rotation[0]);
            rot.push_back(rotation[1]);
            trans.push_back(translation[0]);
            trans.push_back(translation[1]);
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
    if(total_cams < 2){
      cout<<"provide total atleat 2 cam" <<endl;
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

    cam->display();

    string cam_choice[2];
    int camid[2];
    string cam_type[2];
    cam_type[0]="primary";
    cam_type[1]="secondary";

    for(int i=0;i<2;i++){
       
        cout<<"Select the "<<cam_type[i]<<" camera id (default id:0): "<<endl;
        getline(cin,cam_choice[i]);
        if(cam_choice[i].empty())
            cam_choice[i]="0";
        camid[i] = stoi(cam_choice[i]);
        if(!(cam_choice[i].find_first_not_of( "0123456789" ) == string::npos) || camid[0]>=total_cams){
            cout<<"Not a proper id"<<endl;
            delete cam;
            exit(1);
        }
    }

    
    vector<Mat> rot, trans;
    CamTestOptions opts[2];
    string  camtype, filename;
    vector<string> serial;
    CAM_BASE *cameras[2];
    for(int i=0;i<2;i++){
        if(camid[i] < ptgrey_connected){
            PTGRY ptcam;
            cameras[i] = new PTGRY;
            cameras[i]->init_cam(cam_choice[i]);
            cout<<"provide "<<cam_type[i]<<" camera yaml config file name without extension: "<<endl;
            getline(cin,filename);
            serial.push_back(filename);
        }
        else{
            USB_CAM usbcam;
            cameras[i] = new USB_CAM;
            cameras[i]->init_cam(cam->cam_names[camid[i]-ptgrey_connected]);
            cout<<"choose the "<<cam_type[i]<<" camera type (0-streo,1-mono)[default: stereo]: "<<endl;
            getline(cin,camtype);
            if(camtype.empty())
                camtype="0";
            static_cast<USB_CAM*>(cameras[i])->mono = stoi(camtype);
            if(!(camtype.find_first_not_of( "01" ) == string::npos)) {
                cout<<"Not a proper id"<<endl;
                delete cameras[i];
                delete cam; 
                exit(1);
            }
            cout<<"provide "<<cam_type[i]<<" camera yaml config file name without extension: "<<endl;
            getline(cin,filename);
            serial.push_back(filename);   
        }
        cameras[i]->readconfig_ref(serial[i], opts[i], config_path);
    }

    detection(cameras, opts, src_real, shift, detector, serial, save_path, rot, trans);
    cameras[1]->save_params_pair(opts, rot, trans, config_path, serial);
    delete cameras[0];
    delete cameras[1];

    delete cam;
    return 0;
}