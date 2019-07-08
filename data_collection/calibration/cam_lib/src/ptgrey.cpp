/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/


#include "ptgrey.h"

void PTGRY::check_mode(int mode_value){
    switch(mode_value){
        case 0:
            camera_mode = MODE_0;
            break;
        case 1:
            camera_mode = MODE_1;
            break;
        case 2:
            camera_mode = MODE_2;
            break;
        case 3:
            camera_mode = MODE_3;
            break;
        case 4:
            camera_mode = MODE_4;
            break;
        case 5:
            camera_mode = MODE_5;
            break;
        case 6:
            camera_mode = MODE_6;
            break;
        case 7:
            camera_mode = MODE_7;
            break;
        case 8:
            camera_mode = MODE_8;
            break;
        case 9:
            camera_mode = MODE_9;
            break;
        case 10:
            camera_mode = MODE_10;
            break;
        case 11:
            camera_mode = MODE_11;
            break;
        case 12:
            camera_mode = MODE_12;
            break;
        case 13:
            camera_mode = MODE_13;
            break;
        case 14:
            camera_mode = MODE_14;
            break;
        case 15:
            camera_mode = MODE_15;
            break;
        case 16:
            camera_mode = MODE_16;
            break;
        case 17:
            camera_mode = MODE_17;
            break;
        case 18:
            camera_mode = MODE_18;
            break;
        case 19:
            camera_mode = MODE_19;
            break;
        case 20:
            camera_mode = MODE_20;
            break;
        case 21:
            camera_mode = MODE_21;
            break;
        case 22:
            camera_mode = MODE_22;
            break;
        case 23:
            camera_mode = MODE_23;
            break;
        case 24:
            camera_mode = MODE_24;
            break;
        case 25:
            camera_mode = MODE_25;
            break;
        case 26:
            camera_mode = MODE_26;
            break;
        case 27:
            camera_mode = MODE_27;
            break;
        case 28:
            camera_mode = MODE_28;
            break;
        case 29:
            camera_mode = MODE_29;
            break;
        case 30:
            camera_mode = MODE_30;
            break;
        case 31:
            camera_mode = MODE_31;
            break;
        default:
            cerr<<"Assertion failed in file " + string(__FILE__)+ ",at line " + to_string(__LINE__)+"\nNot a proper mode of operation for point grey cameras.\n";
            abort();
            break;
    }
};

void PTGRY::init_cam(const string& cam_choice){
    int camid = stoi(cam_choice);
	PGRGuid guid;
    busMgr.GetCameraFromIndex(camid, &guid);
    InterfaceType interfaceType;
    busMgr.GetInterfaceTypeFromGuid(&guid, &interfaceType); 

    if (interfaceType != INTERFACE_GIGE)
    {            
        cout<<"Connect Gige Camera"<<endl;
        exit(EXIT_FAILURE);
    }

    string mode_value;
    cout<<"provide mode of operation of point grey camera [mode range from 0-31, default-0]: "<<endl;
    getline(cin,mode_value);
    if(mode_value.empty())
        mode_value="0";
    int value=stoi(mode_value);
    check_mode(value);
    cam.Connect(&guid);
    cam.SetGigEImagingMode(MODE_0);
    GigEImageSettingsInfo imageSettingsInfo;
    cam.GetGigEImageSettingsInfo(&imageSettingsInfo);
    GigEImageSettings imageSettings;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.height = imageSettingsInfo.maxHeight;
    imageSettings.width = imageSettingsInfo.maxWidth;
    imageSettings.pixelFormat = PIXEL_FORMAT_RGB8;
    cam.SetGigEImageSettings(&imageSettings);
    cam.StartCapture();
}

Mat PTGRY::retriveImg(){
    cam.RetrieveBuffer(&rawImage);
    Image* raw = &rawImage;
    char* data=(char*)raw->GetData();
    Mat img(raw->GetRows(),raw->GetCols(),CV_8UC3);
    int count=0;
    int rows=raw->GetRows();
    int cols=raw->GetCols();
    for(int j=0;j<rows;j++){
        for(int q=0;q<cols;q++){
            img.at<Vec3b>(j,q)[0]=data[count+2];
            img.at<Vec3b>(j,q)[1]=data[count+1];
            img.at<Vec3b>(j,q)[2]=data[count];
            count=count+3;
        }
    }
    cout<<img.size()<<endl;
    return img;
}

void PTGRY::readconfig_ref(const string& serial,CamTestOptions& opts, const string & config_path){
    FileStorage fs(config_path+serial+".yaml",FileStorage::READ);
    Mat intrinsic;
    if(!fs.isOpened())
        cout<<"unable to open config file"<<endl;
    fs["K"] >> intrinsic;
    fs ["world_center"] >> opts.world_center;
    fs ["project_to_reference_cam"] >> opts.proj_to_refcam;
    fs ["projection_to_world"] >> opts.proj_to_wrld;
    fs ["map1"] >> opts.map1;
    fs ["map2"] >> opts.map2;
    fs.release();
    opts.K.push_back(intrinsic);
}

Mat PTGRY::retriveRectifiedLeft(CamTestOptions opts){
    Mat img = retriveImg();
    Mat rview;
    remap(img,rview,opts.map1,opts.map2,INTER_LINEAR);
    return rview;
}

void PTGRY::retriveRectifiedImgs(vector<Mat>& img, CamTestOptions opts){
    img.clear();
    Mat orginal = retriveImg();
    Mat rview;
    remap(orginal,rview,opts.map1,opts.map2,INTER_LINEAR);
    img.push_back(rview);
}

void PTGRY::save_params(CamTestOptions opts, const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const string& serial){
    Mat P;
    rot[0](Rect(0,0,2,3)).copyTo(P);
    hconcat(P,trans[0],P);
    P=opts.K[0]*P;
    invert(P,opts.proj_to_wrld,DECOMP_SVD);
    opts.world_center = -rot[0].t()*trans[0];
    opts.proj_to_refcam = Mat::eye(3,3,CV_64FC1);
    FileStorage fs(config_path+"/"+serial+".yaml",FileStorage::WRITE);
    //Camera params.
    fs << "camera" << serial;
    fs << "K" << opts.K[0];
    fs << "map1" << opts.map1;
    fs << "map2" << opts.map2;
    fs << "world_center" << opts.world_center;
    fs << "rotation_wrt_wrld" << rot[0];
    fs << "translation_wrt_wrld" << trans[0];
    fs << "project_to_reference_cam" << opts.proj_to_refcam;
    fs << "projection_to_world" << opts.proj_to_wrld;
    fs.release();
}

void PTGRY::save_params_pair(CamTestOptions opts[], const vector<Mat>& rot, const vector<Mat>& trans, const string& config_path, const vector<string>& serial){
    
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
    fs << "camera" << serial[1];
    fs << "K" << opts[1].K[0];
    fs << "map1" << opts[1].map1;
    fs << "map2" << opts[1].map2;
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