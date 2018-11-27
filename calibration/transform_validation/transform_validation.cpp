/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/


#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

void new_coord(const Mat& fin_proj,int rows, int cols, int &new_sx, int &new_sy, int &new_ex, int &new_ey){
    double scale;
    Mat poi, x1;
    vector<double> xloc, yloc;
    poi = (Mat_<double>(3, 1) << 0, 0, 1);
    x1 = fin_proj*poi;
    scale = 1 / x1.at<double>(2, 0);
    x1 = scale*x1;
    xloc.push_back(x1.at<double>(0,0));
    yloc.push_back(x1.at<double>(1,0));

    poi = (Mat_<double>(3, 1) << 0, rows, 1);
    x1 = fin_proj*poi;
    scale = 1 / x1.at<double>(2, 0);
    x1 = scale*x1;
    xloc.push_back(x1.at<double>(0,0));
    yloc.push_back(x1.at<double>(1,0));

     poi = (Mat_<double>(3, 1) << cols, 0, 1);
    x1 = fin_proj*poi;
    scale = 1 / x1.at<double>(2, 0);
    x1 = scale*x1;
    xloc.push_back(x1.at<double>(0,0));
    yloc.push_back(x1.at<double>(1,0));

    poi = (Mat_<double>(3, 1) << cols, rows, 1);
    x1 = fin_proj*poi;
    scale = 1 / x1.at<double>(2, 0);
    x1 = scale*x1;
    xloc.push_back(x1.at<double>(0,0));
    yloc.push_back(x1.at<double>(1,0));

    double start_x=xloc[0], end_x=xloc[0], start_y=yloc[0], end_y=yloc[0];

    for(int i=1; i<xloc.size(); i++){
        start_x = start_x < xloc[i] ? start_x : xloc[i] ;
        end_x   = end_x   > xloc[i] ? end_x   : xloc[i] ;
        start_y = start_y < yloc[i] ? start_y : yloc[i] ;
        end_y   = end_y   > yloc[i] ? end_y   : yloc[i];
    }

    new_sx = (0 < start_x) ? 0 : start_x;
    new_sy = (0 < start_y) ? 0 : start_y;

    new_ex = (cols > end_x) ? cols : end_x;
    new_ey = (rows > end_y) ? rows : end_y;
}

void printUsage(){
    cout <<
        "Image stiching\n\n"
        "./stitching --ref_image [ref_img_path] --secondary_image [secondary_img_path] --projection_yaml [yaml_path] --result_path [saving_path]\n\n"
        "  --ref_image\n"
        "      provide rectified reference image path\n"
        "  --secondary_image \n"
        "      provide rectified secondary image path\n"
        "  --projection_yaml \n"
        "     provide yaml path which gives relation between secondary_image to reference image\n"
        "  --result_path\n"
        "      Type of estimator used for transformation estimation.\n\n";
}

int main(int argc, char* argv[])
{
    Mat img_ref, img1;
    Mat proj1;
    string result_path ="./";
    if (argc == 1)
    {
        printUsage();
        return -1;
    }
    for (int i = 1; i < argc; ++i)
    {
        if (string(argv[i]) == "--help")
        {
            printUsage();
            return -1;
        }
        else if (string(argv[i]) == "--ref_image")
        {
            if(argv[i+1]){
                img_ref = imread(argv[i+1]);
                i++;
            }
            else{
                cout<<"provide rectified reference image path"<<endl;
                return -1;
            }
           
        }
        else if (string(argv[i]) == "--secondary_image")
        {
            if(argv[i+1]){
                img1 = imread(argv[i+1]);
                i++;
            }
            else{
                cout<<"provide rectified secondary image path"<<endl;
                return -1;
            }
        }
        else if (string(argv[i]) == "--projection_yaml")
        {
            if(argv[i+1]){
                FileStorage fs_ext(argv[i+1], FileStorage::READ);
                fs_ext["project_to_reference_cam"] >> proj1;
                i++;
            }
            else{
                cout<<"provide yaml path which gives relation between secondary_image to reference image"<<endl;
                return -1;
            }
        }
        else if (string(argv[i]) == "--result_path")
        {
            result_path = argv[i+1];
            i++;
        }
    }
    if(img_ref.empty()){
        cout<<"provide reference image path"<<endl;
        return -1;
    }

    Mat invp;
    if(!img1.empty() & !proj1.empty()){
        invert(proj1,invp,DECOMP_SVD);
    }
    else{
        if(img1.empty()){
            cout<<"provide rectified secondary image path"<<endl;
        }
        if(proj1.empty()){
             cout<<"provide yaml path which gives relation between secondary_image to reference image"<<endl;
        }
        return -1;
    }

    
    int new_sx, new_sy, new_ex, new_ey;
    new_coord(proj1,img1.rows, img1.cols, new_sx, new_sy, new_ex, new_ey);
    
    Mat poi,x1;
    double scale;

    Mat stitch=Mat::zeros((new_ey-new_sy), (new_ex-new_sx), img1.type());
    for(int j=0;j<stitch.rows;j++){
        for(int k=0;k<stitch.cols;k++){
            if((j+new_sy) > 0 & (k+new_sx) > 0 & (j+new_sy) < img1.rows & (k+new_sx) < img1.cols){
                stitch.at<Vec3b>(j,k) = img_ref.at<Vec3b>((j+new_sy), (k+new_sx));
            }
            else{
                poi = (Mat_<double>(3, 1) << (k+new_sx), (j+new_sy), 1);
                x1 = invp*poi;
                scale = 1 / x1.at<double>(2, 0);
                x1 = scale*x1;
                int x =  int(x1.at<double>(0, 0));
                int y = int(x1.at<double>(1, 0));
                if(x<0 | y<0 | x>img1.cols | y >img1.rows)
                    continue;
                stitch.at<Vec3b>(j,k) = img1.at<Vec3b>(y,x);
            }     
        }
    }
       
    namedWindow("final", CV_WINDOW_NORMAL);
    imwrite(result_path+"/stich_res.png",stitch);
    imshow("final", stitch);
    waitKey(0);
    return 0;
}
