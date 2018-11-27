/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/


#include <iostream>
#include <random>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

typedef struct Tagparams{
    int pattern_count;
    Mat K1, K2;
    vector<Mat> proj[2];
    vector<vector<Point3d> >  wrld[2];
    vector<vector<Point2d> > imgpt[2];
}Tagparams;


void readtagdetails(const string &path,Tagparams& tparams){
    FileStorage fs(path,FileStorage::READ);
    fs["K1"] >> tparams.K1;
    fs["K2"] >> tparams.K2;
    fs["no_of_patterns"] >> tparams.pattern_count;
    FileNode temp_ID;
    for(int i=0;i<tparams.pattern_count;i++){
        vector<Point2d> ipt[2];
        vector<Point3d> wpt[2];
        Mat proj[2];

        temp_ID=fs["tagpattern"+to_string(i)];
        temp_ID["imgpt1"] >> ipt[0];
        temp_ID["imgpt2"] >> ipt[1];
        temp_ID["wrldpt1"] >> wpt[0];
        temp_ID["wrldpt2"] >> wpt[1];
        temp_ID["Proj1"] >> proj[0];
        temp_ID["Proj2"] >> proj[1];
        for(int j=0;j<2;j++){
            tparams.imgpt[j].push_back(ipt[j]);
            tparams.wrld[j].push_back(wpt[j]);
            tparams.proj[j].push_back(proj[j]);
        }
    }
    fs.release();   
}

void calc_jacobian(Mat& jac, Mat& param, Mat& tag_rel, Mat& K,vector<Mat>& P, vector<vector<Point3d> >& wrld) {
    jac=Mat::zeros(wrld.size()*wrld[0].size()*2, 9, CV_64F);
    double *M =param.ptr<double>(0);
    int count =0;
    for (auto j = 0; j < wrld.size(); j++) {
        Mat proj = P[j]* tag_rel;
        double *ptr_Proj = proj.ptr<double>(0);
        for(auto k =0; k< wrld[j].size();k++){
            double X = ptr_Proj[0]*wrld[j][k].x + ptr_Proj[1]*wrld[j][k].y + ptr_Proj[2];
            double Y = ptr_Proj[3]*wrld[j][k].x + ptr_Proj[4]*wrld[j][k].y + ptr_Proj[5];
            double Z = ptr_Proj[6]*wrld[j][k].x + ptr_Proj[7]*wrld[j][k].y + ptr_Proj[8];
            double numer_x = M[0] * X + M[1] * Y + M[2] * Z ;
            double numer_y = M[3] * X + M[4] * Y + M[5] * Z ;
            double denom   = M[6] * X + M[7] * Y + M[8] * Z ;
            for(auto q = 0; q <2; q++){
                double numer;
                if(q==0)
                    numer = numer_x;
                else
                    numer = numer_y;
                double fx = K.at<double>(q,q);
                jac.at<double>(count  , 0) = (fx*X) / denom;
                jac.at<double>(count  , 1) = (fx*Y) / denom;
                jac.at<double>(count  , 2) = (fx*Z) / denom;
                jac.at<double>(count  , 3) = 0;
                jac.at<double>(count  , 4) = 0;
                jac.at<double>(count  , 5) = 0;
                jac.at<double>(count  , 6) = -(fx*X*numer) / (denom*denom);
                jac.at<double>(count  , 7) = -(fx*Y*numer) / (denom*denom);
                jac.at<double>(count  , 8) = -(fx*Z*numer) / (denom*denom);
                count ++;
            }
        }
    }
}

void calc_error(Mat& err,  Mat& param, Mat& tag_rel, Mat& K,vector<Mat>& P, vector<vector<Point3d> >& wrld, vector<vector<Point2d> >& imgpt){
    err.create(wrld.size()*wrld[0].size()*2, 1, CV_64F);
    double *M =param.ptr<double>(0);
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    int count =0;
    for (auto j = 0; j < wrld.size(); j++) {

        Mat proj = P[j]* tag_rel;
        double *ptr_Proj = proj.ptr<double>(0);
        for(auto k =0; k< wrld[j].size();k++){
            double X = ptr_Proj[0]*wrld[j][k].x + ptr_Proj[1]*wrld[j][k].y + ptr_Proj[2];
            double Y = ptr_Proj[3]*wrld[j][k].x + ptr_Proj[4]*wrld[j][k].y + ptr_Proj[5];
            double Z = ptr_Proj[6]*wrld[j][k].x + ptr_Proj[7]*wrld[j][k].y + ptr_Proj[8];
            double numer_x = M[0] * X + M[1] * Y + M[2] * Z ;
            double numer_y = M[3] * X + M[4] * Y + M[5] * Z ;
            double denom   = M[6] * X + M[7] * Y + M[8] * Z ;
            double x_hat = (fx*numer_x)/denom + cx;
            double y_hat = (fy*numer_y)/denom + cy;
            err.at<double>(count  , 0) = x_hat - imgpt[j][k].x ;
            err.at<double>(count+1, 0) = y_hat - imgpt[j][k].y ;
            count += 2;
        }
    }

}

void calc_error_full(Mat& err,  Mat& param, Mat& tag_rel, Mat& K1, Mat& K2, vector<Mat>* P, vector<vector<Point3d> >* wrld, vector<vector<Point2d> >* imgpt){
    err.create(wrld[0].size()*wrld[0][0].size()*4, 1, CV_64F);
    
    double *M = param.ptr<double>(0);
    for (auto j = 0; j < wrld[0].size(); j++) {
        Mat proj1 = P[0][j]*tag_rel.inv(DECOMP_SVD);
        Mat proj2 = P[1][j]*tag_rel;
        double *ptr_Proj1 = proj1.ptr<double>(0);
        double *ptr_Proj2 = proj2.ptr<double>(0);

        for(auto k =0; k< wrld[0][j].size();k++){

            double X1 = ptr_Proj2[0]*wrld[0][j][k].x + ptr_Proj2[1]*wrld[0][j][k].y + ptr_Proj2[2];
            double Y1 = ptr_Proj2[3]*wrld[0][j][k].x + ptr_Proj2[4]*wrld[0][j][k].y + ptr_Proj2[5];
            double Z1 = ptr_Proj2[6]*wrld[0][j][k].x + ptr_Proj2[7]*wrld[0][j][k].y + ptr_Proj2[8];
            double numer_x1 = M[0] * X1 + M[1] * Y1 + M[2] * Z1 ;
            double numer_y1 = M[3] * X1 + M[4] * Y1 + M[5] * Z1 ;
            double denom1   = M[6] * X1 + M[7] * Y1 + M[8] * Z1 ;
            double x_hat1 = (K1.at<double>(0,0)*numer_x1)/denom1 + K1.at<double>(0,2);
            double y_hat1 = (K1.at<double>(1,1)*numer_y1)/denom1 + K1.at<double>(1,2);

            double X2 = ptr_Proj1[0]*wrld[1][j][k].x + ptr_Proj1[1]*wrld[1][j][k].y + ptr_Proj1[2];
            double Y2 = ptr_Proj1[3]*wrld[1][j][k].x + ptr_Proj1[4]*wrld[1][j][k].y + ptr_Proj1[5];
            double Z2 = ptr_Proj1[6]*wrld[1][j][k].x + ptr_Proj1[7]*wrld[1][j][k].y + ptr_Proj1[8];
            double d1 = (M[4]*M[8] - M[5]*M[7])*X2 + (M[2]*M[7] - M[1]*M[8])*Y2 + (M[1]*M[5] - M[2]*M[4])*Z2;
            double d2 = (M[5]*M[6] - M[3]*M[8])*X2 + (M[0]*M[8] - M[2]*M[6])*Y2 + (M[2]*M[3] - M[0]*M[5])*Z2;
            double d3 = (M[3]*M[7] - M[4]*M[6])*X2 + (M[1]*M[6] - M[0]*M[7])*Y2 + (M[0]*M[4] - M[1]*M[3])*Z2;
            double numer_x2 = ptr_Proj1[0] * d1 + ptr_Proj1[1] * d2 + ptr_Proj1[2] * d3 ;
            double numer_y2 = ptr_Proj1[3] * d1 + ptr_Proj1[4] * d2 + ptr_Proj1[5] * d3 ;
            double denom2   = ptr_Proj1[6] * d1 + ptr_Proj1[7] * d2 + ptr_Proj1[8] * d3 ;
            double x_hat2 = (K2.at<double>(0,0)*numer_x2)/denom2 + K2.at<double>(0,2);
            double y_hat2 = (K2.at<double>(1,1)*numer_y2)/denom2 + K2.at<double>(0,2);

            err.at<double>(4*j*k+0, 0) = x_hat1 - imgpt[0][j][k].x ;
            err.at<double>(4*j*k+1, 0) = y_hat1 - imgpt[0][j][k].y ;
            err.at<double>(4*j*k+2, 0) = x_hat2 - imgpt[1][j][k].x ;
            err.at<double>(4*j*k+3, 0) = y_hat2 - imgpt[1][j][k].y ;
        }
    }
}

int main() {
    Tagparams tparams;
    readtagdetails("../tag_cam/data1.yaml", tparams);
    FileStorage fs("tag_relation.yaml", FileStorage::READ);
    Mat tag_rel;
    fs["tag_rel"] >> tag_rel; 
    fs.release();
    auto nparams = 9;
    auto nerrs = tparams.imgpt[0].size()*tparams.imgpt[0][0].size()*2;
    auto criteria = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 1000, DBL_EPSILON);
    auto complete_symm_flag = false;

    auto solver1 = CvLevMarq(nparams, nerrs, criteria, complete_symm_flag);
    auto solver2 = CvLevMarq(nparams, nerrs, criteria, complete_symm_flag);

    Mat initial_param1(9, 1, CV_64FC1);
    double low = 0.0;
    double high = +1.0;
    randu(initial_param1, Scalar(low), Scalar(high));
    Mat err, jac;
    CvMat param1 = initial_param1;
    cvCopy(&param1, solver1.param);

    int test_count=0;
    while(1){
        int iter = 0;
        for (int count=0;count<100;count++) {
        
            const CvMat* _param1 = 0;
            CvMat* _jac1 = 0;
            CvMat* _err1 = 0;
            cout << "iter=" << iter << " state=" << solver1.state << " errNorm=" << solver1.errNorm << endl;
        
            bool proceed = solver1.update(_param1, _jac1, _err1);
            cvCopy(_param1, &param1);

            if (!proceed || !_err1) break;

            if (_jac1) {
                Mat p = Mat(param1.rows, param1.cols, CV_64FC1, param1.data.db);
                calc_jacobian(jac, p,tag_rel, tparams.K1, tparams.proj[1], tparams.wrld[0]);
                CvMat tmp = jac;
                cvCopy(&tmp, _jac1);
            }

            if (_err1) {
                Mat p = Mat(param1.rows, param1.cols, CV_64FC1, param1.data.db);
                calc_error(err, p, tag_rel, tparams.K1, tparams.proj[1], tparams.wrld[0], tparams.imgpt[0]);
                iter++;
                CvMat tmp = err;
                cvCopy(&tmp, _err1);
            }
        }

        iter = 0;
    
        Mat init_p2 = Mat(param1.rows, param1.cols, CV_64FC1, param1.data.db);
        double *P1 = init_p2.ptr<double>(0);
        Mat initial_param2 = (Mat_<double>(9,1) << (P1[4]*P1[8] - P1[5]*P1[7]), (P1[2]*P1[7] - P1[1]*P1[8]), (P1[1]*P1[5] - P1[2]*P1[4]),
                                       (P1[5]*P1[6] - P1[3]*P1[8]), (P1[0]*P1[8] - P1[2]*P1[6]), (P1[2]*P1[3] - P1[0]*P1[5]),
                                       (P1[3]*P1[7] - P1[4]*P1[6]), (P1[1]*P1[6] - P1[0]*P1[7]), (P1[0]*P1[4] - P1[1]*P1[3]));
        double det = P1[0]*(P1[4]*P1[8]-P1[5]*P1[7]) - P1[1]*(P1[3]*P1[8]-P1[5]*P1[6]) + P1[2]*(P1[3]*P1[7]-P1[4]*P1[6]);
        initial_param2 = initial_param2 / det;
        CvMat param2 = initial_param2;
        cvCopy(&param2, solver2.param);
        for (int count=0;count<100;count++) {
            const CvMat* _param2 = 0;
            CvMat* _jac2 = 0;
            CvMat* _err2 = 0;
            cout << "qiter=" << iter << " state=" << solver1.state << " errNorm=" << solver1.errNorm << endl;

            bool proceed = solver2.update(_param2, _jac2, _err2);
            cvCopy(_param2, &param2);


            if (!proceed || !_err2) break;

            if (_jac2) {
                Mat p = Mat(param2.rows, param2.cols, CV_64FC1, param2.data.db);
                calc_jacobian(jac, p,tag_rel, tparams.K2, tparams.proj[0], tparams.wrld[1]);
                CvMat tmp = jac;
                cvCopy(&tmp, _jac2);
            }

            if (_err2) {
                Mat p = Mat(param2.rows, param2.cols, CV_64FC1, param2.data.db);
                calc_error(err, p,tag_rel, tparams.K2, tparams.proj[0], tparams.wrld[1], tparams.imgpt[1]);
                iter++;
                CvMat tmp = err;
                cvCopy(&tmp, _err2);
            }
        }
         Mat init_p1 = Mat(param2.rows, param2.cols, CV_64FC1, param2.data.db);
        double *P2 = init_p1.ptr<double>(0);
        initial_param1 = (Mat_<double>(9,1) << (P2[4]*P2[8] - P2[5]*P2[7]), (P2[2]*P2[7] - P2[1]*P2[8]), (P2[1]*P2[5] - P2[2]*P2[4]),
                                               (P2[5]*P2[6] - P2[3]*P2[8]), (P2[0]*P2[8] - P2[2]*P2[6]), (P2[2]*P2[3] - P2[0]*P2[5]),
                                               (P2[3]*P2[7] - P2[4]*P2[6]), (P2[1]*P2[6] - P2[0]*P2[7]), (P2[0]*P2[4] - P2[1]*P2[3]));

        det = P2[0]*(P2[4]*P2[8]-P2[5]*P2[7]) - P2[1]*(P2[3]*P2[8]-P2[5]*P2[6]) + P2[2]*(P2[3]*P2[7]-P2[4]*P2[6]);

        initial_param1 = initial_param1 / det;

        param1 = initial_param1;
        calc_error_full(err, initial_param1, tag_rel, tparams.K1, tparams.K2, tparams.proj, tparams.wrld, tparams.imgpt);
        err= abs(err);
        Mat res;
        res=abs(err);
        reduce(res, res, 0, CV_REDUCE_AVG);
        cout<<"total_cost_1:"<<res<<endl;
        cvCopy(&param1, solver1.param);
        test_count++;
        if(abs(res.at<double>(0, 0)) < 50){
            break;
        }
    }

    cout << "estimated param: " << endl;
    cout << Mat(param1.rows, param1.cols, CV_64FC1, param1.data.db) << endl;
    Mat cam_rel = Mat(3,3, CV_64FC1, param1.data.db);
    FileStorage fs1("cam_relation.yaml",FileStorage::WRITE);
    fs1 << "cam_rel" << cam_rel;
    fs1.release();
    return 0;
}
