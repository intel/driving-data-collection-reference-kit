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
    Mat K;
    vector<Mat> proj[2];
    vector<vector<Point3d> >  wrld[2];
    vector<vector<Point2d> > imgpt[2];
}Tagparams;


void readtagdetails(const string &path,Tagparams& tparams){
    FileStorage fs(path,FileStorage::READ);
    fs["K"] >> tparams.K;
    fs["no_of_patterns"] >> tparams.pattern_count;
    FileNode temp_ID;
    for(int i=0;i<tparams.pattern_count;i++){
        vector<Point2d> ipt[2];
        vector<Point3d> wpt[2];
        Mat proj[2];

        temp_ID=fs["pose"+to_string(i)];
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

void calc_jacobian(Mat& jac, Mat& param, Mat& K,vector<Mat>& P, vector<vector<Point3d> >& wrld) {
    jac=Mat::zeros(wrld.size()*wrld[0].size()*2, 9, CV_64F);
    double *M =param.ptr<double>(0);
    int count =0;
    for (auto j = 0; j < wrld.size(); j++) {
        Mat proj = K * P[j];
        double *ptr_Proj = proj.ptr<double>(0);
        for(auto k =0; k< wrld[j].size();k++){
        	double numer_x = ptr_Proj[0] * (M[0]*wrld[j][k].x + M[1]*wrld[j][k].y + M[2]) + ptr_Proj[1] * (M[3]*wrld[j][k].x + M[4]*wrld[j][k].y + M[5]) + ptr_Proj[2] * (M[6]*wrld[j][k].x + M[7]*wrld[j][k].y + M[8]) ;
        	double numer_y = ptr_Proj[3] * (M[0]*wrld[j][k].x + M[1]*wrld[j][k].y + M[2]) + ptr_Proj[4] * (M[3]*wrld[j][k].x + M[4]*wrld[j][k].y + M[5]) + ptr_Proj[5] * (M[6]*wrld[j][k].x + M[7]*wrld[j][k].y + M[8]) ;
        	double denom   = ptr_Proj[6] * (M[0]*wrld[j][k].x + M[1]*wrld[j][k].y + M[2]) + ptr_Proj[7] * (M[3]*wrld[j][k].x + M[4]*wrld[j][k].y + M[5]) + ptr_Proj[8] * (M[6]*wrld[j][k].x + M[7]*wrld[j][k].y + M[8]) ;
        	for(auto q = 0; q <2; q++){
                double numer;
                if(q==0)
                    numer = numer_x;
                else
                    numer = numer_y;
                jac.at<double>(count  , 0) = (denom*ptr_Proj[3*q+0]*wrld[j][k].x - numer * ptr_Proj[6]*wrld[j][k].x) / (denom*denom);
                jac.at<double>(count  , 1) = (denom*ptr_Proj[3*q+0]*wrld[j][k].y - numer * ptr_Proj[6]*wrld[j][k].y) / (denom*denom);
                jac.at<double>(count  , 2) = (denom*ptr_Proj[3*q+0]              - numer * ptr_Proj[6]             ) / (denom*denom);
                jac.at<double>(count  , 3) = (denom*ptr_Proj[3*q+1]*wrld[j][k].x - numer * ptr_Proj[7]*wrld[j][k].x) / (denom*denom);
                jac.at<double>(count  , 4) = (denom*ptr_Proj[3*q+1]*wrld[j][k].y - numer * ptr_Proj[7]*wrld[j][k].y) / (denom*denom);
                jac.at<double>(count  , 5) = (denom*ptr_Proj[3*q+1]              - numer * ptr_Proj[7]             ) / (denom*denom);
                jac.at<double>(count  , 6) = (denom*ptr_Proj[3*q+2]*wrld[j][k].x - numer * ptr_Proj[8]*wrld[j][k].x) / (denom*denom);
                jac.at<double>(count  , 7) = (denom*ptr_Proj[3*q+2]*wrld[j][k].y - numer * ptr_Proj[8]*wrld[j][k].y) / (denom*denom);
                jac.at<double>(count  , 8) = (denom*ptr_Proj[3*q+2]              - numer * ptr_Proj[8]             ) / (denom*denom);
                count ++;
            }
        }
    }
}

void calc_error(Mat& err,  Mat& param, Mat& K,vector<Mat>& P, vector<vector<Point3d> >& wrld, vector<vector<Point2d> >& imgpt){
    err.create(wrld.size()*wrld[0].size()*2, 1, CV_64F);
    double *M =param.ptr<double>(0);
    int count =0;
    for (auto j = 0; j < wrld.size(); j++) {
        Mat proj = K * P[j];
        double *ptr_Proj = proj.ptr<double>(0);
        for(auto k =0; k< wrld[j].size();k++){
        	double numer_x = ptr_Proj[0] * (M[0]*wrld[j][k].x + M[1]*wrld[j][k].y + M[2]) + ptr_Proj[1] * (M[3]*wrld[j][k].x + M[4]*wrld[j][k].y + M[5]) + ptr_Proj[2] * (M[6]*wrld[j][k].x + M[7]*wrld[j][k].y + M[8]) ;
        	double numer_y = ptr_Proj[3] * (M[0]*wrld[j][k].x + M[1]*wrld[j][k].y + M[2]) + ptr_Proj[4] * (M[3]*wrld[j][k].x + M[4]*wrld[j][k].y + M[5]) + ptr_Proj[5] * (M[6]*wrld[j][k].x + M[7]*wrld[j][k].y + M[8]) ;
        	double denom   = ptr_Proj[6] * (M[0]*wrld[j][k].x + M[1]*wrld[j][k].y + M[2]) + ptr_Proj[7] * (M[3]*wrld[j][k].x + M[4]*wrld[j][k].y + M[5]) + ptr_Proj[8] * (M[6]*wrld[j][k].x + M[7]*wrld[j][k].y + M[8]) ;
        	double x_hat   = numer_x / denom ;
        	double y_hat   = numer_y / denom ;
             err.at<double>(count  , 0) = x_hat - imgpt[j][k].x ;
            err.at<double>(count+1, 0) = y_hat - imgpt[j][k].y ;
            count += 2;
        }
    }
}

void calc_error_full(Mat& err,  Mat& param, Mat& K,vector<Mat>* P, vector<vector<Point3d> >* wrld, vector<vector<Point2d> >* imgpt){
    err.create(wrld[0].size()*wrld[0][0].size()*4, 1, CV_64F);
    
    double *M = param.ptr<double>(0);
    for (auto j = 0; j < wrld[0].size(); j++) {
        Mat proj1 = K * P[0][j];
        Mat proj2 = K * P[1][j];
        double *ptr_Proj1 = proj1.ptr<double>(0);
        double *ptr_Proj2 = proj2.ptr<double>(0);

        for(auto k =0; k< wrld[0][j].size();k++){
            double numer_x_1 = ptr_Proj2[0] * (M[0]*wrld[0][j][k].x + M[1]*wrld[0][j][k].y + M[2]) + ptr_Proj2[1] * (M[3]*wrld[0][j][k].x + M[4]*wrld[0][j][k].y + M[5]) + ptr_Proj2[2] * (M[6]*wrld[0][j][k].x + M[7]*wrld[0][j][k].y + M[8]) ;
            double numer_y_1 = ptr_Proj2[3] * (M[0]*wrld[0][j][k].x + M[1]*wrld[0][j][k].y + M[2]) + ptr_Proj2[4] * (M[3]*wrld[0][j][k].x + M[4]*wrld[0][j][k].y + M[5]) + ptr_Proj2[5] * (M[6]*wrld[0][j][k].x + M[7]*wrld[0][j][k].y + M[8]) ;
            double denom_1   = ptr_Proj2[6] * (M[0]*wrld[0][j][k].x + M[1]*wrld[0][j][k].y + M[2]) + ptr_Proj2[7] * (M[3]*wrld[0][j][k].x + M[4]*wrld[0][j][k].y + M[5]) + ptr_Proj2[8] * (M[6]*wrld[0][j][k].x + M[7]*wrld[0][j][k].y + M[8]) ;
            double x_hat_1   = numer_x_1 / denom_1 ;
            double y_hat_1   = numer_y_1 / denom_1 ;

            double X = wrld[1][j][k].x;
            double Y = wrld[1][j][k].y;
            double d1 = (M[4]*M[8] - M[5]*M[7])*X + (M[2]*M[7] - M[1]*M[8])*Y + (M[1]*M[5] - M[2]*M[4]);
            double d2 = (M[5]*M[6] - M[3]*M[8])*X + (M[0]*M[8] - M[2]*M[6])*Y + (M[2]*M[3] - M[0]*M[5]);
            double d3 = (M[3]*M[7] - M[4]*M[6])*X + (M[1]*M[6] - M[0]*M[7])*Y + (M[0]*M[4] - M[1]*M[3]);
            double numer_x_2 = ptr_Proj2[0] * d1 + ptr_Proj2[1] * d2 + ptr_Proj2[2] * d3 ;
            double numer_y_2 = ptr_Proj2[3] * d1 + ptr_Proj2[4] * d2 + ptr_Proj2[5] * d3 ;
            double denom_2   = ptr_Proj2[6] * d1 + ptr_Proj2[7] * d2 + ptr_Proj2[8] * d3 ;
            double x_hat_2   = numer_x_2 / denom_2 ;
            double y_hat_2   = numer_y_2 / denom_2 ;

            err.at<double>(4*j*k+0, 0) =  x_hat_1 - imgpt[0][j][k].x ;
            err.at<double>(4*j*k+1, 0) = y_hat_1 - imgpt[0][j][k].y ;
            err.at<double>(4*j*k+2, 0) =  x_hat_2 - imgpt[1][j][k].x ;
            err.at<double>(4*j*k+3, 0) = y_hat_2 - imgpt[1][j][k].y ;
        }
    }
}

int main() {
    Tagparams tparams;
    readtagdetails("../tag_rel/data1.yaml", tparams);
    auto nparams = 9;
    auto nerrs = tparams.imgpt[0].size()*tparams.imgpt[0][0].size()*2;
    cout<<tparams.imgpt[0][0].size()<<endl;
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
            cout<<"uparam"<<cvarrToMat(solver1.param).t()<<endl;

            if (!proceed || !_err1) break;

            if (_jac1) {
                Mat p = Mat(param1.rows, param1.cols, CV_64FC1, param1.data.db);
                calc_jacobian(jac, p, tparams.K, tparams.proj[1], tparams.wrld[0]);
                CvMat tmp = jac;
                cvCopy(&tmp, _jac1);
            }

            if (_err1) {
                Mat p = Mat(param1.rows, param1.cols, CV_64FC1, param1.data.db);
                calc_error(err, p, tparams.K, tparams.proj[1], tparams.wrld[0], tparams.imgpt[0]);
                iter++;
                Mat res;
                res=abs(err);
                reduce(res, res, 0, CV_REDUCE_AVG);
                cout<<"total_cost:"<<res<<endl;
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
            cout<<"quparam"<<cvarrToMat(solver2.param).t()<<endl;


            if (!proceed || !_err2) break;

            if (_jac2) {
                Mat p = Mat(param2.rows, param2.cols, CV_64FC1, param2.data.db);
                calc_jacobian(jac, p, tparams.K, tparams.proj[0], tparams.wrld[1]);
                CvMat tmp = jac;
                cvCopy(&tmp, _jac2);
            }

            if (_err2) {
                Mat p = Mat(param2.rows, param2.cols, CV_64FC1, param2.data.db);
                calc_error(err, p, tparams.K, tparams.proj[0], tparams.wrld[1], tparams.imgpt[1]);
                Mat res;
                res=abs(err);
                reduce(res, res, 0, CV_REDUCE_AVG);
                cout<<"total_cost:"<<res<<endl;
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
        calc_error_full(err, initial_param1,  tparams.K, tparams.proj, tparams.wrld, tparams.imgpt);
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
    Mat tag_rel = Mat(3,3, CV_64FC1, param1.data.db);
    FileStorage fs("tag_relation.yaml",FileStorage::WRITE);
    fs << "tag_rel" << tag_rel;
    fs.release();


    return 0;
}
