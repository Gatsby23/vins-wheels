/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames0,
                vector<string> &vstrImageFilenames1,
                vector<double> &vTimestamps);

Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/leftImage",1000);
	ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/rightImage",1000);

	if(argc != 3)
	{
		printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] \n"
			   "for example: rosrun vins kitti_odom_test "
			   "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
			   "/media/tony-ws1/disk_D/kitti/odometry/sequences/00/ \n");
		return 1;
	}

	string config_file = argv[1];
	printf("config_file: %s\n", argv[1]);
	string sequence = argv[2];
	printf("read sequence: %s\n", argv[2]);
	string dataPath = sequence + "/";

	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);

	// load image list
    const string strPathToSequence=argv[2];
    vector<string> vstrImageFilenames0;
    vector<string> vstrImageFilenames1;
    vector<double> vTimestamps;
    LoadImages(strPathToSequence,vstrImageFilenames0,vstrImageFilenames1,vTimestamps);

//	FILE* file;
//	file = std::fopen((dataPath + "times.txt").c_str() , "r");
//	if(file == NULL){
//	    printf("cannot find file: %stimes.txt\n", dataPath.c_str());
//	    ROS_BREAK();
//	    return 0;
//	}
//	double imageTime;
//	vector<double> imageTimeList;
//	while ( fscanf(file, "%lf", &imageTime) != EOF)
//	{
//	    imageTimeList.push_back(imageTime);
//	}
//	std::fclose(file);

	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	FILE* outFile;
    OUTPUT_FOLDER = strPathToSequence;
	outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
	if(outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

	for (size_t i = 0; i < vTimestamps.size(); i++)
	{	
		if(ros::ok())
		{
			printf("\nprocess image %d\n", (int)i);
			stringstream ss;
			ss << setfill('0') << setw(6) << i;
			leftImagePath = vstrImageFilenames0[i];
			rightImagePath = vstrImageFilenames1[i];
			//printf("%lu  %f \n", i, imageTimeList[i]);
			//printf("%s\n", leftImagePath.c_str() );
			//printf("%s\n", rightImagePath.c_str() );

			imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
            imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );
            if(imLeft.empty() || imRight.empty())
            {
                std::cout<<"!!!! file empty in place:"<<leftImagePath<<endl;
                continue;
            }
            cvtColor(imLeft,imLeft,CV_BayerBG2GRAY,0);

            sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
			imLeftMsg->header.stamp = ros::Time(vTimestamps[i]);
			pubLeftImage.publish(imLeftMsg);

            cvtColor(imRight,imRight,CV_BayerBG2GRAY,0);
//            continue;
            sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
			imRightMsg->header.stamp = ros::Time(vTimestamps[i]);
			pubRightImage.publish(imRightMsg);


			estimator.inputImage(vTimestamps[i], imLeft,imRight);//时间戳 左目 右目
			
			Eigen::Matrix<double, 4, 4> pose;
			estimator.getPoseInWorldFrame(pose);
			if(outFile != NULL)
				fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
																	       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
																	       pose(2,0), pose(2,1), pose(2,2),pose(2,3));
			
			//cv::imshow("leftImage", imLeft);
			//cv::imshow("rightImage", imRight);
			//cv::waitKey(2);
		}
		else
			break;
	}
	if(outFile != NULL)
		fclose (outFile);
	return 0;
}


void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames0,vector<string> &vstrImageFilenames1,
                vector<double> &vTimestamps)
{
    ifstream fTimes0,fTimes1;
    string strPathTimeFile0 = strPathToSequence + "/stereo_left/image_name.txt";
    string strPathTimeFile1 = strPathToSequence + "/stereo_right/image_name.txt";

    fTimes0.open(strPathTimeFile0.c_str());
    fTimes1.open(strPathTimeFile1.c_str());
    vector<string> imageNameList0;
    vector<string> imageNameList1;

    while(!fTimes0.eof() && !fTimes1.eof())
    {
        string s0,s1;
        getline(fTimes0,s0);
        getline(fTimes1,s1);

        if(!s0.empty() && !s1.empty())
        {
            stringstream ss;
            string timeStr=s0.substr(0,19);
            ss << timeStr;
            double t;
            ss >> t;
            vTimestamps.push_back(t/1e9);
            imageNameList0.push_back(s0);
//            imageNameList1.push_back(s1);// 会导致左右目帧号不匹配
            imageNameList1.push_back(s0);
        }
//        std::cout<<s0<<std::endl;
    }
    fTimes0.close();
    fTimes1.close();

    string strPrefixLeft = strPathToSequence;

    const int nTimes = vTimestamps.size();
    vstrImageFilenames0.resize(nTimes);
    vstrImageFilenames1.resize(nTimes);

#if 1
    for(int i=0; i<nTimes; i++)
    {
//        stringstream ss;
//        ss << setfill('0') << setw(6) << i;
//        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageFilenames0[i]=strPrefixLeft+"/stereo_left/"+imageNameList0[i];
        vstrImageFilenames1[i]=strPrefixLeft+"/stereo_right/"+imageNameList1[i];  //vstrImageFilenames0是slam帧
//        cv::Mat left=cv::imread(vstrImageFilenames0[i],CV_LOAD_IMAGE_ANYDEPTH);
//        cv::Mat left_rbg;
//        cvtColor(left,left_rbg,CV_BayerBG2RGB,0);
//        cv::imshow("left",left);
//        cv::imshow("left_rbg",left_rbg);
//        cv::waitKey(1);
    }
#endif

}