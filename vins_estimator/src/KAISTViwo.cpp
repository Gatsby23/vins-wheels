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
bool readImuFile(ifstream &imufile,double &time,
                 Eigen::Vector3d &mag,Eigen::Vector3d &acc,Eigen::Vector3d &gyr,
                 std::vector<std::vector<double>> &odom_ ,sensor_msgs::Imu &imuMsg);
bool readWheels(ifstream &wheelsFile,double &time_,
                Eigen::Vector2d &wheels,
                double &vel_,double &ang_vel_);//文件流 时间 轮编码计数 轮速 角速度
Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;
std::string IMAGE0_TOPIC,IMAGE1_TOPIC,IMU_TOPIC;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc <= 3)
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

    ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>(IMAGE0_TOPIC,1000);
    ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>(IMAGE1_TOPIC,1000);
    ros::Publisher pubImu = n.advertise<sensor_msgs::Imu>(IMU_TOPIC,1000);

    // load image list
    const string strPathToSequence=argv[2];
    vector<string> vstrImageFilenames0;
    vector<string> vstrImageFilenames1;
    vector<double> vTimestamps;
    LoadImages(strPathToSequence,vstrImageFilenames0,vstrImageFilenames1,vTimestamps);
    const string strPathImu=argv[3];//imu 文件地址
    ifstream imu_file_st(strPathImu.c_str());
    const string strPathWheels=argv[4];  //轮速计地址
    ifstream wheels_file_st(strPathWheels.c_str());

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
    ofstream path_Save;
    path_Save.open((OUTPUT_FOLDER + "/vio_tum.txt").c_str() );
    path_Save<<fixed;
//    OUTPUT_FOLDER = strPathToSequence;
    outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
    if(outFile == NULL)
        printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

    double imu_time=0;
    double wheels_time=0;
    Eigen::Vector2d wheels_cnt;
    for (size_t i = 0; i < vTimestamps.size(); i++)
    {
        if(ros::ok())
        {
            Eigen::Vector3d mag,acc,gyr;
            std::vector<std::vector<double>>odom;
            sensor_msgs::Imu imuMsg;
            while(imu_time<vTimestamps[i]) {
                if(readImuFile(imu_file_st, imu_time, mag, acc, gyr, odom, imuMsg)) {
//                    cout<<"imu_time="<<setprecision(17)<<imu_time<<"\tgyr="<<gyr.transpose()<<"\tacc="<<acc.transpose()<<"\tnorm="<<acc.norm()<<endl;
                    pubImu.publish(imuMsg);
                    estimator.inputIMU(imu_time, acc, gyr);
                }
                else break;
            }
            double vel,ang_vel;
            if(wheels_time==0)
                readWheels(wheels_file_st,wheels_time,wheels_cnt,vel,ang_vel);//先读一遍
            while(wheels_time<imu_time){
                {
                    double last_wheels_time=wheels_time;
                    if(readWheels(wheels_file_st,wheels_time,wheels_cnt,vel,ang_vel))//文件流 时间 轮编码计数 轮速 角速度
                    {
                        cout<<"vel_time="<<setprecision(17)<<wheels_time<<"\tlinearVel="<<vel<<"\todomAngleVel= "<<ang_vel<<endl;
                        estimator.inputVEL(wheels_time, vel, ang_vel);
                    }
                    else break;
                }
            }
            printf("\nprocess image %d with time:%f\n", (int)i,vTimestamps[i]);
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


            estimator.inputImage(vTimestamps[i], imLeft);//时间戳 左目 右目

            Eigen::Vector3d vel_est;
            estimator.getVelInWorldFrame(vel_est);
            std::cout<<"vel_estimator = "<<vel_est.transpose() <<"  norm= "<<vel_est.norm()<<std::endl;
            Eigen::Matrix<double, 4, 4> pose;
            estimator.getPoseInWorldFrame(pose);
            Eigen::Quaterniond q_(pose.matrix().block<3,3>(0,0));
            path_Save<<setprecision(6)<<vTimestamps[i]<<" "<<
                     setprecision(7)<<pose(0,3)<<" "<<pose(1,3)<<" "<<pose(2,3)<<" "<<
                     q_.x()<<" "<<q_.y()<<" "<<q_.z()<<" "<<q_.w()<<endl;
            if(outFile != NULL)
                fprintf (outFile, "%f %f %f %f %f %f %f %f \n",
                         vTimestamps[i],
                         pose(0,3), pose(1,3), pose(2,3),
                         q_.x(),q_.y(),q_.z(),q_.w());
//				fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
//																	       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
//																	       pose(2,0), pose(2,1), pose(2,2),pose(2,3));

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
//读取imu数据
bool readImuFile(ifstream &imufile,double &time,Eigen::Vector3d &mag,Eigen::Vector3d &acc,Eigen::Vector3d &gyr,
                 std::vector<std::vector<double>> &odom_ ,sensor_msgs::Imu &imuMsg)
{
    string lineStr_imu;
    std::getline(imufile, lineStr_imu);
    if (imufile.eof()) {
        std::cout << "END OF IMU FILE " << std::endl;
        return 0;
    }
//    cout <<"  getline="<<lineStr_imu << endl;
    stringstream ss(lineStr_imu);
// 按照逗号分隔
    vector<string> lineArray;
    string str;
    while (getline(ss, str, ','))
        lineArray.push_back(str);
    double time_now = stod(lineArray[0]) / 1e9;
//    if(time_now<=time_begin)return 0;//小于开始的时间戳就不要了
    if (time_now == time)return 0;
    time = time_now;
    Eigen::Quaterniond q_;
    q_.x() = stod(lineArray[1]);
    q_.y() = stod(lineArray[2]);
    q_.z() = stod(lineArray[3]);
    q_.w() = stod(lineArray[4]);
    Eigen::Vector3d euler_;
    euler_.x() = stod(lineArray[5]);
    euler_.y() = stod(lineArray[6]);
    euler_.z() = stod(lineArray[7]);

    gyr.x() = stod(lineArray[8]);
//    gyr.x()=gyr.x()*3.1415926/180;
    gyr.y() = stod(lineArray[9]);
//    gyr.y()=gyr.y()*3.1415926/180;
    gyr.z() = stod(lineArray[10]);
//    gyr.z()=gyr.z()*3.1415926/180;
    acc.x() = stod(lineArray[11]);
    acc.y() = stod(lineArray[12]);
    acc.z() = stod(lineArray[13]);

    mag.x() = stod(lineArray[14]);
    mag.y() = stod(lineArray[15]);
    mag.z() = stod(lineArray[16]);

//把imu数据压入队列
//    Eigen::Quaterniond q_imu(1,0,0,0);
//    imu_odom.add_imuPose(imu_time,acc,gyr);

//    gyr_buf.push_back(make_pair(time,gyr));
//    acc_buf.push_back(make_pair(time,acc));
//    imu_integral.process_imu(acc_buf,gyr_buf);
//    imu_odom.add_imuPose(time,imu_integral.last_q , imu_integral.R_init,R_imu_wheels);
//    imu_odom.add_imuPose(time,q_ , imu_integral.R_init,R_imu_wheels);

//    Eigen::Quaterniond q_imu=imu_integral.last_q;
//    vector<double>odom_stampte(8,0);
//    odom_stampte.reserve(8);
//    odom_stampte[0]=time;
//    odom_stampte[1]=imu_integral.last_p.x();
//    odom_stampte[2]=imu_integral.last_p.y();
//    odom_stampte[3]=imu_integral.last_p.z();
//    odom_stampte[4]=imu_integral.last_q.x();
//    odom_stampte[5]=imu_integral.last_q.y();
//    odom_stampte[6]=imu_integral.last_q.z();
//    odom_stampte[7]=imu_integral.last_q.w();
//    odom_.push_back(odom_stampte);

//    pub_odom(time ,imu_integral.last_q,imu_integral.last_p);

    imuMsg.header.frame_id = "world";
    imuMsg.header.stamp = ros::Time(time);
    imuMsg.linear_acceleration.x = acc.x();
    imuMsg.linear_acceleration.y = acc.y();
    imuMsg.linear_acceleration.z = acc.z();
    imuMsg.angular_velocity.x = gyr.x();
    imuMsg.angular_velocity.y = gyr.y();
    imuMsg.angular_velocity.z = gyr.z();
    imuMsg.orientation.x = q_.x();
    imuMsg.orientation.y = q_.y();
    imuMsg.orientation.z = q_.z();
    imuMsg.orientation.w = q_.w();

//    strArray.push_back(lineArray);
//    int64 num_str=stol(strArray[i][0]);//string 转数字
//    strnum.push_back(num_str);
//    cout<<"imu_time====="<<setprecision(17)<<imu_time<<endl;
    return 1;
}
//读取里程计数据
bool readWheels(ifstream &wheelsFile,double &time_,Eigen::Vector2d &wheels,
                double &vel_,double &ang_vel_)
{
    const double coeff_vel=1;
    const double coeff_steer_k0=1;
    const double coeff_steer_k1=0;//0.0012;//0.003822;//-0.003822;
    //数据集时间单位为微秒
    string wheelsFileGetline;
    if(!wheelsFile.eof())
        std::getline(wheelsFile,wheelsFileGetline);
    else{
        std::cout<<"END OF WHEELS FILE "<<std::endl;
        return false;
    }
    if(wheelsFile.eof()){
        std::cout<<"END OF WHEELS FILE  publish odometer"<<std::endl;
        return false;
    }
    stringstream ssLine(wheelsFileGetline);
    // 按照逗号分隔
    vector<string> lineArray;
    string str;
    while (getline(ssLine, str, ','))
        lineArray.push_back(str);

    double time_now = stod(lineArray[0])/1e9;
    time_now=time_now-0.0;//时间差矫正
    double dt = time_now-time_;
    //轮速
    Eigen::Vector2d wheels_now;
    wheels_now.x() = stod(lineArray[1])*0.623022*M_PI/4096.0;//左轮速
    wheels_now.y() = stod(lineArray[2])*0.622356*M_PI/4096.0;//右轮速

    Eigen::Vector2d delta_wheels;
    delta_wheels=wheels_now-wheels;
    if(time_==0){
        time_=time_now;
        wheels=wheels_now;
        return false;
    }
    time_=time_now;
    wheels=wheels_now;
    vel_ = ( delta_wheels.x() + delta_wheels.y() )/2.0/dt;
//    imu_odom.add_odom_linear(time_,vel_,steer_);

    //双轮差速模型
    double linearVel=vel_;
    double BASE_length=1.5285;//轮间距
    double odomAngleVel=(delta_wheels.x() - delta_wheels.y())/BASE_length/dt;
    ang_vel_ = odomAngleVel;

    double delta_dis = linearVel * dt;//位移变化量
    double delta_th = odomAngleVel*dt;//角度变化量

    double delta_x = cos(delta_th) * delta_dis;
    double delta_y = -sin(delta_th) * delta_dis;
//    Eigen::Matrix3d A=Eigen::Matrix3d::Identity();
//    if(delta_th>0.00)
//    {
//        A(0,0)= sin(delta_th)/delta_th;
//        A(1,1)=sin(delta_th)/delta_th;
//        A(0,1)=-(1- cos(delta_th))/delta_th;
//        A(1,0)=(1- cos(delta_th))/delta_th;
//    }
//    Eigen::Matrix3d R_th =Eigen::AngleAxisd(accumulate_th_,Eigen::Vector3d::UnitZ()).toRotationMatrix();
//    Eigen::Vector3d delta_p=R_th*A*Eigen::Vector3d(0,delta_dis,0);
//    accumulate_y_ = accumulate_y_+delta_p(1);
//    accumulate_x_ = accumulate_x_+delta_p(0);

//    accumulate_y_ = accumulate_y_ + cos(accumulate_th_)*delta_dis;//绕Z轴旋转变化到世界坐标系
//    accumulate_x_ = accumulate_x_ - sin(accumulate_th_)*delta_dis;

//    accumulate_x_ = accumulate_x_ + cos(accumulate_th_)*delta_x - sin(accumulate_th_)*delta_y;//绕Z轴旋转变化到世界坐标系
//    accumulate_y_ = accumulate_y_ + sin(accumulate_th_)*delta_x + cos(accumulate_th_)*delta_y;
//    accumulate_th_ += delta_th;

//    tf::Quaternion q = tf::createQuaternionFromYaw(accumulate_th_);
//    vector<double>odom_stampte(8,0);
//    odom_stampte.reserve(8);
//    odom_stampte.reserve(8);
//    odom_stampte[0]=time_;
//    odom_stampte[1]=accumulate_x_;
//    odom_stampte[2]=accumulate_y_;
//    odom_stampte[3]=0.0f;
//    odom_stampte[4]=q.x();
//    odom_stampte[5]=q.y();
//    odom_stampte[6]=q.z();
//    odom_stampte[7]=q.w();
//    odom_.push_back(odom_stampte);
//
//    // publish odom
//    geometry_msgs::Twist last_vel_;
//    last_vel_.linear.x=linearVel;
//    last_vel_.linear.y=0;
//    last_vel_.linear.z=0;
//    last_vel_.angular.x=0;
//    last_vel_.angular.y=0;
//    last_vel_.angular.z=odomAngleVel;
//    nav_msgs::Odometry current_odom_msgs_;
//    current_odom_msgs_.header.stamp = ros::Time(time_);
//    current_odom_msgs_.header.frame_id = "world";
//    current_odom_msgs_.child_frame_id = "odom";
//    current_odom_msgs_.pose.pose.position.x = accumulate_x_;
//    current_odom_msgs_.pose.pose.position.y = accumulate_y_;
//    current_odom_msgs_.pose.pose.position.z = 0.0;
//    current_odom_msgs_.pose.pose.orientation.x = q.x();
//    current_odom_msgs_.pose.pose.orientation.y = q.y();
//    current_odom_msgs_.pose.pose.orientation.z = q.z();
//    current_odom_msgs_.pose.pose.orientation.w = q.w();
//    current_odom_msgs_.pose.covariance[0]  = 1e-3;  // x cov
//    current_odom_msgs_.pose.covariance[7]  = 1e-3;  // y cov
//    current_odom_msgs_.pose.covariance[14] = 1e6;  // z cov
//    current_odom_msgs_.pose.covariance[21] = 1e6;  // roll cov
//    current_odom_msgs_.pose.covariance[28] = 1e6;  // pitch cov
//    current_odom_msgs_.pose.covariance[35] = 1e-3;   //yaw cov
//
//    current_odom_msgs_.twist.twist = last_vel_;
//    current_odom_msgs_.twist.covariance[0]  = 1e-2;  // x cov
//    current_odom_msgs_.twist.covariance[7]  = 1e10;  // y cov
//    current_odom_msgs_.twist.covariance[14] = 1e10;  // z cov
//    current_odom_msgs_.twist.covariance[21] = 1e10;  // roll cov
//    current_odom_msgs_.twist.covariance[28] = 1e10;  // pitch cov
//    current_odom_msgs_.twist.covariance[35] = 5e-2;   //yaw cov
//    if(pub_or_not)
//        odom_pub.publish(current_odom_msgs_);
//
////    if(time_/1e6  - pos_stamped.header.stamp)
//    geometry_msgs::PoseStamped pos_stamped;
//    pos_stamped.header.frame_id="world";
//    pos_stamped.header.stamp=ros::Time(wheel_time);
//    pos_stamped.pose=current_odom_msgs_.pose.pose;
//    wheelsPath.header.stamp=ros::Time(wheel_time);
//    wheelsPath.header.frame_id = "world";
//    wheelsPath.poses.push_back(pos_stamped);
//    if(pub_or_not)
//        wheels_path_pub.publish(wheelsPath);

//    cout<<"time="<<time<<"  wheels="<<wheels.transpose()<<endl;
    return true;
}
