/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 * 在执行标定时，需要把imu的代价函数等切换成我自己写的代码
 * 使用：IMUFactor* imu_factor = new IMUFactor(pre_integrations[j], show);
 *      midPointIntegration_wheel()
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames0,vector<string> &vstrImageFilenames1,
                vector<double> &vTimestamps0,vector<double> &vTimestamps1);
int readImuFile(ifstream &imufile,double &time,
                 Eigen::Vector3d &mag,Eigen::Vector3d &acc,Eigen::Vector3d &gyr,
                 std::vector<std::vector<double>> &odom_ ,sensor_msgs::Imu &imuMsg);//返回1 正常  返回2 数据无效 返回3 数据完成
bool readWheels(ifstream &wheelsFile,double &time_,double &time_last_,Eigen::Vector2d &wheels,
                double &vel_,double &ang_vel_);//文件流 时间 轮编码计数 轮速 角速度
int readGps(ifstream &File,double &time_ , sensor_msgs::NavSatFix &gps_msg);

Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;
std::string IMAGE0_TOPIC,IMAGE1_TOPIC,IMU_TOPIC;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc < 3)
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
    ros::Publisher gps_publisher=n.advertise<sensor_msgs::NavSatFix>("/gps/data_raw", 100, true);
    string strPathToSequence,strPathImu,strPathWheels,strPathGps;
    if(argc==3)
    {
        string basePath = argv[2];
        strPathToSequence=basePath;//图像地址
        strPathImu=basePath+ "/imu.txt";//imu 文件地址
        strPathWheels=basePath+ "/can.txt";  //轮速计地址
        strPathGps=basePath+ "/gps.txt";  //GPS地址
    }
    else
    {
        strPathToSequence=argv[2];//图像地址
        strPathImu=argv[3];//imu 文件地址
        strPathWheels=argv[4];  //轮速计地址
        strPathGps=argv[5];  //GPS地址
    }
    // load image list
    vector<string> vstrImageFilenames0;
    vector<string> vstrImageFilenames1;
    vector<double> vTimestamps0,vTimestamps1;
    LoadImages(strPathToSequence,vstrImageFilenames0,vstrImageFilenames1,vTimestamps0,vTimestamps1);
    ifstream imu_file_st(strPathImu.c_str());
    ifstream wheels_file_st(strPathWheels.c_str());
    ifstream gps_file_st(strPathGps.c_str());

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

    ofstream stateSave;
    stateSave.open((OUTPUT_FOLDER + "/state.txt").c_str() );
    stateSave<<fixed;

    ofstream imu_path(OUTPUT_FOLDER+"imu_int_origin.csv");
//    imu_path.open((OUTPUT_FOLDER+"imu_int_origin.csv").c_str());
    imu_path<<"imu data"<<endl;

    ofstream imu_path2(OUTPUT_FOLDER+"imu_2.csv");
//    imu_path.open((OUTPUT_FOLDER+"imu_int_origin.csv").c_str());
    imu_path2<<"imu data"<<endl;

    ofstream exe_path2(OUTPUT_FOLDER+"exe.csv");
//    imu_path.open((OUTPUT_FOLDER+"imu_int_origin.csv").c_str());
    exe_path2<<"exe data"<<endl;
//    imu_path.close();
//    OUTPUT_FOLDER = strPathToSequence;
    outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
    if(outFile == NULL)
        printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

    double imu_time=0;
    double wheels_time=0,last_wheels_time=0;
    double gps_time=0;
    Eigen::Vector2d wheels_cnt;
    vector<double> vTimestamps;
    if(CAM_NUM==0)vTimestamps=vTimestamps0;//选择相机0还是相机1的时间戳
    else vTimestamps=vTimestamps1;
    for (size_t i = 0; i < vTimestamps.size(); i=i+3)//10916   图像时间戳间隔太低，会使IMU信息矩阵过大  0812序列起始为327
    {
        if(ros::ok())
        {
            Eigen::Vector3d mag,acc,gyr;
            std::vector<std::vector<double>>odom;
            sensor_msgs::Imu imuMsg;
            while(imu_time<vTimestamps[i]) {
                int flag = readImuFile(imu_file_st, imu_time, mag, acc, gyr, odom, imuMsg);
                if(flag==1) {
//                    cout<<"imu_time="<<setprecision(17)<<imu_time<<"\tgyr="<<gyr.transpose()<<"\tacc="<<acc.transpose()<<"\tnorm="<<acc.norm()<<endl;
                    pubImu.publish(imuMsg);
                    estimator.inputIMU(imu_time, acc, gyr);
                }
                else if(flag==2);
                else break;
            }
            double vel,ang_vel;
            if(last_wheels_time==0)
                readWheels(wheels_file_st,wheels_time,last_wheels_time,wheels_cnt,vel,ang_vel);//先读一遍
//            cout<<"vel_time="<<setprecision(17)<<"  linearVel=   ";
            while(wheels_time<imu_time)
            {
//                    double last_wheels_time=wheels_time;
                    if(readWheels(wheels_file_st,wheels_time,last_wheels_time,wheels_cnt,vel,ang_vel))//文件流 时间 轮编码计数 轮速 角速度
                    {
//                        cout<<"vel_time="<<setprecision(17)<<wheels_time<<"\tlinearVel="<<vel<<"\todomAngleVel= "<<ang_vel<<endl;
//                        cout<<setprecision(17)<<vel<<"  ";
                        Eigen::Vector3d velVec(0,vel,0);
                        estimator.inputVEL(wheels_time, velVec, ang_vel);
                    }
                    else break;
            }
//            cout<<endl;
            while(gps_time < imu_time)
            {
                sensor_msgs::NavSatFix gps_msg;
                int flag = readGps(gps_file_st,gps_time,gps_msg);
                if(flag==1)
                {
                    gps_publisher.publish(gps_msg);
                }
                else if(flag==2)
                {
                    ;
                }
                else break;
            }
            printf("\nprocess image %d with time:%f\n", (int)i,vTimestamps[i]);
            leftImagePath = vstrImageFilenames0[i];
            rightImagePath = vstrImageFilenames1[i];
            //printf("%lu  %f \n", i, imageTimeList[i]);
            //printf("%s\n", leftImagePath.c_str() );
            //printf("%s\n", rightImagePath.c_str() );

            imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
            imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );
            if(imLeft.empty() && CAM_NUM==0 || imRight.empty()&&CAM_NUM==1)
            {
                std::cout<<"!!!! file empty in place:"<<leftImagePath<<endl;
                continue;
            }
//            cvtColor(imLeft,imLeft,CV_BayerBG2GRAY,0);

            sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
            imLeftMsg->header.stamp = ros::Time(vTimestamps[i]);
            pubLeftImage.publish(imLeftMsg);

//            cvtColor(imRight,imRight,CV_BayerBG2GRAY,0);
//            continue;
            sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
            imRightMsg->header.stamp = ros::Time(vTimestamps[i]);
            pubRightImage.publish(imRightMsg);

//            continue;
            if(CAM_NUM==0)//选择使用相机0还是相机1 imLeft是相机0
                estimator.inputImage(vTimestamps[i], imLeft);//时间戳 左目 右目
            else
                estimator.inputImage(vTimestamps[i], imRight);//时间戳 左目 右目

            Eigen::Vector3d vel_est;
            estimator.getVelInWorldFrame(vel_est);
//            std::cout<<"vel_estimator = "<<vel_est.transpose() <<"  norm= "<<vel_est.norm()<<std::endl;
            Eigen::Matrix<double, 4, 4> pose;
            estimator.getPoseInWorldFrame(pose);
            Eigen::Quaterniond q_(pose.matrix().block<3,3>(0,0));
            if(estimator.solver_flag==Estimator::SolverFlag::NON_LINEAR)
                path_Save<<setprecision(6)<<vTimestamps[i]<<" "<<
                     setprecision(7)<<pose(0,3)<<" "<<pose(1,3)<<" "<<pose(2,3)<<" "<<
                     q_.x()<<" "<<q_.y()<<" "<<q_.z()<<" "<<q_.w()<<endl;
            stateSave.precision(6);
            stateSave << vTimestamps[i]<< ",";
            stateSave.precision(5);
            stateSave <<"Ps: "<<estimator.Ps[WINDOW_SIZE].x() << ","<< estimator.Ps[WINDOW_SIZE].y() << ","<< estimator.Ps[WINDOW_SIZE].z() << "\t"
                      <<"Vs: "<< estimator.Vs[WINDOW_SIZE].x() << ","<< estimator.Vs[WINDOW_SIZE].y() << ","<< estimator.Vs[WINDOW_SIZE].z() <<","<<estimator.Vs[WINDOW_SIZE].norm()<<"\t"
                      <<"vVel: "<<vel;
            if(estimator.solver_flag==Estimator::SolverFlag::NON_LINEAR && estimator.pre_integrations[estimator.frame_count-1]->sum_dt>0)
            {
                stateSave<<"\tangVel: "<<estimator.pre_integrations[estimator.frame_count-1]->delta_angleaxis.angle()*180.0f/M_PI/estimator.pre_integrations[estimator.frame_count-1]->sum_dt
                         <<"\tpara_SpeedBias:v: "<<estimator.para_SpeedBias[estimator.frame_count-1][0]<<","<<estimator.para_SpeedBias[estimator.frame_count-1][1]<<","<<estimator.para_SpeedBias[estimator.frame_count-1][2]
                         <<"\tba: "<<estimator.para_SpeedBias[estimator.frame_count-1][3]<<","<<estimator.para_SpeedBias[estimator.frame_count-1][4]<<","<<estimator.para_SpeedBias[estimator.frame_count-1][5]
                         <<"\tbg: "<<estimator.para_SpeedBias[estimator.frame_count-1][6]<<","<<estimator.para_SpeedBias[estimator.frame_count-1][7]<<","<<estimator.para_SpeedBias[estimator.frame_count-1][8]<<endl;
            }
            else
                stateSave<<endl;
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
    stateSave.close();
    return 0;
}


void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames0,vector<string> &vstrImageFilenames1,
                vector<double> &vTimestamps0,vector<double> &vTimestamps1)
{
    ifstream fTimes0,fTimes1;
    string strPathTimeFile0 = strPathToSequence + "/image_capturer_0/image_name.txt";//前视相机
    string strPathTimeFile1 = strPathToSequence + "/image_capturer_1/image_name.txt";//后相机

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
            stringstream ss0,ss1;
            string timeStr0=s0.substr(0,16);
            string timeStr1=s1.substr(0,16);
            ss0 << timeStr0;
            ss1 << timeStr1;
            double t0,t1;
            ss0 >> t0;
            ss1 >> t1;
            vTimestamps0.push_back(t0);
            vTimestamps1.push_back(t1);
            imageNameList0.push_back(s0);
            imageNameList1.push_back(s1);// 会导致左右目帧号不匹配
//            imageNameList1.push_back(s0);
        }
//        std::cout<<s0<<std::endl;
    }
    fTimes0.close();
    fTimes1.close();

    string strPrefixLeft = strPathToSequence;

    const int nTimes0 = vTimestamps0.size();
    const int nTimes1 = vTimestamps1.size();
    vstrImageFilenames0.resize(nTimes0);
    vstrImageFilenames1.resize(nTimes1);

#if 1
    for(int i=0; i<nTimes0; i++)
    {
//        stringstream ss;
//        ss << setfill('0') << setw(6) << i;
//        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageFilenames0[i]=strPrefixLeft+"/image_capturer_0/"+imageNameList0[i];
//        cv::Mat left=cv::imread(vstrImageFilenames0[i],CV_LOAD_IMAGE_ANYDEPTH);
//        cv::Mat left_rbg;
//        cvtColor(left,left_rbg,CV_BayerBG2RGB,0);
//        cv::imshow("left",left);
//        cv::imshow("left_rbg",left_rbg);
//        cv::waitKey(1);
    }
    for(int i=0; i<nTimes1; i++)
    {
        vstrImageFilenames1[i]=strPrefixLeft+"/image_capturer_1/"+imageNameList1[i];  //vstrImageFilenames0是slam帧
    }
#endif

}
//读取imu数据
int readImuFile(ifstream &imufile,double &time,Eigen::Vector3d &mag,Eigen::Vector3d &acc,Eigen::Vector3d &gyr,
                 std::vector<std::vector<double>> &odom_ ,sensor_msgs::Imu &imuMsg)
{
    string lineStr_imu;
    std::getline(imufile, lineStr_imu);
    if (imufile.eof()) {
        std::cout << "END OF IMU FILE " << std::endl;
        return 3;
    }
//    cout <<"  getline="<<lineStr_imu << endl;
    stringstream ss(lineStr_imu);
// 按照逗号分隔
    vector<string> lineArray;
    string str;
    while (ss>>str)
        lineArray.push_back(str);
    double time_now = stod(lineArray[1]);
//    if(time_now<=time_begin)return 0;//小于开始的时间戳就不要了
    if (time_now == time)
        return 2;
    time = time_now;
//    Eigen::Quaterniond q_;
//    q_.x() = stod(lineArray[1]);
//    q_.y() = stod(lineArray[2]);
//    q_.z() = stod(lineArray[3]);
//    q_.w() = stod(lineArray[4]);
//    Eigen::Vector3d euler_;
//    euler_.x() = stod(lineArray[5]);
//    euler_.y() = stod(lineArray[6]);
//    euler_.z() = stod(lineArray[7]);

    mag.x() = stod(lineArray[3]);
    mag.y() = stod(lineArray[4]);
    mag.z() = stod(lineArray[5]);

    gyr.x() = stod(lineArray[7]);
//    gyr.x()=gyr.x()*3.1415926/180;
    gyr.y() = stod(lineArray[8]);
//    gyr.y()=gyr.y()*3.1415926/180;
    gyr.z() = stod(lineArray[9]);
//    gyr.z()=gyr.z()*3.1415926/180;
    acc.x() = stod(lineArray[11]);
    acc.y() = stod(lineArray[12]);
    acc.z() = stod(lineArray[13]);


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
//    imuMsg.orientation.x = q_.x();
//    imuMsg.orientation.y = q_.y();
//    imuMsg.orientation.z = q_.z();
//    imuMsg.orientation.w = q_.w();

//    strArray.push_back(lineArray);
//    int64 num_str=stol(strArray[i][0]);//string 转数字
//    strnum.push_back(num_str);
//    cout<<"imu_time====="<<setprecision(17)<<imu_time<<endl;
    return 1;
}
//读取里程计数据
bool readWheels(ifstream &wheelsFile,double &time_,double &time_last_,Eigen::Vector2d &wheels,
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
    while (ssLine>>str)
        lineArray.push_back(str);

    double time_now = stod(lineArray[0]);
    time_now=time_now-0.0;//时间差矫正
//    double dt = time_now-time_;
    double dt = time_now-time_last_;
    //轮速
    Eigen::Vector2d wheels_now;
    wheels_now.x() = stod(lineArray[1]);//轮速
    wheels_now.y() = stod(lineArray[2])*3.1415926535/180*coeff_steer_k0+coeff_steer_k1;//方向盘角度
    wheels_now.x() = wheels_now.x()* cos(wheels_now.y());//速度 cos

    Eigen::Vector2d delta_wheels;
    delta_wheels=wheels_now-wheels;
    if(time_last_==0){
        time_last_=time_now;
        wheels=wheels_now;
        return false;
    }
    time_=0.5f*(time_now+time_last_);
//    std::cout<<"time_now="<<setprecision(17)<<time_now<<" mid="<<0.5*(time_now+time_last_)<<std::endl;
    time_last_=time_now;
    wheels=wheels_now;
    vel_ = wheels_now.x();
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
    return true;
}

int readGps(ifstream &File,double &time_ , sensor_msgs::NavSatFix &gps_msg)
{
    string FileGetline;
    if(!File.eof())
        std::getline(File,FileGetline);
    else{
        std::cout<<"END OF GPS FILE "<<std::endl;
        return 3;
    }
    if(File.eof()){
        std::cout<<"END OF GPS FILE  publish odometer"<<std::endl;
        return 3;
    }

    stringstream gps_ss(FileGetline);
    vector<string>line_data_vec;
    string value_str;
    while (gps_ss>>value_str)
    {
        line_data_vec.push_back(value_str);
    }
    double calibrate1=0,calibrate2=0,calibrate3=0;
    calibrate1=std::stod(line_data_vec[7]);
    calibrate2=std::stod(line_data_vec[8]);
    calibrate3=std::stod(line_data_vec[9]);
    time_ = std::stod(line_data_vec[0]);
    if(calibrate1!=4 || calibrate2!=1 || calibrate3!=1)
        for (int i = 0; i < 9; i++)
            gps_msg.position_covariance[i] = 0;
    else
        return 2;
    ros::Time stamp(time_);
    gps_msg.header.stamp = stamp;
    gps_msg.header.frame_id = "gps_frame";
    gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; std::stoi(line_data_vec[1]);
    gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    gps_msg.latitude = std::stod(line_data_vec[1]);
    gps_msg.longitude = std::stod(line_data_vec[2]);
    gps_msg.altitude = std::stod(line_data_vec[3]);

    return 1;
//    gps_publisher.publish(gps_msg);
}
