/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 * global opt from read file
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>

GlobalOptimization globalEstimator;//新建线程
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path *global_path;
double last_vio_t = -1;
std::queue<sensor_msgs::NavSatFix> gpsQueue;
std::mutex m_buf;
std::string writePlace="";
void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    rot = Eigen::AngleAxisd (M_PI,Eigen::Vector3d::UnitZ()).toRotationMatrix()
            *Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitX()).toRotationMatrix();
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    //printf("gps_callback! \n");
    m_buf.lock();
//    gpsQueue.push(GPS_msg);
    m_buf.unlock();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);


    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFix GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg.header.stamp.toSec();
        printf("vio t: %f, gps t: %f  dif t: %f\n", t, gps_t,gps_t-t);
        // 10ms sync tolerance
        double syncTime=0.05;
        if(gps_t >= t - syncTime && gps_t <= t + syncTime)
        {
//            printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg.latitude;
            double longitude = GPS_msg.longitude;
            double altitude = GPS_msg.altitude;
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg.position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
                globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            break;
        }
        else if(gps_t < t - syncTime)
            gpsQueue.pop();
        else if(gps_t > t + syncTime)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);


    // write result to file
    std::ofstream foutC(writePlace.c_str(), ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << global_t.x() << ","
            << global_t.y() << ","
            << global_t.z() << ","
            << global_q.w() << ","
            << global_q.x() << ","
            << global_q.y() << ","
            << global_q.z() << endl;
    foutC.close();
}

void inputGPS(const sensor_msgs::NavSatFix &GPS_msg);
void inputSlam(const std::vector<double> &odom);
void load_vo(std::string path_,std::vector<std::vector<double>> &odomList);
int readVrsGps(ifstream &File,double &time_ , sensor_msgs::NavSatFix &gps_msg,double slam_time_=0.0);
void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
void pub_odom(double time_ ,Eigen::Quaterniond q,Eigen::Vector3d p);

ros::Publisher slam_path_pub;
nav_msgs::Path slamPath;
ros::Publisher imu_odom_pub;
bool initGPS=false;
bool initFirstGPS_flag=false;
GeographicLib::LocalCartesian geoConverter_main;
//在globalOpt 中构造函数开启了新的线程   threadOpt = std::thread(&GlobalOptimization::optimize, this);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");
    slam_path_pub = n.advertise<nav_msgs::Path>("slamPath",10);
    ros::Publisher gps_publisher=n.advertise<sensor_msgs::NavSatFix>("/gps/data_raw", 100, true);
    imu_odom_pub=n.advertise<nav_msgs::Odometry>("/imu_odom",10);//imu 位姿

    ros::Subscriber sub_GPS = n.subscribe("/gps/data_raw", 100, GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);

    global_path = &globalEstimator.global_path;


    string strPathSlam , strPathVrsGps;
    if(argc>=3)
    {
        strPathSlam = argv[1];
        strPathVrsGps=argv[2];
        writePlace="global_tum.txt";
    }
    else if(argc==2)
    {
        strPathSlam = argv[1];
        strPathSlam=strPathSlam+"/vio_tum.txt";
        strPathVrsGps=argv[1];
        strPathVrsGps=strPathVrsGps+"/vrs_gps.csv";
        writePlace=argv[1];
        writePlace = writePlace+"/global_tum.txt";
    }

    ofstream stateSave;
    stateSave.open(writePlace.c_str());
    stateSave<<fixed;

    std::cout<<"strPathVrsGps: "<<strPathVrsGps<<"\nstrPathSlam: "<<strPathSlam<<std::endl;
    std::vector<std::vector<double>> slamOdomList;
    load_vo(strPathSlam,slamOdomList);

    ifstream vrsGps_file_st(strPathVrsGps.c_str());
    double gps_time=0,slam_time=0;
    for(int i=0;i<slamOdomList.size();i++)
    {
        slam_time=slamOdomList[i][0];
        inputSlam(slamOdomList[i]);
        while(gps_time<slam_time+1)
        {
            sensor_msgs::NavSatFix gps_msg;
            int state=readVrsGps(vrsGps_file_st,gps_time,gps_msg,slam_time);
            if(state==4)
            {
                gps_publisher.publish(gps_msg);
                ros::Duration(0.1).sleep();//为了防止发布不成功
//                sensor_msgs::NavSatFixConstPtr cspt(&gps_msg);//=std::make_shared<sensor_msgs::NavSatFixConstPtr>(gps_msg);
                inputGPS(gps_msg);
            }
            else if(state==0)
                break;
        }
//        ros::Duration(0.001).sleep();//为了防止发布不成功
    }
    map<double, vector<double>> globalPoseMap;
    globalEstimator.getGlobalPoseMap(globalPoseMap);
    // write result to file
    std::ofstream foutC(writePlace.c_str(), ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    map<double, vector<double>>::iterator iter;
    iter = globalPoseMap.begin();
    for(int i=0;i<globalPoseMap.size();i++, iter++)
    {
        foutC.precision(6);
        foutC << iter->first << " ";
        foutC.precision(5);
        foutC << iter->second[0] << " "
              << iter->second[1] << " "
              << iter->second[2] << " "
              << iter->second[4] << " "
              << iter->second[5] << " "
              << iter->second[6] << " "
              << iter->second[3] << endl;
    }
    foutC.close();
    std::cout<<"endl read"<<std::endl;
    return 0;
    ros::spin();
    return 0;
}

int readVrsGps(ifstream &File,double &time_ , sensor_msgs::NavSatFix &gps_msg,double slam_time_)
{
    string FileGetline;
    if(!File.eof())
        std::getline(File,FileGetline);
    else{
        std::cout<<"END OF readVrsGps FILE "<<std::endl;
        return 0;
    }
    if(File.eof()){
        std::cout<<"END OF readVrsGps FILE  publish odometer"<<std::endl;
        return 0;
    }

    stringstream gps_ss(FileGetline);
    vector<string>line_data_vec;
    string value_str;
    while (getline(gps_ss, value_str, ','))
    {
        line_data_vec.push_back(value_str);
    }
    int state = stod(line_data_vec[6]);//状态：1正常，2DGPS，4固定这个精度最高，5浮动
    if(state!=4)
        return state;
    time_ = std::stod(line_data_vec[0])/1e9;
    ros::Time stamp(time_);
    gps_msg.header.stamp = ros::Time(time_);
    gps_msg.header.frame_id = "gps_frame";
    gps_msg.status.status = int(state);// sensor_msgs::NavSatStatus::STATUS_FIX; std::stoi(line_data_vec[1]);
    gps_msg.status.service = stoi(line_data_vec[7]);//sensor_msgs::NavSatStatus::SERVICE_GPS;
    gps_msg.latitude = std::stod(line_data_vec[1]);
    gps_msg.longitude = std::stod(line_data_vec[2]);
    gps_msg.altitude = std::stod(line_data_vec[5]);
    for (int i = 0; i < 9; i++)
    {
        gps_msg.position_covariance[i] = 0;//std::stod(line_data_vec[i + 4]) / 50;
    }
//    double syncTime=0.05;
//    std::cout<<"dif t: "<<time_-slam_time_<<std::endl;
    if(initGPS)
    {
        double xyz[3];
        GPS2XYZ(std::stod(line_data_vec[1]),std::stod(line_data_vec[2]), std::stod(line_data_vec[5]), xyz);
        Eigen::Quaterniond q(Eigen::Quaterniond::Identity());
        Eigen::Vector3d p(xyz[0],xyz[1],xyz[2]);
        pub_odom(time_,q,p);
//        ros::Duration(0.1).sleep();//为了防止发布不成功
    }


    return state;
//    gps_publisher.publish(gps_msg);
}

void load_vo(std::string path_,std::vector<std::vector<double>> &odomList)
{
    ifstream fileCamOdom(path_.c_str());
    std::cout<<"load vo path"<<std::endl;
//    while(!fileCamOdom.eof())
    while(fileCamOdom.peek()!=EOF)
    {
        std::vector<double> odom_(8,0.0);
        std::string fileLine;
        getline(fileCamOdom,fileLine);
//        if(fileCamOdom.eof())break;
        std::stringstream ss;
        ss<<fileLine;
        for(int i=0;i<odom_.size();i++) {
            ss >> odom_[i];
        }
        odomList.push_back(odom_);

        geometry_msgs::PoseStamped pos_stamped;
        pos_stamped.header.frame_id="world";
        pos_stamped.header.stamp=ros::Time(odom_[0]);
        pos_stamped.pose.position.x=odom_[1];
        pos_stamped.pose.position.y=odom_[2];
        pos_stamped.pose.position.z=odom_[3];
        pos_stamped.pose.orientation.x = odom_[4];
        pos_stamped.pose.orientation.y = odom_[5];
        pos_stamped.pose.orientation.z = odom_[6];
        pos_stamped.pose.orientation.w = odom_[7];
        slamPath.header.stamp=ros::Time(odom_[0]);
        slamPath.header.frame_id = "world";
        slamPath.poses.push_back(pos_stamped);
        if(0)
        {
            slam_path_pub.publish(slamPath);
            ros::Duration(0.01).sleep();
        }
//        std::cout<<"odom_[0]"<<odom_[0]<<std::endl;
    }
    std::cout<<"load vo path successful"<<std::endl;
    ros::Duration(1).sleep();//为了防止发布不成功
    slam_path_pub.publish(slamPath);
    std::cout<<"pub vo path "<<std::endl;
    ros::Duration(1).sleep();
}
void inputSlam(const std::vector<double> &odom)
{
    //printf("vio_callback! \n");
    double t = odom[0];
    last_vio_t = t;
    Eigen::Vector3d vio_t(odom[1], odom[2], odom[3]);
    Eigen::Quaterniond vio_q;
    vio_q.w() = odom[7];
    vio_q.x() = odom[4];
    vio_q.y() = odom[5];
    vio_q.z() = odom[6];
    globalEstimator.inputOdom(t, vio_t, vio_q);


    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFix GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg.header.stamp.toSec();
//        printf("vio t: %f, gps t: %f  dif t: %f\n", t, gps_t,gps_t-t);
        // 10ms sync tolerance
        double syncTime=0.05;
        if(gps_t >= t - syncTime && gps_t <= t + syncTime)
        {
//            printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg.latitude;
            double longitude = GPS_msg.longitude;
            double altitude = GPS_msg.altitude;
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg.position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
            globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();

            if(!initGPS)
            {
                double xyz[3];
                GPS2XYZ(latitude,longitude, altitude, xyz);
                Eigen::Quaterniond q(Eigen::Quaterniond::Identity());
                Eigen::Vector3d p(xyz[0],xyz[1],xyz[2]);
                pub_odom(t,q,p);
            }

            break;
        }
        else if(gps_t < t - syncTime)
            gpsQueue.pop();
        else if(gps_t > t + syncTime)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);

}

void inputGPS(const sensor_msgs::NavSatFix &GPS_msg)
{
    //printf("gps_callback! \n");
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();
}

void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter_main.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter_main.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}
void pub_odom(double time_ ,Eigen::Quaterniond q,Eigen::Vector3d p)
{
    // publish odom
    geometry_msgs::Twist last_vel_;
    last_vel_.linear.x=p.x();
    last_vel_.linear.y=p.y();
    last_vel_.linear.z=p.z();
    last_vel_.angular.x=0;
    last_vel_.angular.y=0;
    last_vel_.angular.z=0;
    nav_msgs::Odometry current_imu_odom_msgs_;
    current_imu_odom_msgs_.header.stamp = ros::Time(time_);
    current_imu_odom_msgs_.header.frame_id = "world";
    current_imu_odom_msgs_.child_frame_id = "imu_odom";
    current_imu_odom_msgs_.pose.pose.position.x = p.x();
    current_imu_odom_msgs_.pose.pose.position.y = p.y();
    current_imu_odom_msgs_.pose.pose.position.z = p.z();
    current_imu_odom_msgs_.pose.pose.orientation.x = q.x();
    current_imu_odom_msgs_.pose.pose.orientation.y = q.y();
    current_imu_odom_msgs_.pose.pose.orientation.z = q.z();
    current_imu_odom_msgs_.pose.pose.orientation.w = q.w();
    current_imu_odom_msgs_.pose.covariance[0]  = 1e-3;  // x cov
    current_imu_odom_msgs_.pose.covariance[7]  = 1e-3;  // y cov
    current_imu_odom_msgs_.pose.covariance[14] = 1e6;  // z cov
    current_imu_odom_msgs_.pose.covariance[21] = 1e6;  // roll cov
    current_imu_odom_msgs_.pose.covariance[28] = 1e6;  // pitch cov
    current_imu_odom_msgs_.pose.covariance[35] = 1e-3;   //yaw cov

    current_imu_odom_msgs_.twist.twist = last_vel_;
    current_imu_odom_msgs_.twist.covariance[0]  = 1e-2;  // x cov
    current_imu_odom_msgs_.twist.covariance[7]  = 1e10;  // y cov
    current_imu_odom_msgs_.twist.covariance[14] = 1e10;  // z cov
    current_imu_odom_msgs_.twist.covariance[21] = 1e10;  // roll cov
    current_imu_odom_msgs_.twist.covariance[28] = 1e10;  // pitch cov
    current_imu_odom_msgs_.twist.covariance[35] = 5e-2;   //yaw cov
        imu_odom_pub.publish(current_imu_odom_msgs_);
        ros::Duration(0.001).sleep();//为了防止发布不成功

}
