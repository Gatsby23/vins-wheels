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

#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe_uisee.h"

#include "utility/tic_toc.h"
#include "pose_graph_uisee.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"

//#include "parameters/parameters.h"
#include "./featureTracker/feature_tracker.h"
#include <unistd.h>
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud;
std::vector<std::string> CAM_NAMES;

void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish_uisee();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    //ROS_INFO("image_callback!");
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}
// only for visualization

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

void vio_callback(const std::vector<std::vector<double>> &camPoseLis,int index)//uisee loop
{
    //ROS_INFO("vio_callback!");
    int loop_index_uisee=-1;
    double old_time=1596528944.9046099;
    double cur_time=1596529119.2249601;
    for(int i=0;i<camPoseLis.size()-1;i++) {
        std::vector<double>camPose=camPoseLis[i];
        index=i;
        Vector3d vio_t(camPose[1], camPose[2], camPose[3]);
        Quaterniond vio_q;
        vio_q.w() = camPose[7];
        vio_q.x() = camPose[4];
        vio_q.y() = camPose[5];
        vio_q.z() = camPose[6];
//    std::cout<<"before "<<vio_t.transpose()<<std::endl;
        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio * vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time::now(); //ros::Time(camPose[1]);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = vio_t.x();
        odometry.pose.pose.position.y = vio_t.y();
        odometry.pose.pose.position.z = vio_t.z();
        odometry.pose.pose.orientation.x = vio_q.x();
        odometry.pose.pose.orientation.y = vio_q.y();
        odometry.pose.pose.orientation.z = vio_q.z();
        odometry.pose.pose.orientation.w = vio_q.w();
        pub_odometry_rect.publish(odometry);

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;

        vio_t_cam = vio_t;//+ vio_q * tic;
        vio_q_cam = vio_q;//* qic;
        Eigen::Matrix3d vio_R = Eigen::Matrix3d(vio_q);

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
        if(camPoseLis[i][0]<old_time  &&  camPoseLis[i+1][0]>old_time)
        {
            std::cout<<"camPoseLis[i][0] "<<camPoseLis[i][0]<<" camPoseLis[i+1][0]"<<camPoseLis[i+1][0]<<std::endl;
            loop_index_uisee=i;
        }
        std::cout << "index=" << index;
        if (camPoseLis[i][0]<cur_time  &&  camPoseLis[i+1][0]>cur_time)//(index>300 && index<302)
        {
            KeyFrame *keyframe = new KeyFrame(camPose[0], frame_index, vio_t, vio_R,loop_index_uisee);   //应该是生成fast特征点了，用于DBW2中的query
            posegraph.addKeyFrame_uisee(keyframe, 1);//第二个参数代表是需要回环检测detect_loop 提取的FAST特征点
            posegraph.optimize6DoF_uisee();
        } else //if(index<302)
        {
            KeyFrame *keyframe = new KeyFrame(camPose[0], frame_index, vio_t, vio_R,-1);   //应该是生成fast特征点了，用于DBW2中的query
            posegraph.addKeyFrame_uisee(keyframe, 0);//第二个参数代表是需要回环检测detect_loop 提取的FAST特征点
        }
        usleep(1000);
    }
}

////订阅主节点发布的相机和IMU位姿变换 imu和相机之间的外参R，以imu为参考
void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
    //std::cout<<"callback tic="<<tic<<"  qic="<<qic<<std::endl;
}
/*
void process()
{
    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() 
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)//剔除最开始的SKIP_FIRST_CNT帧
            {
                skip_first_cnt++;
                continue;
            }
            //每隔SKIP_CNT帧进行一次  SKIP_CNT=0
            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            //将距上一关键帧距离（平移向量的模）超过SKIP_DIS的图像创建为关键帧
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];//去畸变后的归一化坐标
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];//特征点的图像坐标
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("p_2d_normal x %f, y %f \n", p_2d_normal.x, p_2d_normal.y);
                    //printf("p_2d_uv u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
//                    printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);

                }

                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   //应该是生成fast特征点了，用于DBW2中的query
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame_uisee(keyframe, 1);//第二个参数代表是需要回环检测detect_loop 提取的FAST特征点
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}
*/
void command()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("program shutting down...\n");
            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void find_cloast(std::vector<std::vector<double>> &imageOdomList)
{
    size_t Lsize=imageOdomList.size()-1;
    double distance=0.1;
    double time_dis=10;
    int max=0;
    for(int i=0;i<Lsize;i++){
        for (int j = i; j < Lsize; j++) {
//            std::cout<<"time_dis  "<<int(imageOdomList[j][0]-imageOdomList[i][0])<<std::endl;
            if(imageOdomList[j][0]-imageOdomList[i][0]<time_dis)
                continue;
            if(abs(imageOdomList[i][1]-imageOdomList[j][1])<distance
                && abs(imageOdomList[i][2]-imageOdomList[j][2])<distance
                && abs(imageOdomList[i][3]-imageOdomList[j][3])<distance){
                if(j-i>max)
                    max=j-i;
                std::cout<<"idx_dif="<<j-i<<"-------------------------------"<<std::endl;
                std::cout<<"finded old_kf  "<<"idx="<<i<<"  "<<std::setprecision(17)<<imageOdomList[i][0]<<" : ";
                for(int m=1;m<8;m++) {
                    std::cout<<std::setprecision(17)<<imageOdomList[i][m]<<"  ";
                }
                std::cout<<" "<<std::endl;
                std::cout<<"finded loop  "<<"idx="<<j<<std::setprecision(17)<<imageOdomList[j][0]<<" : ";
                for(int m=1;m<8;m++) {
                    std::cout<<imageOdomList[j][m]<<"  ";
                }
                std::cout<<" "<<std::endl;
            }
        }
    }
    std::cout<<"max="<<max<<std::endl;
}
void loade_odom()
{

#if 1
    //read uisee odom and camodom
#if 0
    std::string pathOdom="/media/qcx/D/work/20200804_car23_imu_can_camera_yahui/L1/odom_test.txt";
    std::string pathCamOdom="/media/qcx/D/work/20200804_car23_imu_can_camera_yahui/KeyFrameTrajectory_uisee_l1.txt";
#else
    std::string pathFather="/home/uisee/20200804_car23_imu_can_camera_yahui/biaoding";
    std::string pathOdom=pathFather+"/wheels_odom.txt";
    std::string pathCamOdom=pathFather+"/KeyFrameTrajectory_uisee.txt";
#endif
    std::ifstream fileOdom(pathOdom.c_str());
    std::ifstream fileCamOdom(pathCamOdom.c_str());
    Eigen::Matrix4d H_odom;//里程计
    H_odom=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d H_camOdom;//视觉
    H_camOdom=Eigen::Matrix4d::Identity();
    std::vector<std::vector<double>> odomList;
    std::vector<std::vector<double>> camOdomList;
    int idx=0;
    while(!fileOdom.eof())
    {
        std::vector<double> odom_(8,0.0);
        std::string fileLine;
        getline(fileOdom,fileLine);
        std::stringstream ss;
        ss<<fileLine;
        for(int i=0;i<odom_.size();i++) {
            ss >> odom_[i];
        }
        if(odomList.size()>=1)
        {
            if(odomList.back()[7]==odom_[7])
                continue;
        }
        odomList.push_back(odom_);
        if(odom_[0]>1596529072);
//            posegraph.addKeyFrame_uisee(odom_,true);
        std::chrono::milliseconds dura(5);
        idx++;
    }
    vio_callback(odomList,idx);
    std::cout<<"odom_size"<<odomList.size()<<std::endl;
    fileOdom.close();
    while(!fileCamOdom.eof())
    {
        std::vector<double> odom_(8,0.0);
        std::string fileLine;
        getline(fileCamOdom,fileLine);
        std::stringstream ss;
        ss<<fileLine;
        for(int i=0;i<odom_.size();i++) {
            ss >> odom_[i];
        }
        camOdomList.push_back(odom_);
    }
    find_cloast(camOdomList);
    fileCamOdom.close();


#endif
}

FeatureTracker featureTracker;
void loade_img()
{

#if 0
    std::string pathOdom="/media/qcx/D/work/20200804_car23_imu_can_camera_yahui/L1/odom_test.txt";
    std::string pathCamOdom="/media/qcx/D/work/20200804_car23_imu_can_camera_yahui/KeyFrameTrajectory_uisee_l1.txt";
#else
    string strPathToSequence="/home/uisee/20200422_car8_can_imu_camera_yahui/dump_images/image_capturer_0";;
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
#endif
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/image_name.txt";
    fTimes.open(strPathTimeFile.c_str());
    vector<string> imageNameList;

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            string timeStr=s.substr(0,16);
            ss << timeStr;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
            imageNameList.push_back(s);
        }
    }

    string strPrefixLeft = strPathToSequence;

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);
    std::cout<<featureTracker.stereo_cam<<std::endl;
    for(int i=0; i<nTimes; i++)
    {
//        stringstream ss;
//        ss << setfill('0') << setw(6) << i;
//        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageFilenames[i]=strPrefixLeft+"/"+imageNameList[i];
        cv::Mat img;
        img=cv::imread(vstrImageFilenames[i],CV_LOAD_IMAGE_GRAYSCALE);
        cv::imshow("img",img);
        cv::waitKey(1);
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
        featureFrame = featureTracker.trackImage(10, img);
        KeyFrame *keyframe=new KeyFrame(vTimestamps[i], i,img,featureTracker.cur_pts,featureTracker.cur_un_pts,0);
        posegraph.addKeyFrame_uisee(keyframe, 1,1);//第二个参数代表是需要回环检测detect_loop 提取的FAST特征点
    }

}
//闭环检测中每一个关键帧有两类特征点：
//goodFeatureToTrack检测到的点及光流track到的点，这些点在FeatureTracker类中得到
//        KeyFrame类中提取的FAST特征点
int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_fusion");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 1000);
    pub_odometry_rect = n.advertise<nav_msgs::Odometry>("odometry_rect", 1000);


    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;

    if(argc != 2)
    {
        printf("please intput: rosrun loop_fusion loop_fusion_node [config file] \n"
               "for example: rosrun loop_fusion loop_fusion_node "
               "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 0;
    }

    
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);

    std::string IMAGE_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    std::string pkg_path = ros::package::getPath("loop_fusion");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    printf("cam calib path: %s\n", cam0Path.c_str());
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());
    CAM_NAMES.push_back(cam0Path);

    fsSettings["image0_topic"] >> IMAGE_TOPIC;        
    fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    fsSettings["output_path"] >> VINS_RESULT_PATH;
    fsSettings["save_image"] >> DEBUG_IMAGE;

    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
    VINS_RESULT_PATH = VINS_RESULT_PATH + "wheels_odom.txt";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    config_file = argv[1];
//    readParameters(config_file);
//    loade_odom();//读入里程计并优化
    featureTracker.readIntrinsicParameter(CAM_NAMES);
    loade_img();//读入图像
    int USE_IMU = fsSettings["imu"];
    posegraph.setIMUFlag(USE_IMU);//在这里又启动了一个线程
    fsSettings.release();

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("no previous pose graph\n");
        load_flag = 1;
    }



//    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);
    ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
//    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
//    ros::Subscriber sub_margin_point = n.subscribe("/vins_estimator/margin_cloud", 2000, margin_point_callback);



    std::thread measurement_process;
    std::thread keyboard_command_process;

//    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);
    //有一个全局优化线程t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    
    ros::spin();

    return 0;
}
