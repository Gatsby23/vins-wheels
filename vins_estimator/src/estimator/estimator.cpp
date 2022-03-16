/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"//估计器
#include "../utility/visualization.h"

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    while(!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        vel_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  <<"ric"<<endl <<ric[i] << endl <<"tic"<<endl<< tic[i].transpose() << endl;
    }
    tiv = TIV[0];//轮速计到IMU的外参
    riv = RIV[0];
    cout << " exitrinsic vel "  << endl  <<"riv"<<endl <<riv << endl <<"tiv"<<endl<< tiv.transpose() << endl;
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if(!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if(USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }

        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if(restart)
    {
        clearState();
        setParameter();

    }
    mProcess.unlock();
}
//time img0,img1
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
//    for(auto &pos:Ps)
//    {
//        cout<<"PS\n"<<pos<<endl;
//    }
    inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;//map是键-值对的集合，可以理解为关联数组 pair是将2个数据组合成一组数据
    TicToc featureTrackerTime;//时间

    //计算特征点,先跟踪特征点，再计算新的特征点
    if(_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);//左目camera_id=0.右目camera_id=1
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
    }

    if(MULTIPLE_THREAD)
    {
        if(inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));//push入featureBuf队列 这个队列的成员对象一直是一个  t是时间戳，featureframe是特征点数据组
        mBuf.unlock();
        //cout<<"size featureBuf"<<featureBuf.size()<<endl;
        TicToc processTime;
        processMeasurements();//重要，应该是计算位姿的
        if(SHOW_MESSAGE)
            printf("process time_change: %f\n", processTime.toc());
    }

}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    //预测未考虑观测噪声的p、v、q值,同时将发布最新的IMU测量值消息（pvq值）
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
//    printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);//积分出  P V Q 为了输出高速的位姿
//        printf("fastPredictIMU \n");
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        mPropagate.unlock();
    }
}

void Estimator::inputWheelsIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    //预测未考虑观测噪声的p、v、q值,同时将发布最新的IMU测量值消息（pvq值）
    mBuf.lock();
    accWheelsBuf.push(make_pair(t, linearAcceleration));
    gyrWheelsBuf.push(make_pair(t, angularVelocity));
//    printf("input imu with time %f \n", t);
    mBuf.unlock();

//    if (solver_flag == NON_LINEAR)
//    {
//        mPropagate.lock();
//        fastPredictIMU(t, linearAcceleration, angularVelocity);//积分出  P V Q
////        printf("fastPredictIMU \n");
//        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
//        mPropagate.unlock();
//    }
}

void Estimator::inputVEL(double t, const Eigen::Vector3d &velVec, const double &ang_vel)
{
    //预测未考虑观测噪声的p、v、q值,同时将发布最新的IMU测量值消息（pvq值）
    mBuf.lock();
    velBuf.push(make_pair(t, velVec));
//    printf("input imu with time %f \n", t);
    mBuf.unlock();

//    if (solver_flag == NON_LINEAR)
//    {
//        mPropagate.lock();
//        fastPredictIMU(t, linearAcceleration, angularVelocity);//积分出  P V Q
////        printf("fastPredictIMU \n");
//        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
//        mPropagate.unlock();
//    }
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if(!MULTIPLE_THREAD)
        processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::getWHEELSInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &velVector,
                               vector<pair<double, double>> &ang_velVector)
{
    if(velBuf.empty())
    {
        printf("not receive wheels\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= velBuf.back().first)
    {
        while (velBuf.front().first <= t0)
        {
            velBuf.pop();
//            gyrBuf.pop();
        }
        while (velBuf.front().first < t1)
        {
            velVector.push_back(velBuf.front());
            velBuf.pop();
//            gyrVector.push_back(gyrBuf.front());
//            gyrBuf.pop();
        }
        velVector.push_back(velBuf.front());
//        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for wheels\n");
        return false;
    }
    return true;
}

bool Estimator::getWHEELSInterpolation(double t, vector<pair<double, Eigen::Vector3d>> &imuVelVector)
{
//    std::cout<<"t= "<<t<<endl;
    if(velBuf.empty())
    {
        printf("not receive wheels\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t <= velBuf.back().first)
    {
        if(temp_vel.first==0 && t <= velBuf.front().first)//第一次找数据时 t0 比开始时要小。就直接用后一个数据
        {
            imuVelVector.push_back(make_pair(t,velBuf.front().second));
            return true;
        }
        while (velBuf.front().first <= t)//小于该时间戳的都扔掉
        {
            temp_vel = velBuf.front();
            velBuf.pop();
//            gyrBuf.pop();
        }
        pair<double,Eigen::Vector3d>temp_back_t0;//t0后面的一个
        temp_back_t0=velBuf.front();
        double deltaT = t-temp_vel.first;
        double dt = temp_back_t0.first-temp_vel.first;
        Eigen::Vector3d deltaV = temp_back_t0.second-temp_vel.second;
        Eigen::Vector3d vel_t0 = temp_vel.second + deltaV * deltaT / dt;
//        std::cout<<"t= "<<t<<endl;
        imuVelVector.push_back(make_pair(t , vel_t0));
//        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for wheels\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}
bool Estimator::WHEELSAvailable(double t)
{
    if(!velBuf.empty() && t <= velBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        vector<pair<double, Eigen::Vector3d>> velVector,ang_velVector;
        if(!featureBuf.empty())
        {
            feature = featureBuf.front();
            curTime = feature.first + td;
            while(1)//等待轮速计
            {
                if ((!USE_WHEELS  || WHEELSAvailable(feature.first + td)))//有imu数据，且imu accBuf.back数据时间戳大于图像时间戳
                    break;
                else
                {
                    printf("wait for wheels ... \n");
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            while(1)//等待IMU
            {
                if ((!USE_IMU  || IMUAvailable(feature.first + td)))//有imu数据，且imu accBuf.back数据时间戳大于图像时间戳
                    break;
                else
                {
                    printf("wait for imu ... \n");
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            if(USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector);//获取时间间隔内的IMU
//            if(USE_WHEELS)
//            {
//                getWHEELSInterval(prevTime, curTime, velVector,ang_velVector);//获取时间间隔内的轮速计
//                std::cout<<"before  Vs[j]= "<<Vs[frame_count].transpose()<<"\tnorm= "<<Vs[frame_count].norm()<<endl;
//                Vs[frame_count]=velVector.front().second/Vs[frame_count].norm()*Vs[frame_count];
//                std::cout<<"after  Vs[j]= "<<Vs[frame_count].transpose()<<"\tnorm= "<<Vs[frame_count].norm()<<endl;
//                Eigen::Vector3d velVec=velVector.front().second*Eigen::Vector3d::Identity();
//                velVec=Rs[frame_count]*velVec;
//                std::cout<<"velVec= "<<velVec.transpose()<<std::endl;
//            }

            featureBuf.pop();//找到对应图像的imu数据后 特征点的BUFF就pop一个，且，刚开始已经赋值给feature了
            mBuf.unlock();

            if(USE_IMU && !USE_WHEELS)
            {
//                cout<<"处理IMU前的Rs!!!!!\n"<<Rs[frame_count]<<endl;
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);//初始化IMU旋转，使其Z与g平行
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0)//第一个imu数据的时间戳减去上一次prevTime的时间戳
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)//最后一个IMU时间戳
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;//中间数据的时间戳
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);//滑动窗口帧间IMU积分，
                }
//                cout<<"处理IMU后的Rs\n"<<Rs[frame_count]<<endl;
                Eigen::Vector3d acc_without_g=Rs[frame_count] * (accVector[int(accVector.size())-1].second - Bas[frame_count]) ;//- g;
                Eigen::Vector3d r_acc_=Rs[frame_count] * (accVector[int(accVector.size())-1].second ) ;//- g;
                double length=sqrt(pow(accVector[int(accVector.size())-1].second.x(),2)+pow(accVector[int(accVector.size())-1].second.y(),2)+pow(accVector[int(accVector.size())-1].second.z(),2));
                if(SHOW_MESSAGE) cout<<"length="<<length<<"   acc_origion="<<accVector[int(accVector.size())-1].second.transpose()<<"       acc_without_g"<<acc_without_g.transpose()<<"  r_acc_="<<r_acc_.transpose()<<"  g="<<g.transpose()<<endl;
                //if(abs(length-9.8)<0.05)
                    writr_imu_data(accVector[int(accVector.size())-1].first,length,accVector[int(accVector.size())-1].second,acc_without_g,r_acc_);
                if(SHOW_MESSAGE)
                    printf("------------------------processIMU \n");
            }
            else if(USE_IMU && USE_WHEELS)
            {
//                cout<<"处理IMU前的Rs!!!!!\n"<<Rs[frame_count]<<endl;
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);//初始化IMU旋转，使其Z与g平行
                if(SHOW_MESSAGE)
                    std::cout<<"before pre_integrations Vs"<<Vs[frame_count].transpose()<<"\tnorm= "<<Vs[frame_count].norm()<<std::endl;
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0)//第一个imu数据的时间戳减去上一次prevTime的时间戳
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)//最后一个IMU时间戳
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;//中间数据的时间戳

                    getWHEELSInterpolation(accVector[i].first, velVector);//获取时间间隔内的轮速计
                    if(i==0 && solver_flag == NON_LINEAR)
                    {
                        vector<pair<double, Eigen::Vector3d>> velVector_temp;
                        getWHEELSInterpolation(prevTime, velVector_temp);//获取时间间隔内的轮速计
                        Eigen::Vector3d velVec = velVector_temp.front().second;
                        if(SHOW_MESSAGE)
                            std::cout<<"velVec="<<velVec.transpose()<<endl;
                        velVec = Rs[frame_count] * velVec;
//                        Vs[frame_count]=velVec;
//                        std::cout<<"use vel Vs"<<setprecision(17)<<prevTime<<"\tvs="<<Vs[frame_count].transpose()<<"\tnorm= "<<Vs[frame_count].norm()<<std::endl;
                    }
//                    std::cout<<"getWHEELSInterpolation "<<accVector[i].first<<"  "<<velVector[i].first<<" "<<velVector[i].second<<endl;
                    processIMU_with_wheel(accVector[i].first, dt, accVector[i].second, gyrVector[i].second, velVector[i].second);//滑动窗口帧间IMU积分，
                }
//                cout<<"处理IMU后的Rs\n"<<Rs[frame_count]<<endl;
                Eigen::Vector3d acc_without_g=Rs[frame_count] * (accVector[int(accVector.size())-1].second - Bas[frame_count]) ;//- g;
                Eigen::Vector3d r_acc_=Rs[frame_count] * (accVector[int(accVector.size())-1].second ) ;//- g;
                double length=sqrt(pow(accVector[int(accVector.size())-1].second.x(),2)+pow(accVector[int(accVector.size())-1].second.y(),2)+pow(accVector[int(accVector.size())-1].second.z(),2));
                if(SHOW_MESSAGE)
                    cout<<"length="<<length<<"   acc_origion="<<accVector[int(accVector.size())-1].second.transpose()<<"       acc_without_g"<<acc_without_g.transpose()<<"  r_acc_="<<r_acc_.transpose()<<"  g="<<g.transpose()<<endl;
                //if(abs(length-9.8)<0.05)
                writr_imu_data(accVector[int(accVector.size())-1].first,length,accVector[int(accVector.size())-1].second,acc_without_g,r_acc_);
                writr_integrate_data(OUTPUT_FOLDER+"imu_int_origin.csv");
                if(SHOW_MESSAGE)
                    printf("------------------------processIMU \n");
            }

            mProcess.lock();
            processImage(feature.second, feature.first);//重要   特征点相关，时间戳// 处理图像 和IMU
            writr_ece(OUTPUT_FOLDER+"exe.csv");//写外参
            if(SHOW_MESSAGE){
                std::cout<<"para_Ex_Pose "<<para_Ex_Pose[0][0]<<" "<<para_Ex_Pose[0][1] <<" "<<para_Ex_Pose[0][2] <<" "<<
                         para_Ex_Pose[0][3] <<" "<<para_Ex_Pose[0][4] <<" "<<para_Ex_Pose[0][5] <<" "<<para_Ex_Pose[0][6] <<std::endl;
            }
            prevTime = curTime;
            if(SHOW_MESSAGE)
                std::cout<<"latest_V= "<<latest_V.transpose()<<std::endl;
            printStatistics(*this, 0);//打印统计信息

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);

            pubOdometry(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
            mProcess.unlock();
        }

        if (! MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
//存储IMU数据
void Estimator::writr_imu_data(double time,double length_,Eigen::Vector3d acc_ori,Eigen::Vector3d acc_whithout_g,Eigen::Vector3d R_acc_)
{
    // write result to file
    string imu_data_path=OUTPUT_FOLDER+"imu_2.csv";
    ofstream foutC(imu_data_path, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << inputImageCnt << ",";
    foutC.precision(7);
    foutC << length_<<","
          <<acc_ori.x() << ","
          << acc_ori.y() << ","
          << acc_ori.z() << ","
          << acc_whithout_g.x() << ","
          << acc_whithout_g.y() << ","
          << acc_whithout_g.z() << ","
          << R_acc_.x() << ","
          << R_acc_.y() << ","
          << R_acc_.z() << ","
          <<Bas->x()<<","
          <<Bas->y()<<","
          <<Bas->z()<<","
          <<frame_count<<","
          <<marginalization_flag<<","
          << endl;
    foutC.close();
}

void Estimator::writr_integrate_data(string path)
{
    // write result to file
    string imu_data_path=OUTPUT_FOLDER+"imu_2.csv";
    imu_data_path = path;
    ofstream foutC(imu_data_path, ios::app);//ios::app insert data from file end;
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << inputImageCnt << ",";
    foutC.precision(7);
    if(pre_integrations[frame_count]!= nullptr)
    {
//        std::cout<<"write "<<pre_integrations[frame_count]->delta_p_i_vel<<endl;
        Eigen::Matrix3d delta_R = pre_integrations[frame_count]->delta_q.toRotationMatrix();
        Eigen::AngleAxis<double> angleaxis;
        angleaxis.fromRotationMatrix(delta_R);
        foutC << pre_integrations[frame_count]->delta_p_i_vel[0]<<","
              <<pre_integrations[frame_count]->delta_p_i_vel[1] << ","
              << pre_integrations[frame_count]->delta_p_i_vel[2] << ","
              << para_Pose[frame_count][0]<<","
              << para_Pose[frame_count][1]<<","
              << para_Pose[frame_count][2]<<","
              << angleaxis.angle()*180.0f/M_PI<<","
              << angleaxis.axis().x()<<","
              << angleaxis.axis().y()<<","
              << angleaxis.axis().z()<<","
              <<frame_count<<","
              <<marginalization_flag<<","
              <<pre_integrations[frame_count]->delta_angleaxis.angle()*180.0f/M_PI/pre_integrations[frame_count]->sum_dt<<","
              << endl;
        foutC.close();
    }
}

void Estimator::writr_ece(string path)
{
    // write result to file
    string imu_data_path=OUTPUT_FOLDER+"imu_2.csv";
    imu_data_path = path;
    ofstream foutC(imu_data_path, ios::app);//ios::app insert data from file end;
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << inputImageCnt << ",";
    foutC.precision(7);
    if(pre_integrations[frame_count]!= nullptr)
    {
//        std::cout<<"write "<<pre_integrations[frame_count]->delta_p_i_vel<<endl;
        Eigen::Quaterniond exe_q;
//        exe_q.x()=para_Ex_Pose[0][3];
//        exe_q.y()=para_Ex_Pose[0][4];
//        exe_q.z()=para_Ex_Pose[0][5];
//        exe_q.w()=para_Ex_Pose[0][6];
        exe_q = ric[0];
        Eigen::Matrix3d exe_R = exe_q.toRotationMatrix();
        Eigen::AngleAxis<double> angleaxis;
        Eigen::Vector3d Euler_exe = exe_R.eulerAngles(2,1,0);
        foutC <<    tic[0].x()<<","
              <<    tic[0].y() << ","
              <<    tic[0].z()<< ","
              <<    exe_q.x()<<","
              <<    exe_q.y()<<","
              <<    exe_q.z()<<","
              <<    exe_q.w()<<","
              << Euler_exe.x()*180.0f/M_PI<<","
              << Euler_exe.y()*180.0f/M_PI<<","
              << Euler_exe.z()*180.0f/M_PI<<","
              << endl;
        foutC.close();
    }
}


void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);//主要是为了做一个重力对齐
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
//    R0 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d EulerR0 = R0.eulerAngles(2,1,0);
    std::cout<<"EulerR0= "<<180.0f/M_PI*EulerR0.transpose()<<std::endl;

    Rs[0] = R0;
    Eigen::Matrix3d Disturb;
    Disturb<<              1 ,-0.414   ,   1,
    1   ,   1 ,-0.414,
           -0.414 ,     1  ,    1;
    Rs[0]=Disturb*Rs[0];
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

//在imu迭代里 循环调用 处理imu
void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{

    // 1.imu未进来数据
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
// 2.IMU 预积分类对象还没出现，创建一个
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        // 3.预积分操作  是为了初始化的预积分
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // 4.dt、加速度、角速度加到buf中
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        // 5.采用的是中值积分的传播方式  这时是积分  IMU预积分出相机位姿先验
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;//滑动窗口的Rs[(WINDOW_SIZE + 1)];
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
        if(SHOW_MESSAGE){
            std::cout<<"pre_integrations time"<<setprecision(17)<<t<<"  Vs[j]= "<<Vs[j].transpose()<<"\tnorm= "<<Vs[j].norm()
                     //        <<"\tBgs[j]= "<<Bgs[j].transpose()<<"\tBas[j]= "<<Bas[j].transpose()
                     <<endl;
        }
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processIMU_with_wheel(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity ,const Eigen::Vector3d vel)
{

    // 1.imu未进来数据
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        vel_0 = vel;
    }
// 2.IMU 预积分类对象还没出现，创建一个
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, vel_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        // 3.预积分操作  是为了初始化的预积分
       // Eigen::Vector3d velVec=Eigen::Vector3d(vel,0,0);
        Eigen::Vector3d velVec=vel;
        if(velVec.norm() == 0)
        {
            findBias( dt, angular_velocity,linear_acceleration);
            Eigen::Vector3d accAve = BiasAccSum/BiasAccCnt;
            Eigen::Matrix3d R0 = Utility::g2R(accAve);//主要是为了做一个重力对齐
            Eigen::Vector3d euler = toEuler(BiasR0)*180.0/M_PI;
            std::cout<<"BiasCnt "<<BiasCnt<<"  BiasAverage "<<BiasAverage.transpose()<<"\tBiasAccCnt "<<BiasAccCnt<<" euler: "<<euler.transpose()<<std::endl;
        }
        else
        {
            BiasAccSum = Eigen::Vector3d::Zero();
            BiasAccCnt = 0;
        }
        pre_integrations[frame_count]->push_back_wheels(dt, linear_acceleration, angular_velocity , velVec);
//        cout<<"pre_integrations vel_0 "<<pre_integrations[frame_count]->vel_0.transpose()<<endl;

        Eigen::Vector3d delta_p_imu = pre_integrations[frame_count]->delta_q * (-tiv) + riv * pre_integrations[frame_count]->delta_p_i_vel + tiv;
//        std::cout<<setprecision(6)
//        <<"pre_integrations: delta_p="<<pre_integrations[frame_count]->delta_p.transpose()
//        <<"\tdelta_p_i_vel="<<pre_integrations[frame_count]->delta_p_i_vel.transpose()
//        <<"\tdelta_v="<<pre_integrations[frame_count]->delta_v.transpose()
//        <<"\t delta_p_imu="<<delta_p_imu.transpose()
//        <<endl;

        //if(solver_flag != NON_LINEAR)
//        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
        tmp_pre_integration->push_back_wheels(dt, linear_acceleration, angular_velocity , velVec);
//        cout<<"tmp_pre_integration vel_0 "<<tmp_pre_integration->vel_0.transpose()<<endl;
        // 4.dt、加速度、角速度加到buf中
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        vel_velocity_buf[frame_count].push_back(velVec);

        int j = frame_count;
        // 5.采用的是中值积分的传播方式  这时是积分  IMU预积分出相机位姿先验
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;//滑动窗口的Rs[(WINDOW_SIZE + 1)];
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
//        Eigen::Matrix3d delta_R_rpy=(Eigen::AngleAxisd(un_gyr(2)*dt,Eigen::Vector3d::UnitZ())
//                                     *Eigen::AngleAxisd(un_gyr(1)*dt,Eigen::Vector3d::UnitY())
//                                     *Eigen::AngleAxisd(un_gyr(0)*dt,Eigen::Vector3d::UnitX())).toRotationMatrix();
//        Eigen::Quaterniond delta_q_normed =  Utility::deltaQ(un_gyr * dt);
//        delta_q_normed.normalize();
//        Rs[j] *= delta_q_normed.toRotationMatrix();
//        Rs[j] *= delta_R_rpy;
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);//有二阶龙格库塔的效果
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
        Eigen::Matrix4d T_i_v=Eigen::Matrix4d::Identity();
        T_i_v.matrix().block<3,3>(0,0)=riv;
        T_i_v.matrix().block<3,1>(0,3)=tiv;
        Eigen::Matrix4d T_v=Eigen::Matrix4d::Identity();
        T_v.matrix().block<3,1>(0,3)=vel;
        T_v.matrix().block<3,3>(0,0)=Utility::deltaQ(un_gyr * dt).toRotationMatrix();
//        Eigen::Vector3d Vels=Rs[j]*Eigen::Vector3d(vel,0,0);
        T_v=T_i_v*T_v*(T_i_v.inverse());
        Eigen::Vector3d Vels=Rs[j] * T_v.matrix().block<3,1>(0,3);
//        if(frame_count>2)
//        {
//            std::cout<<"pre_integrations time "<<setprecision(17)<<t
//                     <<"\tdelta P[i]"<<(Ps[j]-Ps[j-1]).transpose() <<"\tVs[j]="<<Vs[j].transpose()<<"\t vel= "<<vel<<endl;
//        }
//                 <<setprecision(10)<<"\tVs[j]="<<Vs[j].transpose()<<"\tnorm= "<<Vs[j].norm()
//                 <<"\nvel= "<<vel<<"\t Vels="<<Vels.transpose()<<"\t norm="<<Vels.norm()<<"\traw vel="<<vel
////                 <<"\nT_v"<<T_v
//                 //        <<"\tBgs[j]= "<<Bgs[j].transpose()<<"\tBas[j]= "<<Bas[j].transpose()
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    vel_0 = vel;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    //image的数据类型分别表示feature_id,camera_id,点的x,y,z坐标，u,v坐标，在x,y方向上的跟踪速度
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    //为了保证系统的实时性和准确性，需要对当前帧之前某一部分帧进行优化，而不是全部历史帧，优化帧的个数便是滑动窗口的大小。
    //不难理解，为了维持窗口大小，要去除旧的帧添加新的帧，即边缘化 Marginalization。到底是删去最旧的帧（MARGIN_OLD）还是
    //删去刚刚进来窗口倒数第二帧(MARGIN_SECOND_NEW)，就需要对 当前帧与之前帧 进行视差比较，如果是当前帧变化很小，就会删
    //去倒数第二帧，如果变化很大，就删去最旧的帧 --- 理解一下这个意思！
    //检查视差代码[1]根据视差来决定marg掉哪一帧，如果次新帧是关键帧，marg（边缘化）掉最老帧，如果次新帧不是关键帧,marg掉次新帧
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))//关键帧计数器 关键帧特征点 td常为0  往添加f_manager成员 如右目特征点
    {
        marginalization_flag = MARGIN_OLD;//边缘化标志
        if(SHOW_MESSAGE){
            printf("keyframe\n");
            cout<<"frame_count "<<frame_count<<" td "<<td<<endl;
        }
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        if(SHOW_MESSAGE){
            printf("non-keyframe\n");
        }
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());//计算滑窗内被track过的特征点的数量
    Headers[frame_count] = header; //frame_counter最后基本全是10

    //【2】将图像数据、时间、临时预积分值存储到图像帧类中,ImageFrame这个类的定义在initial_alignment.h中
    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, vel_0, Bas[frame_count], Bgs[frame_count]};

    //[3]如果ESTIMATE_EXTRINSIC == 2表示需要在线估计imu和camera之间的外参数
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            //寻找和当前帧和上一帧对应的关键帧序号
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            //计算前后帧之间的旋转
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;//表示外参估计只计算一次 只优化旋转
            }
        }
    }

    //[4]判断是初始化还是非线性优化

    if (solver_flag == INITIAL)
    {
        //纯视觉sfm
        if (STEREO==0 && !USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialSfm();
                    initial_timestamp = header;
                }
                if(result)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;
                }
                if(result)
                {
                    initResult = true;
                    ofstream stateSave(OUTPUT_FOLDER + "/init_state.txt");
//                    stateSave.open((OUTPUT_FOLDER + "/state.txt").c_str() );
                    stateSave<<fixed;
                    for (int i = 0; i <= frame_count; i++)
                    {
                        stateSave.precision(6);
                        stateSave <<"init before opt " <<i<< "\t";
                        stateSave.precision(5);
                        stateSave <<"Ps: "<<Ps[i].x() << ","<< Ps[i].y() << ","<< Ps[i].z() << "\t"
                                  <<"Vs: "<< Vs[i].x() << ","<< Vs[i].y() << ","<< Vs[i].z() <<","<<Vs[i].norm()<< "\t"
                                  <<"Bas: "<<Bas[i].x()<<","<<Bas[i].y()<<","<<Bas[i].z()<< "\t"
                                  <<"Bgs: "<<Bgs[i].x()<<","<<Bgs[i].y()<<","<<Bgs[i].z()
                                  <<endl;
                    }
//                    std::cout<<"s= "<<s<<std::endl;
                    optimization();

                    for (int i = 0; i <= frame_count; i++)
                    {
                        stateSave.precision(6);
                        stateSave <<"init after opt " <<i<< "\t";
                        stateSave.precision(5);
                        stateSave <<"Ps: "<<Ps[i].x() << ","<< Ps[i].y() << ","<< Ps[i].z() << "\t"
                                  <<"Vs: "<< Vs[i].x() << ","<< Vs[i].y() << ","<< Vs[i].z() <<","<<Vs[i].norm()<< "\t"
                                  <<"Bas: "<<Bas[i].x()<<","<<Bas[i].y()<<","<<Bas[i].z()<< "\t"
                                  <<"Bgs: "<<Bgs[i].x()<<","<<Bgs[i].y()<<","<<Bgs[i].z()
                                  <<endl;
                    }

                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if(STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);//陀螺仪bias
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);//双目+IMU 重新积分
                }
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if(STEREO && !USE_IMU)
        {

            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);//tic[0]左相机T tic[1] 右相机T ric[0]左相机R ric[1]右相机R//在这里改变了Ps和Rs
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);//双目三角化三维坐标

            optimization();//非线性最小二乘优化重投影误差

            if(frame_count == WINDOW_SIZE)
            {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;//初始化完毕，程序正常运行
                slideWindow();
                ROS_INFO("Initialization finish!");
                std::cout<<"Initialization finish!"<<std::endl;
            }
        }

        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else//非线性优化
    {
        TicToc t_solve;
        if(!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);//pnp求位姿
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);//三角化 depth
//        std::cout<<"ps\n"
//        <<Ps[0].transpose()<<"\n PS_1: "<<Ps[1].transpose()<<"\nPS_2: "<<Ps[2].transpose()<<"\nPS_3: "<<Ps[3].transpose()<<"\nPS_4:"<<Ps[4].transpose()<<"\nPS_5:"<<Ps[5].transpose()
//        <<"\nPS_6:"<<Ps[6].transpose()<<"\nPS_7:"<<Ps[7].transpose()<<"\nPS_8:"<<Ps[8].transpose()<<"\nPS_9:"<<Ps[9].transpose()<<"\nPS_10:"<<Ps[10].transpose()<<"\n---------PS------------"<<std::endl;
        optimization();//优化_used
//        writr_integrate_data(OUTPUT_FOLDER+"imu_int_after_opt.csv");
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (! MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }

        ROS_DEBUG("solver costs: %fms", t_solve.toc()); //异常检测与恢复, 检测到异常，系统将切换回初始化阶段

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();//滑窗
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
        std::cout<<setprecision(17)<<frame_it->first<<"\t T="<<frame_it->second.T.transpose()<<endl;
    }
    if (visualInitialAlign())//视觉惯性对齐
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}
//纯视觉初始化
bool Estimator::initialSfm()
{
    TicToc t_sfm;
    //check imu observibility
    if(0)
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
                      relative_R, relative_T,
                      sfm_f, sfm_tracked_points))
    {
        ROS_INFO("global SFM failed!");
        std::cout<<"global SFM failed!"<<std::endl;
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            //continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }

    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return true;
    }

}

bool Estimator::visualInitialAlign()//视觉惯性对齐
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);//视觉惯性对齐 重新积分
        std::cout<<setprecision(6)
                 <<"repropagate : delta_p="<<pre_integrations[i]->delta_p.transpose()
                 <<"\tdelta_p_i_vel="<<pre_integrations[i]->delta_p_i_vel.transpose()
                 <<"\tdelta_v="<<pre_integrations[i]->delta_v.transpose()
                 <<"\t t_imu"<< (TIV[0] - pre_integrations[i]->delta_q * TIV[0] + pre_integrations[i]->delta_p_i_vel).transpose()
                 <<endl;
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1, kfv=-1;
    map<double, ImageFrame>::iterator frame_i;
//    std::cout<<"x=\n"<<x<<std::endl;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        kv++;
        if(frame_i->second.is_key_frame)
        {
            kfv++;
            Vs[kfv] = frame_i->second.R * x.segment<3>(kv * 3);
            std::cout<<"time= "<<setprecision(17)<<frame_i->first<<"\tVs[kv]="<<Vs[kv].transpose()<<std::endl;
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    std::cout<<"g after change state"<<g.transpose()<<std::endl;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
//        std::cout<<"PS[i]"<<Ps[i].transpose()<<endl;
        if(i>0)
            std::cout<<"curTime="<<setprecision(17)<<curTime<<"\tPS[i]-PS[i-1]"<<(Ps[i]-Ps[i-1]).transpose()<<endl;
    }
    for (int i = 0; i <= frame_count; i++)
    {
        std::cout<<"PS[i]= "<<Ps[i].transpose()<<"\tVs[i]="<<Vs[i].transpose()<<endl;
    }
    std::cout<<"s= "<<s<<std::endl;
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());
    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_INFO("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if(USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);
//                Bas[i].y() = 0;

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);

                if(BiasCnt>3000)
                {
                    double decreaseBias = 1e-3;
//                    GYR_W =0.000001;//TODO 改成可调参数
//                    Bgs[i] = BiasAverage;
//Bgs[i] = 0.5*BiasAverage+0.5*Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8]);
                }

        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if(USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if(USE_IMU)
        td = para_Td[0][0];

}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        //return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    //初始化ceres
    ceres::Problem problem;//创建一个ceres Problem实例, loss_function定义为CauchyLoss.
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(5.0);
//    loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    double cnt_1 = 0, cnt_5 = 0, cnt_large_5 = 0;
    if(1){
        cv::Mat imgTrack = featureTracker.getTrackImage();
        int f_m_cnt = 0;
        int feature_index = -1;
        for (auto &it_per_id: f_manager.feature) {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;
            ++feature_index;
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;
            for (auto &it_per_frame: it_per_id.feature_per_frame) {
                imu_j++;
                if (imu_i != imu_j) {
                    Vector3d pts_j = it_per_frame.point;
                    Eigen::Vector3d tic(para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2]);//外参
                    Eigen::Quaterniond qic(para_Ex_Pose[0][6], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5]);
                    Eigen::Vector3d Pi(para_Pose[imu_i][0], para_Pose[imu_i][1], para_Pose[imu_i][2]);
                    Eigen::Quaterniond Qi(para_Pose[imu_i][6], para_Pose[imu_i][3], para_Pose[imu_i][4],
                                          para_Pose[imu_i][5]);
                    Eigen::Vector3d Pj(para_Pose[imu_j][0], para_Pose[imu_j][1], para_Pose[imu_j][2]);
                    Eigen::Quaterniond Qj(para_Pose[imu_j][6], para_Pose[imu_j][3], para_Pose[imu_j][4],
                                          para_Pose[imu_j][5]);
                    double inv_dep_i = para_Feature[feature_index][0];
                    double td = para_Td[0][0];
                    Eigen::Vector3d velocity_i, velocity_j;
                    velocity_i.x() = it_per_id.feature_per_frame[0].velocity.x();
                    velocity_i.y() = it_per_id.feature_per_frame[0].velocity.y();
                    velocity_i.z() = 0;
                    velocity_j.x() = it_per_frame.velocity.x();
                    velocity_j.y() = it_per_frame.velocity.y();
                    velocity_j.z() = 0;
                    double td_i = it_per_id.feature_per_frame[0].cur_td;
                    double td_j = it_per_frame.cur_td;
                    Eigen::Vector3d pts_i_td, pts_j_td;
                    pts_i_td = pts_i - (td - td_i) * velocity_i;
                    pts_j_td = pts_j - (td - td_j) * velocity_j;
                    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;
                    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
                    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
                    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
                    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
                    Eigen::Vector2d residual;
                    double dep_j = pts_camera_j.z();
                    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
                    Eigen::Matrix3d PI;
                    residual = ProjectionTwoFrameOneCamFactor::sqrt_info * residual;
                    PI<< 8.1690378992770002e+02, 0, 6.0850726281690004e+02, 0, 8.1156803828490001e+02, 2.6347599764440002e+02, 0, 0, 1;
                    if (0) {
                        std::cout << "cam residual: (res_x res_y) : " << residual.transpose() << " \tin :"
                                  << (PI * pts_j).transpose() << std::endl;
                    }
                    Eigen::Vector3d Pt = (PI * pts_j);
                    double r = residual.norm();
                    cv::Point2f rightPt;
                    rightPt.x = int(Pt.x());
                    rightPt.y = int(Pt.y());
                    if (r < 1) {
                        cv::circle(imgTrack, rightPt, 1, cv::Scalar(0, 255, 0), 2);
                        cnt_1++;
                    } else if (r < 5) {
                        cv::circle(imgTrack, rightPt, r, cv::Scalar(0, 255, 255), 2);
                        cnt_5++;
                    } else {
                        cnt_large_5++;
                        cv::circle(imgTrack, rightPt, r, cv::Scalar(0, 0, 0), 2);
                    }
                }
                f_m_cnt++;
            }
        }
        if (1)//SHOW_MESSAGE)
        {
            cv::putText(imgTrack, "cnt_1: " + to_string(cnt_1), cv::Point2f(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 0, 255));
            cv::putText(imgTrack, "cnt_5: " + to_string(cnt_5), cv::Point2f(10, 45), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 0, 255));
            cv::putText(imgTrack, "cnt_large_5: " + to_string(cnt_large_5), cv::Point2f(10, 60), CV_FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(0, 0, 255));
            cv::putText(imgTrack, "inputImageCnt: " + to_string(inputImageCnt), cv::Point2f(10, 75), CV_FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(0, 0, 255));
            cv::putText(imgTrack, "imuSumT: " + to_string(pre_integrations[0]->sum_dt), cv::Point2f(10, 90), CV_FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(0, 0, 255));
            cv::imshow("imgTrack", imgTrack);
            cv::waitKey(1);
        }
    }

    //加入优化变量  位姿 速度
    for (int i = 0; i < frame_count + 1; i++)
    {
        bool show=false;
        if(i==frame_count-1 && SHOW_MESSAGE) show = true;
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization(show);//本地参数化
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);//参数_位姿
        if(USE_IMU)
        {
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
        }
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);//这个函数是设置参数块为常数  将帧头设置为零


    //加入相机外参
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            if(SHOW_MESSAGE)
                ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            if(SHOW_MESSAGE)
                ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);//将imu和图像时间戳偏差设为定值

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);//边缘化
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            if(SHOW_MESSAGE)
            {
                std::cout<<"frame_count: "<<i<<"\t sumdt="<<setprecision(5)<<pre_integrations[j]->sum_dt
                         <<"\t pre del_p_vel="<<pre_integrations[j]->delta_p_i_vel.transpose()<<"\tdelta_v="<<pre_integrations[j]->delta_v.transpose()
                         <<"\tpos j:"<<para_Pose[j][0]<<"  "<<para_Pose[j][1]<<"  "<<para_Pose[j][2]<<endl;
                std::cout << "covariance_enc_i\n" << pre_integrations[i]->covariance_enc << std::endl;
                std::cout << "covariance_enc_j\n" << pre_integrations[j]->covariance_enc << std::endl;
            }
            bool show=false;
            if(i==frame_count-2 && SHOW_MESSAGE) show = true;
            if (IMU_FACTOR == 0) {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[j], show);
                problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                                         para_SpeedBias[j]);
            } else if (IMU_FACTOR == 1) {
                IMUFactor_origin *imu_factor = new IMUFactor_origin(pre_integrations[j]);
                problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                                         para_SpeedBias[j]);
            } else {
                double wheelVel = 0;
                double imuSumT = pre_integrations[j]->sum_dt;
//                if(imuSumT>2)
//                    continue;
                int angVelOrigin = fabs(pre_integrations[j]->delta_angleaxis.angle()*180.0f/M_PI/pre_integrations[j]->sum_dt);
                int angVElDif = int((fabs(pre_integrations[j]->delta_angleaxis.angle() * 180.0f / M_PI / pre_integrations[j]->sum_dt) - MAX_ANGVEL_BIAS) );
                Eigen::Vector3d EulerDelta = toEuler(pre_integrations[j]->delta_q.toRotationMatrix());
                angVElDif = int((fabs(EulerDelta.z() * 180.0f / M_PI / pre_integrations[j]->sum_dt) - MAX_ANGVEL_BIAS) );
                int decrease = 0;
                if(angVElDif < 1)
                    decrease = int((cnt_1-MAX_CNT_1)/100) + angVElDif*2;
                else
                    decrease = int((cnt_1-MAX_CNT_1)/100)+2;
                if(decrease>0)
                    decrease=0;
//                std::cout<<"decrease:"<<decrease<<"\tangVEl:"<<angVElDif<<std::endl;
                IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[j], show,decrease*2);
                problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                                         para_SpeedBias[j]);
            }
//            IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[j], show);
//            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
//                                     para_SpeedBias[j]);
        }
    }
//    if(0)
    if(0)
    {
//        problem.SetParameterBlockConstant(para_Pose[0]);//将imu和图像时间戳偏差设为定值
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            WHEELSFactor *wheels_factor = new WHEELSFactor(pre_integrations[j]);
            problem.AddResidualBlock(wheels_factor, NULL,para_Pose[i], para_Pose[j]);
        }
    }
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
//            break;
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if(STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }

            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    if(SHOW_MESSAGE)
        options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
//    if (marginalization_flag == MARGIN_OLD)
//        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
//    else
//        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    if(SHOW_MESSAGE)
    {
        std::cout << "summary.BriefReport()\n" << summary.BriefReport() << "\n";
//    std::cout << summary.message << "\n";
//    std::cout << summary.FullReport() << "\n";
        printf("solver costs: %f \n", t_solver.toc());
    }

    double2vector();
    //printf("frame_count: %d \n", frame_count);
//    return ;  //!!!!!!!!!!!!!!!!!!!!!!!
    if(frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor  构建新的marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                if (IMU_FACTOR == 0) {
                    IMUFactor *imu_factor = new IMUFactor(pre_integrations[1], false);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                } else if (IMU_FACTOR == 1) {
                    IMUFactor_origin *imu_factor = new IMUFactor_origin(pre_integrations[1]);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                } else {
                    IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[1], false);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
//                IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[1], false);
//                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
//                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }
        if(0)//(USE_WHEELS)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
//                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                WHEELSFactor* wheels_factor = new WHEELSFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(wheels_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;

    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];


            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::optimizationBias() //TODO 添加优化bias的滑动窗口
{
    TicToc t_whole, t_prepare;
    vector2double();

    //初始化ceres
    ceres::Problem problem;//创建一个ceres Problem实例, loss_function定义为CauchyLoss.
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    //加入优化变量  位姿 速度
    for (int i = 0; i < frame_count + 1; i++)
    {
        bool show=false;
        if(i==frame_count-1) show = true;
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization(show);//本地参数化
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);//参数_位姿
        if(USE_IMU)
        {
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
        }
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);//这个函数是设置参数块为常数  将帧头设置为零


    //加入相机外参
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);//将imu和图像时间戳偏差设为定值

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);//边缘化
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            if(SHOW_MESSAGE)
            {
                std::cout<<"frame_count: "<<i<<"\t sumdt="<<setprecision(5)<<pre_integrations[j]->sum_dt
                         <<"\t pre del_p_vel="<<pre_integrations[j]->delta_p_i_vel.transpose()<<"\tdelta_v="<<pre_integrations[j]->delta_v.transpose()
                        <<"\tpos j:"<<para_Pose[j][0]<<"  "<<para_Pose[j][1]<<"  "<<para_Pose[j][2]<<endl;
            }
            bool show=false;
            if(i==frame_count-1) show = true;
            if (IMU_FACTOR == 0) {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[j], show);
                problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                                         para_SpeedBias[j]);
            } else if (IMU_FACTOR == 1) {
                IMUFactor_origin *imu_factor = new IMUFactor_origin(pre_integrations[j]);
                problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                                         para_SpeedBias[j]);
            } else {
                IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[j], show);
                problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
                                         para_SpeedBias[j]);
            }
//            IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[j], show);
//            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],
//                                     para_SpeedBias[j]);
        }
    }
//    if(0)
    if(0)
    {
//        problem.SetParameterBlockConstant(para_Pose[0]);//将imu和图像时间戳偏差设为定值
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            WHEELSFactor *wheels_factor = new WHEELSFactor(pre_integrations[j]);
            problem.AddResidualBlock(wheels_factor, NULL,para_Pose[i], para_Pose[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
//            break;
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if(STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }

            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
//    if (marginalization_flag == MARGIN_OLD)
//        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
//    else
//        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
//    cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    if(SHOW_MESSAGE)
        std::cout << "summary.BriefReport()\n" << summary.BriefReport() << "\n";
//    std::cout << summary.message << "\n";
//    std::cout << summary.FullReport() << "\n";
    if(SHOW_MESSAGE)
        printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    //printf("frame_count: %d \n", frame_count);
//    return ;  //!!!!!!!!!!!!!!!!!!!!!!!
    if(frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor  构建新的marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                if (IMU_FACTOR == 0) {
                    IMUFactor *imu_factor = new IMUFactor(pre_integrations[1], false);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                } else if (IMU_FACTOR == 1) {
                    IMUFactor_origin *imu_factor = new IMUFactor_origin(pre_integrations[1]);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                } else {
                    IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[1], false);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
//                IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[1], false);
//                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
//                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }
        if(0)//(USE_WHEELS)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
//                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                WHEELSFactor* wheels_factor = new WHEELSFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(wheels_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;

    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];


            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                    vel_velocity_buf[i].swap(vel_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                //重新更新预积分的变量
                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0,vel_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
                vel_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                double velNormal = 0;
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    Vector3d tmp_vel_velocity = vel_velocity_buf[frame_count][i];
                    velNormal = velNormal  + tmp_vel_velocity.norm();
                }
                if(1)//(inputImageCnt<500)
                    velNormal = 1;
                for (unsigned int i = 0; (i < dt_buf[frame_count].size()) && (velNormal!=0); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
                    Vector3d tmp_vel_velocity = vel_velocity_buf[frame_count][i];
//                    if(tmp_vel_velocity.norm() != 0)//轮速为0 舍弃  可以降低停车启动时的向下漂移
//                        continue;
                    //把次新帧去掉，保留最新帧，但是把次新帧的IMU信息留着，即在次新帧的基础上把当前帧IMU信息添加进去，使得IMU信息更新为最新帧的。
                    pre_integrations[frame_count - 1]->push_back_wheels(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity,tmp_vel_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                    vel_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }
                if(velNormal!=0)
                {
                    Vs[frame_count - 1] = Vs[frame_count];
                    Bas[frame_count - 1] = Bas[frame_count];
                    Bgs[frame_count - 1] = Bgs[frame_count];
                }

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, vel_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
                vel_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getVelInWorldFrame(Eigen::Vector3d &v)
{
    v=Vs[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(STEREO && it_per_frame.is_stereo)
            {

                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
            }
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);

    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    //中值法
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}

void Estimator::findBias(double dt, const Eigen::Vector3d &gyr,const Eigen::Vector3d &acc)
{
    BiasCnt++;
    BiasAccCnt++;
    BiasSum = BiasSum + gyr;
    BiasAverage = BiasAverage*0.999 + gyr*0.001;
    BiasAccSum = BiasAccSum + acc;
    BiasR0 = Utility::g2R(BiasAccSum/BiasAccCnt);//主要是为了做一个重力对齐
}

Eigen::Vector3d Estimator::toEuler(const Eigen::Matrix3d &R)
{
//    assert(isRotationMatrix(R));
    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }

    Eigen::Vector3d v_euler;
    v_euler.x() = x;
    v_euler.y() = y;
    v_euler.z() = z;

    return v_euler;
}
