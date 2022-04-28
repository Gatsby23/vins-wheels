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

#include "initial_alignment.h"

void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);//i-j之间的旋转
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    delta_bg = A.ldlt().solve(b);
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);//陀螺仪bias变化，重新积分
    }
}


MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    Vector3d g0 = g.normalized() * G.norm();
    Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int k = 0; k < 4; k++)
    {
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();

            double dt = frame_j->second.pre_integration->sum_dt;


            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0] - frame_i->second.R.transpose() * dt * dt / 2 * g0;

            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;


            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();

            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();

            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
            A = A * 1000.0;
            b = b * 1000.0;
            x = A.ldlt().solve(b);
            VectorXd dg = x.segment<2>(n_state - 3);
            g0 = (g0 + lxly * dg).normalized() * G.norm();
            double s = x(n_state - 1);
            std::cout<<"RefineGravity s="<<s<<std::endl;
    }   
    g = g0;
}

void RefineGravityWithS(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x,double &s_)
{
    Vector3d g0 = g.normalized() * G.norm();
    Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 ;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();
    VectorXd x_without_s;

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int k = 0; k < 4; k++)
    {
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);//水平方向重力分力
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            MatrixXd tmp_A(6, 8);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();

            double dt = frame_j->second.pre_integration->sum_dt;


            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
//            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0]
                                        - frame_i->second.R.transpose() * dt * dt / 2 * g0
                                        - s_*(frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T));

            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;


            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();

            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            A.bottomRightCorner<3, 2>() += r_A.bottomRightCorner<3, 2>();
            b.tail<2>() += r_b.tail<2>();

            A.block<6, 2>(i * 3, n_state - 3) += r_A.topRightCorner<6, 2>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        A = A * 1000.0;
        b = b * 1000.0;
        x_without_s = A.ldlt().solve(b);
        VectorXd dg = x_without_s.segment<2>(n_state - 2);
        g0 = (g0 + lxly * dg).normalized() * G.norm();
        //double s = x(n_state - 1);
    }
    g = g0;
    n_state=n_state+1;
    x = Eigen::VectorXd(n_state);
    x.segment(0,n_state-1)=x_without_s;
    x(n_state - 1)=s_;
}
//线性对准
bool LinearAlignment(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 3 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;
        std::cout<<"sum dt= "<<dt <<"\t\ttime="<<frame_i->first<<std::endl;
        //
        // A是论文中的H矩阵 b是论文中的观测矩阵z
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;  //为什么要除以一百
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
        //cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();//左上方的六个
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();//右下方
        b.tail<4>() += r_b.tail<4>();

        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
//        cout<<setprecision(2)<<"A=\n"<<A<<endl;
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);//LDLT求最小二乘解  其实就是处理AT*A*x=AT*b问题
    cout<<setprecision(2)<<"Ax=\n"<<A*x<<endl;
    cout<<"b= \n"<<b<<endl;
    cout<<"x=\n"<<x<<endl;
    double s = x(n_state - 1) / 100.0;
    cout<<"s="<<s<<endl;
    ROS_DEBUG("estimated scale: %f", s);
    g = x.segment<3>(n_state - 4);
    cout<<"g before RefineGravity ="<<g<<endl;
    ROS_DEBUG_STREAM(" result g     " << g.norm() << " " << g.transpose());
    if(fabs(g.norm() - G.norm()) > 0.5 || s < 0)
    {
        return false;
    }

    RefineGravity(all_image_frame, g, x);//重力优化
    Matrix3d R_g = Utility::g2R(g);//计算夹角
    Eigen::Vector3d EulerRg = R_g.eulerAngles(2,1,0);
    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    cout<<"g after RefineGravity ="<<g<<endl;//相机坐标系下的g
    cout<< "EulerRg= "<<(180.f/M_PI*EulerRg).transpose()<<endl;

    ROS_DEBUG_STREAM(" refine     " << g.norm() << " " << g.transpose());
    if(s < 0.0 )
        return false;
    else
        return true;
}

bool LinearAlignmentWithS(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x, double &s_)
{
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 3;// + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();
    VectorXd x_without_s;

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 9);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;
        //
        // A是论文中的H矩阵 b是论文中的观测矩阵z
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
//        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100;
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0]
                                                - s_*(frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T));
        //cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();//左上方的六个
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<4, 3>() += r_A.bottomRightCorner<4, 3>();//右下方
        b.tail<3>() += r_b.tail<3>();

        A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();//最右面三列
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();//最下面四行
//        cout<<setprecision(2)<<"A=\n"<<A<<endl;
        Eigen::Vector3d T_sfm = (s_*(frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T)));
        std::cout<<"T sfm\t"<<T_sfm.transpose()<<std::endl;
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x_without_s = A.ldlt().solve(b);//LDLT求最小二乘解  其实就是处理AT*A*x=AT*b问题
    n_state=n_state+1;
    x = Eigen::VectorXd(n_state);
    x.segment(0,n_state-1)=x_without_s;
    x(n_state - 1)=s_;
//    cout<<setprecision(2)<<"A=\n"<<A<<endl;
//    cout<<"b= \n"<<b<<endl;
    cout<<"x=\n"<<x<<endl;
    int kv = -1;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
            kv++;
            Eigen::Vector3d V_ = frame_i->second.R * x.segment<3>(kv * 3);
            std::cout<<"time= "<<setprecision(17)<<frame_i->first<<"\tVs[kv]="<<V_.transpose()<<"\t x="<<x.segment<3>(kv * 3).transpose()<<std::endl;
    }
    double s = x(n_state - 1) / 100.0;
    cout<<"s="<<s<<endl;
    ROS_DEBUG("estimated scale: %f", s);
    g = x.segment<3>(n_state - 4);
    cout<<"g before RefineGravity =\n"<<g<<"\t g norm="<<g.norm()<<endl;
    ROS_DEBUG_STREAM(" result g     " << g.norm() << " " << g.transpose());
    if(fabs(g.norm() - G.norm()) > 0.5 || s < 0)
    {
        return false;
    }

//    RefineGravity(all_image_frame, g, x);//重力优化
    RefineGravityWithS(all_image_frame, g, x,s_);//重力优化
    kv = -1;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
            kv++;
            Eigen::Vector3d V_ = frame_i->second.R * x.segment<3>(kv * 3);
            std::cout<<"RefineGravityWithS time= "<<setprecision(17)<<frame_i->first<<"\tVs[kv]="<<V_.transpose()<<"\t x="<<x.segment<3>(kv * 3).transpose()<<std::endl;
    }

//    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s_;
    Matrix3d R_g = Utility::g2R(g);//计算夹角
    Eigen::Vector3d EulerRg = R_g.eulerAngles(2,1,0);
    cout<< "EulerRg= "<<(180.f/M_PI*EulerRg).transpose()<<endl;
    cout<<"g after RefineGravity ="<<g<<"\t g norm="<<g.norm()<<endl;//相机坐标系下的g
    ROS_DEBUG_STREAM(" refine     " << g.norm() << " " << g.transpose());
    if(s < 0.0 )
        return false;
    else
        return true;
}

bool solve_s(map<double, ImageFrame> &all_image_frame , double &s)
{
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count;
    VectorXd A{n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);
        Eigen::Matrix4d Hi=Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Hj=Eigen::Matrix4d::Identity();
        Hi.matrix().block<3,3>(0,0)=frame_i->second.R;
        Hi.matrix().block<3,1>(0,3)=frame_i->second.T;
        Hj.matrix().block<3,3>(0,0)=frame_j->second.R;
        Hj.matrix().block<3,1>(0,3)=frame_j->second.T;
        Eigen::Matrix4d Hij=Hi.inverse()*Hj;
        double temp_A=Hij.matrix().block<3,1>(0,3).norm();
        Eigen::Vector3d T_imu=frame_j->second.pre_integration->delta_p_i_vel + TIV[0] - frame_j->second.pre_integration->delta_q * TIV[0];
        double temp_b=(T_imu + frame_j->second.pre_integration->delta_q*TIC[0]-TIC[0]).norm();
        A(i)=temp_A;
        b(i)=temp_b;
    }
    std::cout<<"solved A\n"<<A<<std::endl;
    std::cout<<"solved b\n"<<b<<std::endl;
    VectorXd s_ = (A.transpose()*A).inverse()*(A.transpose()*b);
    s=s_.norm();
    std::cout<<"s solved="<<s<<std::endl;
    return true;
}
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    solveGyroscopeBias(all_image_frame, Bgs);//陀螺仪bias矫正
    double s;//尺度
//    solve_s(all_image_frame,s);//计算尺度
//    if(LinearAlignmentWithS(all_image_frame, g, x, s))////线性对准  LinearAlignment(all_image_frame, g, x)  LinearAlignmentWithS(all_image_frame, g, x, s)
    if(LinearAlignment(all_image_frame, g, x))
        return true;
    else 
        return false;
}
