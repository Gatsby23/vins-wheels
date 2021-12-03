/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include "../utility/utility.h"
#include "../estimator/parameters.h"

#include <ceres/ceres.h>
using namespace Eigen;

class IntegrationBase
{
  public:
    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
        : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
          linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
          jacobian{Eigen::Matrix<double, 18, 18>::Identity()}, covariance{Eigen::Matrix<double, 18, 18>::Zero()},
          jacobian_origin{Eigen::Matrix<double, 15, 15>::Identity()}, covariance_origin{Eigen::Matrix<double, 15, 15>::Zero()},
          sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()},
          delta_p_i_vel{Eigen::Vector3d::Zero()}

    {
        noise = Eigen::Matrix<double, 21, 21>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(18, 18) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();//轮速

        noise_origin =  Eigen::Matrix<double, 18, 18>::Zero();
        noise_origin.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_vel_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
            : acc_0{_acc_0}, gyr_0{_gyr_0}, vel_0{_vel_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
              linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
              jacobian{Eigen::Matrix<double, 18, 18>::Identity()}, covariance{Eigen::Matrix<double, 18, 18>::Zero()},
              jacobian_origin{Eigen::Matrix<double, 15, 15>::Identity()}, covariance_origin{Eigen::Matrix<double, 15, 15>::Zero()},
              jacobian_enc{Eigen::Matrix<double, 18, 18>::Identity()}, covariance_enc{Eigen::Matrix<double, 18, 18>::Zero()},
              sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()},
              delta_p_i_vel{Eigen::Vector3d::Zero()}

    {
        noise = Eigen::Matrix<double, 21, 21>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(18, 18) =  (ENC_N * ENC_N) * Eigen::Matrix3d::Identity();//轮速

        noise_origin =  Eigen::Matrix<double, 18, 18>::Zero();
        noise_origin.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise_origin.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();

        noise_enc = Eigen::Matrix<double, 24, 24>::Zero();
        noise_enc.block<3, 3>(0, 0) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise_enc.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise_enc.block<3, 3>(6, 6) = (ENC_N * ENC_N) * Eigen::Matrix3d::Identity();
        noise_enc.block<3, 3>(9, 9) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise_enc.block<3, 3>(12, 12) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise_enc.block<3, 3>(15, 15) = (ENC_N * ENC_N) * Eigen::Matrix3d::Identity();
        noise_enc.block<3, 3>(18, 18) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise_enc.block<3, 3>(21, 21) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();

    }

    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        //cout<<"acc_buf.size()  "<<acc_buf.size()<<endl;
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);
        propagate(dt, acc, gyr);
    }

    void push_back_wheels(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const Eigen::Vector3d &vel)
    {
        //cout<<"acc_buf.size()  "<<acc_buf.size()<<endl;
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);
        vel_buf.push_back(vel);
        propagate_wheel(dt, acc, gyr,vel);
    }

    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        delta_p_i_vel.setZero();
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        jacobian.setIdentity();
        covariance.setZero();
//        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
//            propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
        std::cout<<"dt_buf.size() "<<dt_buf.size()<<std::endl;
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate_wheel(dt_buf[i], acc_buf[i], gyr_buf[i],vel_buf[i]);
    }

    void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
//        ROS_INFO("midpoint integration");
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;         

        if(update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;

            R_w_x<<0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
            R_a_0_x<<0, -a_0_x(2), a_0_x(1),
                a_0_x(2), 0, -a_0_x(0),
                -a_0_x(1), a_0_x(0), 0;
            R_a_1_x<<0, -a_1_x(2), a_1_x(1),
                a_1_x(2), 0, -a_1_x(0),
                -a_1_x(1), a_1_x(0), 0;

            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt + 
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //cout<<"A"<<endl<<A<<endl;

            MatrixXd V = MatrixXd::Zero(15,18);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

            //step_jacobian = F;
            //step_V = V;
            jacobian = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }

    }

    void midPointIntegration_wheel(double _dt,
                             const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian,
                             const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_vel_1,
                             const Eigen::Vector3d &delta_p_vel, Eigen::Vector3d &result_delta_p_vel//轮速计
                             )
    {
//        ROS_INFO("midpoint integration");
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Vector3d un_vel_0 = delta_q * RIV[0] * (_vel_0);//轮速
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Vector3d un_vel_1 = result_delta_q * RIV[0]* (_vel_1);//轮速
        Vector3d un_vel = 0.5 * (un_vel_0 + un_vel_1);

        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        result_delta_p_vel = delta_p_vel + un_vel * _dt;//轮速计
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;

        if(update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;

            Vector3d vel_0_x,vel_1_x;
            vel_0_x = RIV[0]*_vel_0;
            vel_1_x = RIV[0]*_vel_1;
            Matrix3d R_vel_0_x,R_vel_1_x;

            R_w_x<<0, -w_x(2), w_x(1),
                    w_x(2), 0, -w_x(0),
                    -w_x(1), w_x(0), 0;
            R_a_0_x<<0, -a_0_x(2), a_0_x(1),
                    a_0_x(2), 0, -a_0_x(0),
                    -a_0_x(1), a_0_x(0), 0;
            R_a_1_x<<0, -a_1_x(2), a_1_x(1),
                    a_1_x(2), 0, -a_1_x(0),
                    -a_1_x(1), a_1_x(0), 0;
            R_vel_0_x<< 0,                  -vel_0_x(2),     vel_0_x(1),
                        vel_0_x(2),          0,               -vel_0_x(0),
                        -vel_0_x(1),     vel_0_x(0),          0;
            R_vel_1_x<< 0,                   -vel_1_x(2),       vel_1_x(1),
                        vel_1_x(2),          0,                 -vel_1_x(0),
                        -vel_1_x(1),     vel_1_x(0),             0;


            MatrixXd F = MatrixXd::Zero(18, 18);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //vel
            F.block<3, 3>(15, 3) = (-0.5 * delta_q.toRotationMatrix() * R_vel_0_x * _dt +
                                   -0.5 * result_delta_q.toRotationMatrix() * R_vel_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt);  //对角度
            F.block<3, 3>(15, 12) = (-0.5 * result_delta_q.toRotationMatrix() * R_vel_1_x * _dt);//对角度bias
            F.block<3, 3>(15, 15) = Matrix3d::Identity();//对轮速计偏差

            MatrixXd F_origin = F.matrix().block<15,15>(0,0);
            //cout<<"A"<<endl<<A<<endl;

            MatrixXd V = MatrixXd::Zero(18,21);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

            //轮式计
            V.block<3, 3>(15, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_vel_1_x * _dt * 0.5 * _dt;
            V.block<3, 3>(15, 9) = V.block<3, 3>(15, 3);
            V.block<3, 3>(15, 18) = 0.5 * (delta_q.toRotationMatrix()*RIV[0]+result_delta_q.toRotationMatrix()*RIV[0]) * _dt;

            MatrixXd V_origin = V.matrix().block<15,18>(0,0);
            //step_jacobian = F;
            //step_V = V;
            jacobian = F * jacobian;
            jacobian_origin = F_origin * jacobian_origin;

            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
            covariance_origin = F_origin * covariance_origin * F_origin.transpose() + V_origin * noise_origin * V_origin.transpose();

        }

    }
//vins-gps-wheels
    void midPointIntegration_encode(double _dt,
                                   const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                                   const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                                   const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                                   const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                                   Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                                   Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian,
                                   const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_vel_1,
                                   const Eigen::Vector3d &delta_p_vel, Eigen::Vector3d &result_delta_p_vel//轮速计
    )
    {
//        ROS_INFO("midpoint integration");
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Vector3d un_vel_0 = delta_q * RIV[0] * (_vel_0);//轮速
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Vector3d un_vel_1 = result_delta_q * RIV[0]* (_vel_1);//轮速
        Vector3d un_vel = 0.5 * (un_vel_0 + un_vel_1);

        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        result_delta_p_vel = delta_p_vel + un_vel * _dt;//轮速计
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;

        if(update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Vector3d e_0_x = RIV[0] * _vel_0;
            Vector3d e_1_x = RIV[0] * _vel_0;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x, R_e_0_x, R_e_1_x;

            R_w_x<<0, -w_x(2), w_x(1),
                    w_x(2), 0, -w_x(0),
                    -w_x(1), w_x(0), 0;
            R_a_0_x<<0, -a_0_x(2), a_0_x(1),
                    a_0_x(2), 0, -a_0_x(0),
                    -a_0_x(1), a_0_x(0), 0;
            R_a_1_x<<0, -a_1_x(2), a_1_x(1),
                    a_1_x(2), 0, -a_1_x(0),
                    -a_1_x(1), a_1_x(0), 0;
            R_e_0_x << 0, -e_0_x(2), e_0_x(1),
                    e_0_x(2), 0, -e_0_x(0),
                    -e_0_x(1), e_0_x(0), 0;
            R_e_1_x << 0, -e_1_x(2), e_1_x(1),
                    e_1_x(2), 0, -e_1_x(0),
                    -e_1_x(1), e_1_x(0), 0;

            MatrixXd F = MatrixXd::Zero(18, 18);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 12) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 15) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 15) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 12) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 15) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;

            F.block<3, 3>(9, 3) = -0.5 * delta_q.toRotationMatrix() * R_e_0_x * _dt +
                                  -0.5 * result_delta_q.toRotationMatrix() * R_e_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(9, 15) = 0.5 * result_delta_q.toRotationMatrix() * R_e_1_x * _dt * _dt;


            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            F.block<3, 3>(15, 15) = Matrix3d::Identity();

            MatrixXd V = MatrixXd::Zero(18, 24);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 9) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 12) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 12) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 9) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 12) =  V.block<3, 3>(6, 3);

            V.block<3, 3>(9, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_e_1_x * _dt * _dt; // 相差-
            V.block<3, 3>(9, 6) = 0.5 * delta_q.toRotationMatrix() * RIV[0] * _dt;
            V.block<3, 3>(9, 12) = V.block<3, 3>(9, 3);
            V.block<3, 3>(9, 15) = 0.5 * result_delta_q.toRotationMatrix() * RIV[0] * _dt;

            V.block<3, 3>(12, 18) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(15, 21) = MatrixXd::Identity(3,3) * _dt;

            // step_jacobian_enc = F;
            // step_V_enc = V;

            jacobian_enc = F * jacobian_enc;
            covariance_enc = F * covariance_enc * F.transpose() + V * noise_enc * V.transpose();
        }

    }


    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;

        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);

        //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
        //                    linearized_ba, linearized_bg);
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;  
     
    }

    void propagate_wheel(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1 ,const Eigen::Vector3d &_vel_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        vel_1 = _vel_1;
        Vector3d result_delta_p;
        Vector3d result_delta_p_vel;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;
//        std::cout<<"midPointIntegration_wheel after  delta_p_i_vel:"<<setprecision(8)
//        <<delta_p_i_vel.transpose()<<"\tdelta_q: "<<delta_q.coeffs().transpose()<<
//                 " delta_p: "<<delta_p.transpose()
//                 <<"\n linearized_ba "<<linearized_ba.transpose()<<"\tlinearized_bg "<<linearized_bg.transpose()<<endl;
//        std::cout<<"vel_0 "<<vel_0.transpose()<<"\tvel_1 "<<vel_1.transpose()<<std::endl;
//    midPointIntegration_encode   midPointIntegration_wheel
        midPointIntegration_wheel(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                                  linearized_ba, linearized_bg,
                                  result_delta_p, result_delta_q, result_delta_v,
                                  result_linearized_ba, result_linearized_bg, 1,
                                  vel_0, vel_1, delta_p_i_vel, result_delta_p_vel);

        //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
        //                    linearized_ba, linearized_bg);
        delta_p = result_delta_p;
        delta_p_i_vel = result_delta_p_vel;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;
        vel_0 = vel_1;
//        std::cout<<"after vel_0 "<<vel_0.transpose()<<"\tvel_1 "<<vel_1.transpose()<<std::endl;
    }

    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }

    Eigen::Matrix<double, 18, 1> evaluate_wheel(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 18, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Matrix3d dyita_dbg = jacobian.block<3, 3>(O_P_Vel, O_BG);

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
        Eigen::Vector3d corrected_delta_p_i_vel = delta_p_i_vel + dyita_dbg * dbg;

        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
//        residuals.block<3, 1>(O_P_Vel,0)=Qi.inverse() * (Pj - Pi)  - TIV[0] + (Qi.inverse()*Qj) * TIV[0] - corrected_delta_p_i_vel;
        residuals.block<3, 1>(O_P_Vel,0)=(Qi.inverse() * ((Pj + Qj * TIV[0]) - (Pi + Qi * TIV[0])) - corrected_delta_p_i_vel);

//        residuals.block<3, 1>(O_P_Vel,0)=Eigen::Vector3d::Zero();
        Eigen::Vector3d temp_ = -TIV[0] + corrected_delta_q * TIV[0];
        Eigen::Vector3d temp2_ = Qi.inverse() * (Pj - Pi);
        Eigen::Vector3d temp3_ = Qi.inverse() * (Pj - Pi) - TIV[0] + (Qi.inverse()*Qj) * TIV[0];
        std::cout<<"corrected_delta_p_i_vel: "<<setprecision(10)<<corrected_delta_p_i_vel.transpose();
        std::cout<<"\t\t-TIV[0] + corrected_delta_q * TIV[0]: "<<setprecision(10)<<temp_.transpose()<<std::endl;
        std::cout<<"delta_p_i_vel:***********\t"<<setprecision(10)<<delta_p_i_vel.transpose()<<"\tsum_dt: "<<sum_dt<<std::endl;
        std::cout<<"delta(P)-TIV :***********\t"<<setprecision(10)<<temp2_.transpose();
        std::cout<<"\t\tdelta(P):   "<<setprecision(10)<<temp3_.transpose()<<"\nVi:"<<Vi.norm()<<"\tVj:"<<Vj.norm()<<std::endl;
        std::cout<<"Bgi: "<<Bgi.transpose()<<"\tBai:"<<Bai.transpose()<<std::endl;

        return residuals;
    }
//vins-gps-wheels
    Eigen::Matrix<double, 18, 1> evaluate_encode(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Pi_io, const Eigen::Quaterniond &Qi_io,
                                          const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Pj_io, const Eigen::Quaterniond &Qj_io,
                                          const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 18, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian_enc.block<3, 3>(0, 12);
        Eigen::Matrix3d dp_dbg = jacobian_enc.block<3, 3>(0, 15);

        Eigen::Matrix3d dq_dbg = jacobian_enc.block<3, 3>(3, 15);

        Eigen::Matrix3d dv_dba = jacobian_enc.block<3, 3>(6, 12);
        Eigen::Matrix3d dv_dbg = jacobian_enc.block<3, 3>(6, 15);

        Eigen::Matrix3d do_dbg = jacobian_enc.block<3, 3>(9, 15);

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
        Eigen::Vector3d corrected_delta_eta = delta_p_i_vel + do_dbg * dbg;
        // ROS_INFO_STREAM("do_dbg " << do_dbg);
        // ROS_INFO_STREAM("delta_eta " << delta_eta.transpose());
        // ROS_INFO_STREAM("Corrected_delta_eta " << corrected_delta_eta.transpose());


        residuals.block<3, 1>(0, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(3, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(6, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;

        // residuals.block<3, 1>(9, 0) = Qi.inverse() * (Pj - Pi) - TIO + Qi.inverse() * Qj * TIO - corrected_delta_eta;
        residuals.block<3, 1>(9, 0) = (Qi.inverse() * ((Pj + Qj * TIV[0]) - (Pi + Qi * TIV[0])) - corrected_delta_eta);

        residuals.block<3, 1>(12, 0) = Baj - Bai;
        residuals.block<3, 1>(15, 0) = Bgj - Bgi;
        return residuals;
    }

    Eigen::Matrix<double, 3, 1> evaluate_only_wheel(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi,
                                                const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj)
    {
        Eigen::Matrix<double, 3, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Matrix3d dyita_dbg = jacobian.block<3, 3>(O_P_Vel, O_BG);

        residuals.block<3, 1>(0,0)=Qi.inverse() * (Pj - Pi)  - delta_p_i_vel;//- TIV[0] + (Qi.inverse()*Qj) * TIV[0]
//        residuals.block<3, 1>(O_P_Vel,0)=Eigen::Vector3d::Zero();
        Eigen::Vector3d temp_ = -TIV[0] + (Qi.inverse()*Qj) * TIV[0];
        Eigen::Vector3d temp2_ = Qi.inverse() * (Pj - Pi);
        Eigen::Vector3d temp3_ = Qi.inverse() * (Pj - Pi) - TIV[0] + (Qi.inverse()*Qj) * TIV[0];
        std::cout<<"delta_p_i_vel:                          "<<delta_p_i_vel.transpose();
        std::cout<<"\t\t-TIV[0] + corrected_delta_q * TIV[0]:   "<<temp_.transpose()<<std::endl;
        std::cout<<"Qi.inverse() * (Pj - Pi):               "<<temp2_.transpose();
        std::cout<<"\t\tdelta(P):                              "<<temp3_.transpose()<<std::endl;

        return residuals;
    }

    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;
    Eigen::Vector3d vel_0, vel_1;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 18, 18> jacobian, covariance;
    Eigen::Matrix<double, 18, 18> jacobian_enc, covariance_enc;
    Eigen::Matrix<double, 15, 15> jacobian_origin, covariance_origin;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 21, 21> noise;
    Eigen::Matrix<double, 18, 18> noise_origin;
    Eigen::Matrix<double, 24, 24> noise_enc;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Vector3d delta_p_i_vel;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
    std::vector<Eigen::Vector3d> vel_buf;

};
/*

    void eulerIntegration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        result_delta_p = delta_p + delta_v * _dt + 0.5 * (delta_q * (_acc_1 - linearized_ba)) * _dt * _dt;
        result_delta_v = delta_v + delta_q * (_acc_1 - linearized_ba) * _dt;
        Vector3d omg = _gyr_1 - linearized_bg;
        omg = omg * _dt / 2;
        Quaterniond dR(1, omg(0), omg(1), omg(2));
        result_delta_q = (delta_q * dR);   
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;         

        if(update_jacobian)
        {
            Vector3d w_x = _gyr_1 - linearized_bg;
            Vector3d a_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_x;

            R_w_x<<0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
            R_a_x<<0, -a_x(2), a_x(1),
                a_x(2), 0, -a_x(0),
                -a_x(1), a_x(0), 0;

            MatrixXd A = MatrixXd::Zero(15, 15);
            // one step euler 0.5
            A.block<3, 3>(0, 3) = 0.5 * (-1 * delta_q.toRotationMatrix()) * R_a_x * _dt;
            A.block<3, 3>(0, 6) = MatrixXd::Identity(3,3);
            A.block<3, 3>(0, 9) = 0.5 * (-1 * delta_q.toRotationMatrix()) * _dt;
            A.block<3, 3>(3, 3) = -R_w_x;
            A.block<3, 3>(3, 12) = -1 * MatrixXd::Identity(3,3);
            A.block<3, 3>(6, 3) = (-1 * delta_q.toRotationMatrix()) * R_a_x;
            A.block<3, 3>(6, 9) = (-1 * delta_q.toRotationMatrix());
            //cout<<"A"<<endl<<A<<endl;

            MatrixXd U = MatrixXd::Zero(15,12);
            U.block<3, 3>(0, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            U.block<3, 3>(3, 3) =  MatrixXd::Identity(3,3);
            U.block<3, 3>(6, 0) =  delta_q.toRotationMatrix();
            U.block<3, 3>(9, 6) = MatrixXd::Identity(3,3);
            U.block<3, 3>(12, 9) = MatrixXd::Identity(3,3);

            // put outside
            Eigen::Matrix<double, 12, 12> noise = Eigen::Matrix<double, 12, 12>::Zero();
            noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
            noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
            noise.block<3, 3>(6, 6) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
            noise.block<3, 3>(9, 9) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();

            //write F directly
            MatrixXd F, V;
            F = (MatrixXd::Identity(15,15) + _dt * A);
            V = _dt * U;
            step_jacobian = F;
            step_V = V;
            jacobian = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }

    }     


    void checkJacobian(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, 
                                   const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg)
    {
        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;
        midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 0);

        Vector3d turb_delta_p;
        Quaterniond turb_delta_q;
        Vector3d turb_delta_v;
        Vector3d turb_linearized_ba;
        Vector3d turb_linearized_bg;

        Vector3d turb(0.0001, -0.003, 0.003);

        midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p + turb, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb p       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 0) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 0) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 0) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 0) * turb).transpose() << endl;
        cout << "bg diff " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff " << (step_jacobian.block<3, 3>(12, 0) * turb).transpose() << endl;

        midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q * Quaterniond(1, turb(0) / 2, turb(1) / 2, turb(2) / 2), delta_v,
                            linearized_ba, linearized_bg,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb q       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 3) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 3) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 3) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 3) * turb).transpose() << endl;
        cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff" << (step_jacobian.block<3, 3>(12, 3) * turb).transpose() << endl;

        midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v + turb,
                            linearized_ba, linearized_bg,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb v       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 6) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 6) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 6) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 6) * turb).transpose() << endl;
        cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff" << (step_jacobian.block<3, 3>(12, 6) * turb).transpose() << endl;

        midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba + turb, linearized_bg,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb ba       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 9) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 9) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 9) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 9) * turb).transpose() << endl;
        cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff" << (step_jacobian.block<3, 3>(12, 9) * turb).transpose() << endl;

        midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg + turb,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb bg       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_jacobian.block<3, 3>(0, 12) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_jacobian.block<3, 3>(3, 12) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_jacobian.block<3, 3>(6, 12) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_jacobian.block<3, 3>(9, 12) * turb).transpose() << endl;
        cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff" << (step_jacobian.block<3, 3>(12, 12) * turb).transpose() << endl;

        midPointIntegration(_dt, _acc_0 + turb, _gyr_0, _acc_1 , _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb acc_0       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_V.block<3, 3>(0, 0) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_V.block<3, 3>(3, 0) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_V.block<3, 3>(6, 0) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_V.block<3, 3>(9, 0) * turb).transpose() << endl;
        cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff" << (step_V.block<3, 3>(12, 0) * turb).transpose() << endl;

        midPointIntegration(_dt, _acc_0, _gyr_0 + turb, _acc_1 , _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb _gyr_0       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_V.block<3, 3>(0, 3) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_V.block<3, 3>(3, 3) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_V.block<3, 3>(6, 3) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_V.block<3, 3>(9, 3) * turb).transpose() << endl;
        cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff" << (step_V.block<3, 3>(12, 3) * turb).transpose() << endl;

        midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1 + turb, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb acc_1       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_V.block<3, 3>(0, 6) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_V.block<3, 3>(3, 6) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_V.block<3, 3>(6, 6) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_V.block<3, 3>(9, 6) * turb).transpose() << endl;
        cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff" << (step_V.block<3, 3>(12, 6) * turb).transpose() << endl;

        midPointIntegration(_dt, _acc_0, _gyr_0, _acc_1 , _gyr_1 + turb, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            turb_delta_p, turb_delta_q, turb_delta_v,
                            turb_linearized_ba, turb_linearized_bg, 0);
        cout << "turb _gyr_1       " << endl;
        cout << "p diff       " << (turb_delta_p - result_delta_p).transpose() << endl;
        cout << "p jacob diff " << (step_V.block<3, 3>(0, 9) * turb).transpose() << endl;
        cout << "q diff       " << ((result_delta_q.inverse() * turb_delta_q).vec() * 2).transpose() << endl;
        cout << "q jacob diff " << (step_V.block<3, 3>(3, 9) * turb).transpose() << endl;
        cout << "v diff       " << (turb_delta_v - result_delta_v).transpose() << endl;
        cout << "v jacob diff " << (step_V.block<3, 3>(6, 9) * turb).transpose() << endl;
        cout << "ba diff      " << (turb_linearized_ba - result_linearized_ba).transpose() << endl;
        cout << "ba jacob diff" << (step_V.block<3, 3>(9, 9) * turb).transpose() << endl;
        cout << "bg diff      " << (turb_linearized_bg - result_linearized_bg).transpose() << endl;
        cout << "bg jacob diff" << (step_V.block<3, 3>(12, 9) * turb).transpose() << endl;
    }
    */