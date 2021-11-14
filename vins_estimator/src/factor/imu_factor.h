/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class IMUFactor : public ceres::SizedCostFunction<18, 7, 9, 7, 9>
{
  public:
    IMUFactor() = delete;
    IMUFactor(IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

        Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

//Eigen::Matrix<double, 15, 15> Fd;
//Eigen::Matrix<double, 15, 12> Gd;

//Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
//Eigen::Quaterniond pQj = Qi * delta_q;
//Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
//Eigen::Vector3d pBaj = Bai;
//Eigen::Vector3d pBgj = Bgi;

//Vi + Qi * delta_v - g * sum_dt = Vj;
//Qi * delta_q = Qj;

//delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
//delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
//delta_q = Qi.inverse() * Qj;

#if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

//        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
//        residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
//                                            Pj, Qj, Vj, Baj, Bgj);
        Eigen::Map<Eigen::Matrix<double, 18, 1>> residual(residuals);
        residual = pre_integration->evaluate_wheel(Pi, Qi, Vi, Bai, Bgi,
                                             Pj, Qj, Vj, Baj, Bgj);

//        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        Eigen::Matrix<double, 18, 18> sqrt_info_wheel=Eigen::LLT<Eigen::Matrix<double, 18, 18>>(pre_integration->covariance.inverse()).matrixL().transpose();;
        Eigen::Matrix<double, 18, 18> sqrt_info_wheel_test=Eigen::LLT<Eigen::Matrix<double, 18, 18>>(pre_integration->covariance.inverse()).matrixL().transpose();;
//        std::cout<<"pre_integration->covariance:\n"<<setprecision(6)<<pre_integration->covariance<<endl;
//        std::cout<<"pre_integration->jacobian:\n"<<setprecision(6)<<pre_integration->jacobian<<endl;
//        std::cout<<"pre_integration->covariance.inverse():\n"<<setprecision(6)<<pre_integration->covariance.inverse()<<std::endl;
        std::cout<<"sqrt_info_wheel origin :\n"<<setprecision(6)<<sqrt_info_wheel<<endl;
//        sqrt_info_wheel.setIdentity();
//        sqrt_info_wheel.matrix().block<15,15>(0,0)=sqrt_info;
//        residual = sqrt_info * residual;
        Eigen::Matrix<double, 18, 18> sqrt_info;
        sqrt_info.setZero();
        sqrt_info.matrix().block<15,15>(0,0)=sqrt_info_wheel.matrix().block<15,15>(0,0);
        sqrt_info_wheel=sqrt_info;
        std::cout<<"sqrt_info_wheel short :\n"<<setprecision(6)<<sqrt_info_wheel<<endl;
        std::cout<<"residual raw:\t"<<setprecision(6)<<residual.transpose()<<endl;
        residual = sqrt_info_wheel * residual;
        std::cout<<"residual:\t"<<setprecision(6)<<residual.transpose()<<endl;
        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);

            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration");
                //std::cout << pre_integration->jacobian << std::endl;
///                ROS_BREAK();
            }

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);//将c数组转成eigen矩阵，复用了数组里的内存空间
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
#endif

                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i.block<3,3>(O_P_Vel,O_P) = -Qi.inverse().toRotationMatrix();// 轮速计
                jacobian_pose_i.block<3, 3>(O_P_Vel, O_R) = Utility::skewSymmetric(Qi.inverse() * ( Pj - Pi )) + Utility::skewSymmetric(Qi.inverse() * Qj * TIV[0]);

                std::cout<<"jacobian_pose_i\n"<<jacobian_pose_i<<std::endl;
                std::cout<<"jacobian_pose_i after sqrt info long\n"<<sqrt_info_wheel_test*jacobian_pose_i<<std::endl;
                jacobian_pose_i = sqrt_info_wheel * jacobian_pose_i;
                std::cout<<"jacobian_pose_i after sqrt info\n"<<jacobian_pose_i<<std::endl;
                jacobian_pose_i.matrix().block<3,7>(O_P_Vel,0)= (sqrt_info_wheel_test*jacobian_pose_i).matrix().block<3,7>(O_P_Vel,0);
                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in preintegration");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                //jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
#endif

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

//                jacobian_speedbias_i.block<3, 3>(O_P_Vel, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;//轮速计
                std::cout<<"jacobian_speedbias_i\n"<<jacobian_speedbias_i<<std::endl;
                std::cout<<"jacobian_speedbias_i after sqrt info long\n"<<sqrt_info_wheel_test*jacobian_speedbias_i<<std::endl;
                jacobian_speedbias_i = sqrt_info_wheel * jacobian_speedbias_i;
                std::cout<<"jacobian_pose_i after sqrt info\n"<<jacobian_speedbias_i<<std::endl;

                //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif
                jacobian_pose_j.block<3, 3>(O_P_Vel, O_P) = Qi.inverse().toRotationMatrix();//轮速计
                jacobian_pose_j.block<3, 3>(O_P_Vel, O_R) = -Qi.inverse().toRotationMatrix() * Utility::skewSymmetric(Qj * TIV[0]);//轮速计

                std::cout<<"jacobian_speedbias_i\n"<<jacobian_pose_j<<std::endl;
                std::cout<<"jacobian_speedbias_i after sqrt info long\n"<<sqrt_info_wheel_test*jacobian_pose_j<<std::endl;
                jacobian_pose_j = sqrt_info_wheel * jacobian_pose_j;
                std::cout<<"jacobian_pose_i after sqrt info\n"<<jacobian_pose_j<<std::endl;
                jacobian_pose_j.matrix().block<3,7>(O_P_Vel,0)= (sqrt_info_wheel_test*jacobian_pose_j).matrix().block<3,7>(O_P_Vel,0);
                //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                std::cout<<"jacobian_speedbias_i\n"<<jacobian_speedbias_j<<std::endl;
                std::cout<<"jacobian_speedbias_i after sqrt info long\n"<<sqrt_info_wheel_test*jacobian_speedbias_j<<std::endl;
                jacobian_speedbias_j = sqrt_info_wheel * jacobian_speedbias_j;
                std::cout<<"jacobian_pose_i after sqrt info\n"<<jacobian_speedbias_j<<std::endl;

                //ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    IntegrationBase* pre_integration;

};

