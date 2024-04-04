/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS,"
without warranty of any kind, including without limitation the warranties
of merchantability, fitness for a particular purpose and non-infringement.
KUKA makes no warranty that the Software is free of defects or is suitable
for any particular purpose. In no event shall KUKA be responsible for loss
or damages arising from the installation or use of the Software,
including but not limited to any indirect, punitive, special, incidental
or consequential damages of any character including, without limitation,
damages for loss of goodwill, work stoppage, computer failure or malfunction,
or any and all other commercial damages or losses.
The entire risk to the quality and performance of the Software is not borne by KUKA.
Should the Software prove defective, KUKA is not liable for the entire cost
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2015
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned
to KUKA Roboter GmbH immediately upon request.
This material and the information illustrated or contained herein may not be used,
reproduced, stored in a retrieval system, or transmitted in whole
or in part in any way - electronic, mechanical, photocopying, recording,
or otherwise, without the prior written consent of KUKA Roboter GmbH.



\file
\version {1.9}
*/
#ifndef _KUKA_FRI_MY_LBR_CLIENT_H
#define _KUKA_FRI_MY_LBR_CLIENT_H

#include <fstream>
#include <chrono>
#include "friLBRClient.h"
#include "exp_robots.h"
#include "exp_trajs.h"


using namespace KUKA::FRI;
/**
 * \brief Template client implementation.
 */
class MyLBRClient : public LBRClient
{

public:

    /**
    * \brief Constructor.
    */
    MyLBRClient(double freqHz, double amplitude);

    /**
    * \brief Destructor.
    */
    ~MyLBRClient();

    /**
    * \brief Callback for FRI state changes.
    *
    * @param oldState
    * @param newState
    */
    virtual void onStateChange(ESessionState oldState, ESessionState newState);

    /**
    * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void monitor();

    /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void waitForCommand();


    /**
    * \brief Callback for the FRI state 'Commanding Active'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void command();

private:


    // Create iiwa as child of primitive class
    iiwa14 *myLBR;

    // Double values to get measured robot values and command robot values
    double tau_command[7];
    double q_init[7];
    double q_init2[7];
    double q_command[7];
    double q_curr[7];
    double q_old[7];
    double q_arr[7];
    double dq_arr[7];


    // Time parameters for control loop
    double ts;
    double t;

    // The number of time steps for the simulation
    int n_step;

    // Choose the body you want to control and the position on this body
    Eigen::Vector3d  p_curr;
    Eigen::Vector3d dp_curr;

    double t1i;
    double t1f;
    double t2i;
    double t2f;
    double t3i;
    double t3f;
    double t4i;
    double t4f;
    double t5i;
    double t5f;
    double t6i;
    double t6f;
    double t7i;
    double t7f;
    double t8i;
    double t8f;
    double t9i;
    double t9f;
    double t10i;
    double t10f;

    // The virtual task-space trajectory, position.
    Eigen::Vector3d p0;
    Eigen::Vector3d p01;
    Eigen::Vector3d p02;
    Eigen::Vector3d p03;
    Eigen::Vector3d p04;
    Eigen::Vector3d p05;
    Eigen::Vector3d p06;
    Eigen::Vector3d p07;
    Eigen::Vector3d p08;
    Eigen::Vector3d p09;
    Eigen::Vector3d p10;

    Eigen::Vector3d w01;
    Eigen::Vector3d w02;
    Eigen::Vector3d w03;
    Eigen::Vector3d w04;
    Eigen::Vector3d w05;
    Eigen::Vector3d w06;
    Eigen::Vector3d w07;
    Eigen::Vector3d w08;

    Eigen::Vector3d dp0;
    Eigen::Vector3d dp01;
    Eigen::Vector3d dp02;
    Eigen::Vector3d dp03;
    Eigen::Vector3d dp04;
    Eigen::Vector3d dp05;
    Eigen::Vector3d dp06;
    Eigen::Vector3d dp07;
    Eigen::Vector3d dp08;
    Eigen::Vector3d dp09;
    Eigen::Vector3d dp10;

    double t_freq;

    Eigen::Vector3d p0i;
    Eigen::Vector3d delx;
    Eigen::Vector3d delx1;
    Eigen::Vector3d delx2;

    Eigen::Vector3d dely;
    Eigen::Vector3d dely1;
    Eigen::Vector3d dely2;

    Eigen::Vector3d delz;
    Eigen::Vector3d delz1;
    Eigen::Vector3d delz2;

    // Current position and velocity as Eigen vector
    Eigen::VectorXd q;
    Eigen::VectorXd q0_init;
    Eigen::VectorXd q0_curr;
    Eigen::VectorXd q0_init2;
    Eigen::VectorXd dq;

    // Command torque vectors (with and without constraints)
    Eigen::VectorXd tau_ctrl;
    Eigen::VectorXd tau_prev;
    Eigen::VectorXd tau_imp1;
    Eigen::VectorXd tau_imp2;
    Eigen::VectorXd tau_imp3;
    Eigen::VectorXd tau_pprev;
    Eigen::VectorXd tau_total;

    // DECLARE VARIABLES FOR YOUR CONTROLLER HERE!!!
    Eigen::Matrix4d H;
    Eigen::MatrixXd J;
    Eigen::MatrixXd Jp;     // The position Jacobian
    Eigen::MatrixXd Jr;     // The rotation Jacobian
    Eigen::Matrix3d Kp;     // Task-space stiffness, position
    Eigen::Matrix3d Bp;     // Task-space damping, position
    Eigen::MatrixXd Bq;     // Joint-space damping.
    Eigen::MatrixXd Kq;     // Joint-space damping.

    Eigen::Matrix3d R_init;  // SO(3) Matrix for the initial orientation
    Eigen::Matrix3d R_curr;  // SO(3) Matrix for the current orientation
    Eigen::Matrix3d R_des;   // SO(3) Matrix for the desired orientation
    Eigen::Matrix3d R_del;   // SO(3) Matrix for the desired orientation

    Eigen::Matrix3d R_des1;   // SO(3) Matrix for the desired orientation
    Eigen::Matrix3d R_des2;   // SO(3) Matrix for the desired orientation
    Eigen::Matrix3d R_des3;   // SO(3) Matrix for the desired orientation


    MinimumJerkTrajectory *mjt_p1;
    MinimumJerkTrajectory *mjt_p2;
    MinimumJerkTrajectory *mjt_p3;
    MinimumJerkTrajectory *mjt_p4;
    MinimumJerkTrajectory *mjt_p5;
    MinimumJerkTrajectory *mjt_p6;
    MinimumJerkTrajectory *mjt_p7;
    MinimumJerkTrajectory *mjt_p8;
    MinimumJerkTrajectory *mjt_p9;
    MinimumJerkTrajectory *mjt_p10;
    MinimumJerkTrajectory *mjt_p11;
    MinimumJerkTrajectory *mjt_p12;

    MinimumJerkTrajectory *mjt_w1;
    MinimumJerkTrajectory *mjt_w2;
    MinimumJerkTrajectory *mjt_w3;
    MinimumJerkTrajectory *mjt_w4;
    MinimumJerkTrajectory *mjt_w5;
    MinimumJerkTrajectory *mjt_w6;
    MinimumJerkTrajectory *mjt_w7;
    MinimumJerkTrajectory *mjt_w8;
    MinimumJerkTrajectory *mjt_w9;
    MinimumJerkTrajectory *mjt_w10;
    MinimumJerkTrajectory *mjt_w11;
    MinimumJerkTrajectory *mjt_w12;


    double Kq_gain;
    double tmp_gain;

    double  D1;
    double  D2;
    double  D3;

    double ti  ;
    double toff;

    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point end;

    // The axis-angle of R_del
    double theta;
    Eigen::Matrix3d w_axis_mat;
    Eigen::Vector3d w_axis;

    // File for Saving the Data
    std::ofstream f;
    Eigen::IOFormat fmt;
};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
