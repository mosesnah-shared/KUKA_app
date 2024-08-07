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
    Eigen::Vector3d  p_init;
    Eigen::Vector3d dp_curr;

    Eigen::Vector3d  p_pour;

    // The virtual task-space trajectory, position.
    Eigen::Vector3d p0;
    Eigen::Vector3d w01;

    Eigen::Vector3d dp0;

    // Current position and velocity as Eigen vector
    Eigen::VectorXd q;
    Eigen::VectorXd q0_init;
    Eigen::VectorXd q0_curr;
    Eigen::VectorXd dq;

    // Command torque vectors (with and without constraints)
    Eigen::VectorXd tau_ctrl;
    Eigen::VectorXd tau_prev;
    Eigen::VectorXd tau_imp1;
    Eigen::VectorXd tau_imp2;
    Eigen::VectorXd tau_imp3;
    Eigen::VectorXd tau_imp4;
    Eigen::VectorXd tau_pprev;
    Eigen::VectorXd tau_total;

    // DECLARE VARIABLES FOR YOUR CONTROLLER HERE!!!
    Eigen::Matrix4d H;
    Eigen::Matrix4d Hfix;
    Eigen::MatrixXd J;
    Eigen::MatrixXd Jp;     // The position Jacobian

    // The kinematics of the 2nd position
    Eigen::Matrix4d H_2nd;
    Eigen::MatrixXd J_2nd;
    Eigen::MatrixXd Jp_2nd;
    Eigen::Vector3d p0_2nd;
    Eigen::Vector3d p_curr_2nd;
    Eigen::Vector3d dp_curr_2nd;

    // Torque gain
    double Kp_gain1;
    double Kp_gain2;

    Eigen::MatrixXd Jr;     // The rotation Jacobian
    Eigen::Matrix3d Kp;     // Task-space stiffness, position
    Eigen::Matrix3d Bp;     // Task-space damping, position
    Eigen::MatrixXd Bq;     // Joint-space damping.
    Eigen::MatrixXd Kq;     // Joint-space damping.

    Eigen::Matrix3d R_init;  // SO(3) Matrix for the initial orientation
    Eigen::Matrix3d R_curr;  // SO(3) Matrix for the current orientation
    Eigen::Matrix3d R_des;   // SO(3) Matrix for the desired orientation
    Eigen::Matrix3d R_del;   // SO(3) Matrix for the desired orientation
    Eigen::Matrix3d R_init_des;

    MinimumJerkTrajectory *mjt_w;
    MinimumJerkTrajectory *mjt_p1;
    MinimumJerkTrajectory *mjt_p2;

    // Offset for Pouring
    Eigen::Vector3d offset1;
    Eigen::Vector3d offset2;
    Eigen::Vector3d p0_fix;


    // Data from Imitation Learning
    Eigen::MatrixXd pos_data;
    Eigen::MatrixXd R_data_shake;
    Eigen::MatrixXd R_data_pour;

    double Kq_gain;

    int N_pos_shake;
    int N_orient_shake;
    int N_orient_pour;

    int N_curr_pos;
    int N_curr_orient_shake;
    int N_curr_orient_pour;

    double t_pour_done;
    double t_shake_done;
    double t_pressed_first;
    double t_pressed_second;
    double t_pressed_second_p;

    bool is_pressed_first;
    bool is_pressed_second;
    bool is_pos_done;
    bool is_shake_done;
    bool is_pour_done;
    bool is_fix_pose;

    int n_shake;
    int nn_step;
    int sgn;
    double toff;

    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point end;

    Eigen::Matrix3d w_axis_mat;
    Eigen::Vector3d w_axis;

    // File for Saving the Data
    std::ofstream f;
    Eigen::IOFormat fmt;
};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
