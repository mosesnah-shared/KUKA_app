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
#include <random>
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
    double t_sub; // Separate t_sub is useful
    double t_max;

    // The number of time steps for the simulation
    int n_step;

    // Choose the body you want to control and the position on this body
    Eigen::Vector3d  p_curr;
    Eigen::Vector3d dp_curr;

    // The virtual task-space trajectory, position.
    Eigen::Vector3d p0;
    Eigen::Vector3d p0_right;
    Eigen::Vector3d p0_left;

    Eigen::Vector3d dp0;

    double t_freq;
    double t_rot_tmp;

    Eigen::Vector3d p0i;
    Eigen::Vector3d p0_start;
    Eigen::Vector3d p0_write;
    Eigen::Vector3d delz;

    Eigen::Matrix3d R0i;
    Eigen::Matrix3d R0_start;
    Eigen::Matrix3d Rmat0;

    Eigen::Matrix3d Rtmp1;
    Eigen::Matrix3d Rtmp2;
    double theta_tmp;



    Eigen::Vector3d del1;
    Eigen::Vector3d del2;

    // The box where the robot should play
    Eigen::Vector2d pmax;
    Eigen::Vector2d pmin;

    // Current position and velocity as Eigen vector
    Eigen::VectorXd q;
    Eigen::VectorXd q_start;
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
    Eigen::Matrix4d H_start;
    Eigen::MatrixXd J;
    Eigen::MatrixXd Jp;     // The position Jacobian
    Eigen::MatrixXd Jr;     // The rotation Jacobian
    Eigen::Matrix3d Kp;     // Task-space stiffness, position
    Eigen::Matrix3d Bp;     // Task-space damping, position
    Eigen::MatrixXd Bq;     // Joint-space damping.
    double          kr;     // Task-space stiffness, orientation
    double          br;     // Task-space   damping, orientation

    Eigen::Matrix3d R_curr;  // SO(3) Matrix for the current orientation
    Eigen::Matrix3d R_del;   // SO(3) Matrix for the desired orientation

    double D1;
    double D2;
    double ti;

    // From current to set position
    MinimumJerkTrajectory *mjt1;
    MinimumJerkTrajectory *mjt2;

    MinimumJerkTrajectory *mjt3;
    MinimumJerkTrajectory *mjt4;

    int tmp_freq;

    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point end;

    // The axis-angle of R_del
    double theta;
    Eigen::Matrix3d w_axis_tmp;
    Eigen::Matrix3d w_axis_mat;
    Eigen::Vector3d w_axis;

    // Data from Imitation Learning
    Eigen::MatrixXd pos_data;
    Eigen::MatrixXd vel_data;

    // Number of data points, position
    int N_imit_pos;
    int N_curr_pos;

    // Radius and angular velocity of the oscillation
    double r_osc;
    double omega_osc;

    // File for Saving the Data
    std::ofstream f;
    Eigen::IOFormat fmt;

    bool is_erase;
    bool is_pressed;
    bool is_run_imit_pos;
    bool is_all_done;
    bool is_lift_up;


};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
