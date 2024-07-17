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
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <chrono>
#include <iomanip>
#include <vector>


#include "MyLBRClient.h"
#include "exp_robots.h"
#include "exp_trajs.h"

using namespace std;

#ifndef M_PI
    #define M_PI 3.14159265358979
#endif

#ifndef NCoef
    #define NCoef 1
#endif

static double filterOutput[ 7 ][ NCoef+1 ]; // output samples. Static variables are initialised to 0 by default.
static double  filterInput[ 7 ][ NCoef+1 ]; //  input samples. Static variables are initialised to 0 by default.

Eigen::MatrixXd readCSV(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Couldn't open the file: " << filename << endl;
        exit(1);
    }

    vector<vector<double>> values;

    string line;
    int lineNum = 0;
    int numCols = 0;
    while (getline(file, line)) {
        ++lineNum;
        stringstream ss(line);
        string cell;
        vector<double> row;
        while (getline(ss, cell, ',')) {
            try {
                row.push_back(stod(cell));
            } catch (const std::invalid_argument& e) {
                cerr << "Error: Invalid argument at line " << lineNum << ", column: " << row.size() + 1 << endl;
                exit(1);
            }
        }
        values.push_back(row);
        if (numCols == 0)
            numCols = row.size();
        else if (row.size() != numCols) {
            cerr << "Error: Inconsistent number of columns in the CSV file." << endl;
            exit(1);
        }
    }

    if (values.empty()) {
        cerr << "Error: CSV file is empty." << endl;
        exit(1);
    }

    // Create Eigen Matrix
    Eigen::MatrixXd mat(values.size(), numCols);
    for (int i = 0; i < values.size(); ++i) {
        for (int j = 0; j < numCols; ++j) {
            mat(i, j) = values[i][j];
        }
    }

    return mat;
}


Eigen::Matrix3d R3_to_so3(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skewSym;
    skewSym <<  0,      -v(2),  v(1),
                v(2),   0,     -v(0),
                -v(1),   v(0),  0;
    return skewSym;
}

Eigen::Matrix3d R3_to_SO3(const Eigen::Vector3d& axis_angle) {
    // Normalize the axis of rotation
    Eigen::Vector3d axis = axis_angle.normalized();

    // Compute the angle of rotation
    double angle = axis_angle.norm();

    // Compute the skew-symmetric matrix
    Eigen::Matrix3d skew_sym = R3_to_so3(axis);

    // Compute the rotation matrix using Rodriguez formula
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity()
                                      + std::sin(angle) * skew_sym
                                      + (1 - std::cos(angle)) * (skew_sym * skew_sym);

    // Perform Gram-Schmidt orthogonalization to ensure orthogonality
    //    Eigen::HouseholderQR<Eigen::Matrix3d> qr(rotation_matrix);
    //    rotation_matrix = qr.householderQ();

    return rotation_matrix;
}



Eigen::Vector3d so3_to_R3(const Eigen::Matrix3d& skewSym) {
    Eigen::Vector3d v;
    v << skewSym(2, 1), skewSym(0, 2), skewSym(1, 0);
    return v;
}

Eigen::Matrix3d SO3_to_so3(const Eigen::Matrix3d& R_del ) {
    Eigen::Matrix3d w_axis_mat;


    if (std::abs(R_del.trace() + 1) <= 1e-7) {
        if (std::abs(R_del(2, 2) + 1) >= 1e-7) {
            Eigen::Vector3d tmp = Eigen::Vector3d::Zero();
            tmp(0) = 1. / sqrt(2 * (1 + R_del(2, 2))) * R_del(0, 2);
            tmp(1) = 1. / sqrt(2 * (1 + R_del(2, 2))) * R_del(1, 2);
            tmp(2) = 1. / sqrt(2 * (1 + R_del(2, 2))) * (1 + R_del(2, 2));
            w_axis_mat = R3_to_so3(tmp);
        }
        else if (std::abs(R_del(1, 1) + 1) >= 1e-7) {
            Eigen::Vector3d tmp = Eigen::Vector3d::Zero();
            tmp(0) = 1. / sqrt(2 * (1 + R_del(1, 1))) * R_del(0, 1);
            tmp(1) = 1. / sqrt(2 * (1 + R_del(1, 1))) * (1 + R_del(1, 1));
            tmp(2) = 1. / sqrt(2 * (1 + R_del(1, 1))) * R_del(2, 1);
            w_axis_mat = R3_to_so3(tmp);
        }
        else {
            Eigen::Vector3d tmp = Eigen::Vector3d::Zero();
            tmp(0) = 1. / sqrt(2 * (1 + R_del(0, 0))) * (1 + R_del(0, 0));
            tmp(1) = 1. / sqrt(2 * (1 + R_del(0, 0))) * R_del(1, 0);
            tmp(2) = 1. / sqrt(2 * (1 + R_del(0, 0))) * R_del(2, 0);
            w_axis_mat = R3_to_so3(tmp);
        }
    }
    else {
        // Calculate theta
        Eigen::Matrix3d diff = R_del - Eigen::Matrix3d::Identity();

        if ( std::abs( diff.norm( ) ) < 1e-6 )
        {
            w_axis_mat = Eigen::Matrix3d::Zero(3, 3);
        }
        else
        {
            double theta = std::acos( 0.5* ( R_del.trace( ) - 1 ) );
            w_axis_mat = theta*(R_del - R_del.transpose()) / (2 * sin(theta));
        }
    }

    return w_axis_mat;
}

//******************************************************************************
MyLBRClient::MyLBRClient(double freqHz, double amplitude)
{

    /** Initialization */
    // !! WARNING !!
    // THESE JOINT POSITION VALUES MUST BE THE SAME WITH THE JAVA APPLICATION!!
    q_init[0] =  -3.21 * M_PI/180;
    q_init[1] =  46.19 * M_PI/180;
    q_init[2] =  17.52 * M_PI/180;
    q_init[3] = -87.16 * M_PI/180;
    q_init[4] =  -5.03 * M_PI/180;
    q_init[5] = -37.73 * M_PI/180;
    q_init[6] =  0.000 * M_PI/180;

    // Use Explicit-cpp to create your robot
    myLBR = new iiwa14( 1, "Dwight", Eigen::Vector3d( 0.0, 0.0, 0.15 ) );

    // Initialization must be called!!
    myLBR->init( );

    // Current position and velocity
    // These two variables are used as "Eigen" objects rather than a double array
    q  = Eigen::VectorXd::Zero( myLBR->nq );
    dq = Eigen::VectorXd::Zero( myLBR->nq );
    q0_init  = Eigen::VectorXd::Zero( myLBR->nq );
    q0_curr  = Eigen::VectorXd::Zero( myLBR->nq );

    // Time variables for control loop
    t      = 0;     // The current Time
    ts     = 0;     // The  sample Time
    n_step = 0;     // The number of time steps, integer

    // Initialize joint torques and joint positions (also needed for waitForCommand()!)
    for( int i=0; i < myLBR->nq; i++ )
    {
        q( i ) = q_init[ i ];
        q0_init( i ) = q_init[ i ];
        q_curr[ i ] = q_init[ i ];
         q_old[ i ] = q_init[ i ];

         // The Actual command of the joint-position and torque
          q_command[ i ] = 0.0;
        tau_command[ i ] = 0.0;
    }

    // Once Initialized, get the initial end-effector position
    // Forward Kinematics and the current position
    offset1 = Eigen::Vector3d(  0.00, 0.00, 0.00 );
    offset2 = Eigen::Vector3d( -0.04, 0.07, 0.13 );

    H = myLBR->getForwardKinematics( q, 7, offset1 );

    p_init  = H.block< 3, 1 >( 0, 3 );
    R_init  = H.block< 3, 3 >( 0, 0 );

    p_curr  = p_init;
    R_curr  = R_init;
    dp_curr = Eigen::VectorXd::Zero( 3 );

    // Set the Rdesired postures
    R_init_des << -0.3177, -0.2450,  0.9160,
                   0.6259,  0.6715,  0.3967,
                  -0.7123,  0.6993, -0.0599;

    Eigen::Vector3d wdel = so3_to_R3( SO3_to_so3( R_init.transpose( ) * R_init_des ) );

    mjt_w  = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ),  wdel, 1.0, 1.0 );
    w_axis  = Eigen::Vector3d::Zero( 3 );

    // The taus (or torques) for the command
    tau_ctrl   = Eigen::VectorXd::Zero( myLBR->nq );    // The torque from the controller design,
    tau_prev   = Eigen::VectorXd::Zero( myLBR->nq );    // The previous torque         , i.e., tau_{n-1} where tau_n is current value
    tau_pprev  = Eigen::VectorXd::Zero( myLBR->nq );    // The previous-previous torque, i.e., tau_{n-2} where tau_n is current value
    tau_total  = Eigen::VectorXd::Zero( myLBR->nq );    // The total tau which will be commanded

    tau_imp1   = Eigen::VectorXd::Zero( myLBR->nq );    // Position    Task-space  Impedance Control
    tau_imp2   = Eigen::VectorXd::Zero( myLBR->nq );    // Orientation Task-space  Impedance Control
    tau_imp3   = Eigen::VectorXd::Zero( myLBR->nq );    //             Joint-space Impedance Control

    nn_step = 1;

    // For the Task-space impedance control, the Forward Kinematics (H) and Hybrid Jacobian Matrix (JH) is Required
    H     = Eigen::Matrix4d::Zero( 4, 4 );
    H_2nd = Eigen::Matrix4d::Zero( 4, 4 );
    Hfix  = Eigen::Matrix4d::Zero( 4, 4 );
    J     = Eigen::MatrixXd::Zero( 6, myLBR->nq );
    Jp    = Eigen::MatrixXd::Zero( 3, myLBR->nq );
    Jr    = Eigen::MatrixXd::Zero( 3, myLBR->nq );

    // The stiffness/damping matrices
    Kp = 600 * Eigen::MatrixXd::Identity( 3, 3 );
    Bp =  40 * Eigen::MatrixXd::Identity( 3, 3 );

    Kq = 3.0 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );
    Bq = 4.5 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );
    Kq_gain = 1.0;

    // The gain of the both position modules
    Kp_gain1 = 1.0;
    Kp_gain2 = 0.0;

    // The Jacobian matrices
    J_2nd  = Eigen::MatrixXd::Zero( 6, myLBR->nq );
    Jp_2nd = Eigen::MatrixXd::Zero( 3, myLBR->nq );
    p0_2nd = Eigen::Vector3d::Zero( 3 );
    p_curr_2nd  = Eigen::Vector3d::Zero( 3 );
    dp_curr_2nd = Eigen::Vector3d::Zero( 3 );

    // Initial print
    printf( "Exp[licit](c)-cpp-FRI, https://explicit-robotics.github.io \n\n" );
    printf( "Robot '" );
    printf( "%s", myLBR->Name );
    printf( "' initialised. Ready to rumble! \n" );
    printf( "The current script runs Task-space Impedance Control, Position\n" );

    // Open a file
    f.open( "cocktail_data.txt" );
    fmt = Eigen::IOFormat(5, 0, ", ", "\n", "[", "]");

    // Read the Data
    pos_data     = readCSV( "/home/baxterplayground/Documents/DMPModular/data/csv/shake_pos.csv"    );
    R_data_shake = readCSV( "/home/baxterplayground/Documents/DMPModular/data/csv/shake_2p0scl.csv" );
    N_pos_shake    =     pos_data.cols( );
    N_orient_shake = R_data_shake.cols( )/3;

    // Pour 2nd Option is simply using minimum-jerk trajectory
    N_curr_pos          = 0;
    N_curr_orient_shake = 0;

    // Sign of data increase for pour
    sgn  = 1.0;
    toff = 3.0;

    p_pour = Eigen::Vector3d( 0.15, 0.10, -0.31 );
    mjt_p1 = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ),  p_pour, 3.0, 2.0 );
    mjt_p2 = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), -p_pour, 3.0, 3.0 );

    // Another movement before the pour
    mjt_w0 = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d( 0.0, 0.0, -0.9 ), 3.0, 2.0 );
    mjt_w1 = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d( 0.0, 0.0, -1.2 ), 5.5, 2.0 );
    mjt_w2 = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d( 0.0, 0.0,  0.8 ), 3.5, 8.5 );
    mjt_w3 = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d( 0.0, 0.0,  1.3 ), 3.0, 3.0 );

    std::cout << "Matrix size: " << pos_data.rows() << " rows x " << pos_data.cols() << " columns" << std::endl;
    std::cout << "Data Length for Orientation, Rhythmic: " << N_orient_shake << std::endl;

    is_pos_done   = false;
    is_pour_done  = false;
    is_shake_done = false;
    is_fix_pose   = false;

    is_pressed_first  = false;
    is_pressed_second = false;

    n_shake = 0;

    t_pour_done  = 0.0;
    t_shake_done = 0.0;

}


/**
* \brief Destructor
*
*/
MyLBRClient::~MyLBRClient()
{
    f.close( );
}

/**
* \brief Implements an IIR Filter which is used to send the previous joint position to the command function, so that KUKA's internal friction compensation can be activated. The filter function was generated by the application WinFilter (http://www.winfilter.20m.com/).
*
* @param NewSample The current joint position to be provided as input to the filter.
*/
// iir updates the filterInput and filterOutput
void iir(double NewSample[7])
{
    double ACoef[ NCoef+1 ] =
    {
        0.05921059165970496400,
        0.05921059165970496400
    };

    double BCoef[ NCoef+1 ] =
    {
        1.00000000000000000000,
        -0.88161859236318907000
    };

    // Shift the old samples
    for ( int i=0; i<7; i++ )
    {
        for( int n=NCoef; n>0; n-- )
        {
             filterInput[ i ][ n ] =  filterInput[ i ][ n-1 ];
            filterOutput[ i ][ n ] = filterOutput[ i ][ n-1 ];
        }
    }

    // Calculate the new output
    for ( int i=0; i<7; i++ )
    {
         filterInput[ i ][ 0 ] = NewSample[ i ];
        filterOutput[ i ][ 0 ] = ACoef[ 0 ] * filterInput[ i ][ 0 ];
    }

    for (int i=0; i<7; i++)
    {
        for( int n=1; n<=NCoef; n++ )
        {
            filterOutput[ i ][ 0 ] += ACoef[ n ] * filterInput[ i ][ n ] - BCoef[ n ] * filterOutput[ i ][ n ];
        }
    }
}

//******************************************************************************
void MyLBRClient::onStateChange( ESessionState oldState, ESessionState newState )
{
    LBRClient::onStateChange( oldState, newState );

    // react on state change events
    switch (newState)
    {
        case MONITORING_WAIT:
        {
            break;
        }
        case MONITORING_READY:
        {
            ts = robotState( ).getSampleTime( );
            break;
        }
        case COMMANDING_WAIT:
        {
            break;
        }
        case COMMANDING_ACTIVE:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}

//******************************************************************************
void MyLBRClient::monitor()
{

    // Copied from FRIClient.cpp
    robotCommand( ).setJointPosition( robotState( ).getCommandedJointPosition( ) );

    // Copy measured joint positions (radians) to q_curr, which is a double
    memcpy( q_curr, robotState( ).getMeasuredJointPosition( ), 7*sizeof(double) );

    // Initialize the q for the previous NCoef timesteps
    for( int i=0; i<NCoef+1; i++ )
    {
        iir( q_curr );
    }

}

//******************************************************************************
void MyLBRClient::waitForCommand()
{
    // If we want to command torques, we have to command them all the time; even in
    // waitForCommand(). This has to be done due to consistency checks. In this state it is
    // only necessary, that some torque vlaues are sent. The LBR does not take the
    // specific value into account.

    if(robotState().getClientCommandMode() == TORQUE)
    {
        robotCommand( ).setTorque( tau_command );
        robotCommand( ).setJointPosition( robotState( ).getIpoJointPosition( ) );            // Just overlaying same position
    }

}

//******************************************************************************
void MyLBRClient::command()
{

    // ************************************************************
    // Get robot measurements
    memcpy(  q_old, q_curr, 7*sizeof( double ) );
    memcpy( q_curr, robotState( ).getMeasuredJointPosition( ), 7*sizeof( double ) );

    for (int i=0; i < myLBR->nq; i++)
    {
        q[ i ] = q_curr[ i ];
    }

    for (int i=0; i < 7; i++)
    {
        dq[ i ] = ( q_curr[ i ] - q_old[ i ]) / ts;
    }

    // ************************************************************ //
    // ********************* CONTROLLER START ********************* //
    // ************************************************************ //

    start = std::chrono::steady_clock::now( );
    H = myLBR->getForwardKinematics( q, 7, offset1 );
    p_curr = H.block< 3, 1 >( 0, 3 );
    R_curr = H.block< 3, 3 >( 0, 0 );

    // Get the current end-effector velocity
    // Hybrid Jacobian Matrix (6x7) and its linear velocity part (3x7)
    J  = myLBR->getHybridJacobian( q, offset1 );
    Jp = J.block( 0, 0, 3, myLBR->nq );
    Jr = J.block( 3, 0, 3, myLBR->nq );

    // For the 2nd-module
    J_2nd  = myLBR->getHybridJacobian( q, offset2 );
    Jp_2nd = J_2nd.block( 0, 0, 3, myLBR->nq );
    H_2nd  = myLBR->getForwardKinematics( q, 7, offset2 );
    p_curr_2nd = H_2nd.block< 3, 1 >( 0, 3 );

    // Set the initial conditions
    w01   = mjt_w->getPosition( t );
    p0    = p_init;
    R_des = R_init * R3_to_SO3( w01 );

    // ============== START ============= //
    // Comment these out for full routine!
    // If over some time, go through the position array
    if( is_pressed_first && !is_pos_done )
    {
        // Position Update
        if( t_pressed_first >= 3.0 )
        {
            if ( n_step % nn_step == 0 )
            {
                N_curr_pos+=3;
            }

            if( N_curr_pos > N_pos_shake-1 )
            {
                N_curr_pos = N_pos_shake-1;
                is_pos_done = true;
            }
        }

        // Add Orientation
        if( t_pressed_first >= 23 && !is_pos_done )
        {
            N_curr_orient_shake += 6;
            if( N_curr_orient_shake > N_orient_shake-1 )
            {
                N_curr_orient_shake = 0;
            }
            nn_step = 2;
        }

    }
    else if( is_pos_done && !is_pressed_second && ( n_shake <= 3 ) )
    {
        N_curr_orient_shake += 6;

        if( N_curr_orient_shake > N_orient_shake-1 )
        {
            N_curr_orient_shake = 0;
            n_shake++;
        }

    }
    // The n_shake number above and below must be the same
    else if( is_pos_done && !is_pressed_second && ( n_shake > 2 ) )
    {
        is_shake_done = true;
    }
    else
    {
        N_curr_pos = 0;
        N_curr_orient_shake = 0;
    }
    // =============== END ============== //


    // Control the end-effector
    p0 += pos_data.col( N_curr_pos );

    if( is_pressed_first && is_shake_done )
    {
        Kq_gain -= 0.01;
        if( Kq_gain < 0.0 )
        {
            Kq_gain = 0.0;
        }
        p0 += mjt_p1->getPosition( t_shake_done );

        w_ready = mjt_w0->getPosition( t_shake_done );
    }

    // If t_pressed_second_p is done, then ready to pour
    // Save the position that will be used to fix the end-effector position
    if( t_pressed_second_p > 1.0 && !is_fix_pose && !is_pour_done )
    {
        Hfix = myLBR->getForwardKinematics( q, 7, offset2 );
        p0_2nd = Hfix.block< 3, 1 >( 0, 3 );
        is_fix_pose = true;

    }
    if( is_fix_pose && !is_pour_done )
    {
        Kp_gain1 -= 0.002;
        Kp_gain2 += 0.002;

        if( Kp_gain1 < 0 )
        {
            Kp_gain1 = 0.0;
        }
        if( Kp_gain2 > 1.0  )
        {
            Kp_gain2 = 1.0;
        }
    }

    // Calculate the current end-effector's position
    dp_curr     =     Jp * dq;
    dp_curr_2nd = Jp_2nd * dq;

    // Control for orientation
    R_des = R_des * R_data_shake.block< 3, 3 >( 0, 3*N_curr_orient_shake );

    // If position done and second button pressed
    if( is_pos_done && is_pressed_second && !is_pour_done )
    {

        // Add orientation for pouring
        w_pour1 = mjt_w1->getPosition( t_pressed_second );
        w_pour2 = mjt_w2->getPosition( t_pressed_second );

        // This should be large enough!
        if( t_pressed_second >= 14.0 )
        {
            is_pour_done = true;
        }
    }

    if( is_pos_done && is_pressed_second && is_pour_done )
    {
        Kp_gain1 += 0.002;
        Kp_gain2 -= 0.002;

        if( Kp_gain1 > 1.0  )
        {
            Kp_gain1 = 1.0;
        }
        if( Kp_gain2 < 0 )
        {
            Kp_gain2 = 0.0;
        }

        p0 += mjt_p2->getPosition( t_pour_done );

        if( t_pour_done >= 2.4 )
        {
            Kq_gain += 0.02;
            if( Kq_gain > 1.0 )
            {
                Kq_gain = 1.0;
            }
        }
        w_done = mjt_w3->getPosition( t_pour_done );
    }

    R_des = R_des * R3_to_SO3( w_ready+w_pour1+w_pour2+w_done );

    // The difference between the two rotation matrices
    R_del  = R_curr.transpose( ) * R_des;
    w_axis = so3_to_R3( SO3_to_so3( R_del ) );

    tau_imp1 = Jp.transpose( ) * ( Kp * ( p0 - p_curr ) + Bp * ( - dp_curr ) );

    tau_imp2 = Kq_gain * Kq * ( q0_init - q ) + Bq * ( -dq );
    tau_imp3 = Jr.transpose( ) * ( 70 * R_curr * w_axis - 5 * Jr * dq );

    // The 2nd module
    tau_imp4 = Jp_2nd.transpose( ) * ( Kp * ( p0_2nd - p_curr_2nd ) + Bp * ( - dp_curr_2nd ) );

    // Superposition of Mechanical Impedances
    tau_ctrl = Kp_gain1 * tau_imp1 + tau_imp2 + tau_imp3 + Kp_gain2 * tau_imp4;

    // ************************************************************ //
    // ********************* CONTROLLER ENDS ********************* //
    // ************************************************************ //

    // A simple filter for the torque command
    tau_total = ( tau_ctrl + tau_prev + tau_pprev ) / 3;

    for ( int i=0; i<7; i++ )
    {
          q_command[ i ] = filterOutput[ i ][ 0 ];
        tau_command[ i ] = tau_total[ i ];
    }

    // Command values (must be double arrays!)
    if ( robotState().getClientCommandMode( ) == TORQUE )
    {
        robotCommand( ).setJointPosition( q_command );
        robotCommand( ).setTorque( tau_command );
    }

    // IIR filter input
    iir( q_curr );

    // If the first-step of the controller
    if ( n_step == 0 )
    {
        tau_prev  = tau_ctrl;
        tau_pprev = tau_ctrl;
    }
    else
    {
        tau_prev  = tau_ctrl;
        tau_pprev = tau_prev;
    }

    // First Button Pressed
    if ( robotState().getBooleanIOValue( "MediaFlange.UserButton" ) && !is_pressed_first && !is_pressed_second )
    {
        is_pressed_first = true;
        t_pressed_first = 0.0;

        std::cout << "Button Pressed First!" << std::endl;
    }

    // Second Button Pressed
    if ( robotState().getBooleanIOValue( "MediaFlange.UserButton" ) && is_pressed_first && is_pos_done && !is_pressed_second )
    {
        is_pressed_second = true;
        t_pressed_second = 0.0;
        t_pressed_second_p = 0.0;

        std::cout << "Button Pressed Second!" << std::endl;
    }

    // Add the sample time to the current time
    t += ts;
    n_step++;

    if( is_pressed_first )
    {
        t_pressed_first += ts;
    }
    if( is_shake_done )
    {
        t_shake_done += ts;
    }

    if( is_pressed_second )
    {
        t_pressed_second += ts;
        t_pressed_second_p += ts;
    }

    if( is_pour_done )
    {
        t_pour_done += ts;
    }


}
