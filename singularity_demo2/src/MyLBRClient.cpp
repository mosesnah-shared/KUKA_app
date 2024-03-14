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
    myLBR = new iiwa14( 1, "Dwight" );

    // Initialization must be called!!
    myLBR->init( );

    // Current position and velocity
    // These two variables are used as "Eigen" objects rather than a double array
    q  = Eigen::VectorXd::Zero( myLBR->nq );
    dq = Eigen::VectorXd::Zero( myLBR->nq );
    q0_init = Eigen::VectorXd::Zero( myLBR->nq );

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
    H = myLBR->getForwardKinematics( q );
    p_curr  = H.block< 3, 1 >( 0, 3 );
    R_des   = H.block< 3, 3 >( 0, 0 );

    R_curr  = R_des;
    R_del   = R_curr.transpose( ) * R_des;
    dp_curr = Eigen::VectorXd::Zero( 3 );

    // Create the Minimum-jerk trajectory
    D   = 3.3;
    ti   = 1.0;
    toff = -0.8;

    p0i = p_curr;

    delx = Eigen::Vector3d( 0.4, 0.0, 0.0 );
    dely = Eigen::Vector3d( 0.0, 0.4, 0.0 );
    delz = Eigen::Vector3d( 0.0, 0.0, 0.4 );

    mjt1  = new MinimumJerkTrajectory( 3,                              p0i,    p0i + delx, D, ti                );    // Forward
    mjt2  = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ),        - delx, D, ti + 1 * ( D + toff ) - 0.3 );    // Backward
    mjt3  = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ),          delx, D, ti + 2 * ( D + toff ) );    // Forward
    mjt4  = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ),        - delx, D, ti + 3 * ( D + toff ) - 0.3 );    // Backward

    t_freq = ti + 4 * ( D + toff ) - 0.3;

    // The taus (or torques) for the command
    tau_ctrl   = Eigen::VectorXd::Zero( myLBR->nq );    // The torque from the controller design,
    tau_prev   = Eigen::VectorXd::Zero( myLBR->nq );    // The previous torque         , i.e., tau_{n-1} where tau_n is current value
    tau_pprev  = Eigen::VectorXd::Zero( myLBR->nq );    // The previous-previous torque, i.e., tau_{n-2} where tau_n is current value
    tau_total  = Eigen::VectorXd::Zero( myLBR->nq );    // The total tau which will be commanded

    tau_imp1   = Eigen::VectorXd::Zero( myLBR->nq );    // Position    Task-space  Impedance Control
    tau_imp2   = Eigen::VectorXd::Zero( myLBR->nq );    // Orientation Task-space  Impedance Control
    tau_imp3   = Eigen::VectorXd::Zero( myLBR->nq );    //             Joint-space Impedance Control

    w_axis_mat = Eigen::Matrix3d::Zero( 3, 3 );
    w_axis     = Eigen::Vector3d::Zero( 3 );

    // For the Task-space impedance control, the Forward Kinematics (H) and Hybrid Jacobian Matrix (JH) is Required
    H  = Eigen::Matrix4d::Zero( 4, 4 );
    J  = Eigen::MatrixXd::Zero( 6, myLBR->nq );
    Jp = Eigen::MatrixXd::Zero( 3, myLBR->nq );
    Jr = Eigen::MatrixXd::Zero( 3, myLBR->nq );

    // The stiffness/damping matrices
    Kp = 800 * Eigen::MatrixXd::Identity( 3, 3 );
    Bp =  80 * Eigen::MatrixXd::Identity( 3, 3 );

    Kq = 6.0 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );
    Bq = 1.5 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );

    // Initial print
    printf( "Exp[licit](c)-cpp-FRI, https://explicit-robotics.github.io \n\n" );
    printf( "Robot '" );
    printf( "%s", myLBR->Name );
    printf( "' initialised. Ready to rumble! \n" );
    printf( "The current script runs Task-space Impedance Control, Position\n" );

    Kq_gain = 0;

        // Open a file
    f.open( "singularity_test1.txt" );
    fmt = Eigen::IOFormat(5, 0, ", ", "\n", "[", "]");


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
    memcpy( q_old, q_curr, 7*sizeof( double ) );
    memcpy( q_curr, robotState( ).getMeasuredJointPosition( ), 7*sizeof(double) );

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
    H = myLBR->getForwardKinematics( q );
    p_curr = H.block< 3, 1 >( 0, 3 );
    R_curr = H.block< 3, 3 >( 0, 0 );

    // The difference between the two rotation matrices
    R_del   = R_curr.transpose( ) * R_des;

    theta = acos( ( R_del.trace( ) - 1 )/2 );

    if( theta >= 0.001 )
    {
        w_axis_mat = ( R_del - R_del.transpose( ) ) / ( 2 * sin( theta ) );
    }
    else
    {
        w_axis_mat = Eigen::Matrix3d::Zero( 3, 3 );
    }

    w_axis( 0 ) = -w_axis_mat( 1, 2 );
    w_axis( 1 ) =  w_axis_mat( 0, 2 );
    w_axis( 2 ) = -w_axis_mat( 0, 1 );

    // Get the current end-effector velocity
    // Hybrid Jacobian Matrix (6x7) and its linear velocity part (3x7)
    J  = myLBR->getHybridJacobian( q );

    Jp = J.block( 0, 0, 3, myLBR->nq );
    Jr = J.block( 3, 0, 3, myLBR->nq );

    // Calculate the current end-effector's position
    dp_curr = Jp * dq;

    // Get the virtual trajectory
    p01  = mjt1->getPosition( std::fmod( t, t_freq ) );
    dp01 = mjt1->getVelocity( std::fmod( t, t_freq ) );
    p02  = mjt2->getPosition( std::fmod( t, t_freq ) );
    dp02 = mjt2->getVelocity( std::fmod( t, t_freq ) );
    p03  = mjt3->getPosition( std::fmod( t, t_freq ) );
    dp03 = mjt3->getVelocity( std::fmod( t, t_freq ) );
    p04  = mjt4->getPosition( std::fmod( t, t_freq ) );
    dp04 = mjt4->getVelocity( std::fmod( t, t_freq ) );

    p0  =  p01 +  p02 +  p03 +  p04;
    dp0 = dp01 + dp02 + dp03 + dp04;

    // Calculate the tau
    // For Maintaining the Robot Posture
    //    tau_imp1 = Jp.transpose( ) * ( Kp * ( p0i - p_curr ) + Bp * ( - dp_curr ) );

    if( t >= ( ti + 1 * ( D + toff ) ) && t <= ( ti + 1 * ( D + toff ) + 0.5*D ) )
    {
        if( Kq_gain <= 1 )
        {
            Kq_gain += 0.001;
        }
        else
        {
            Kq_gain = 1;
        }
    }

    if( t >= ( ti + 1 * ( D + toff ) + 0.5*D ) && t <= ( ti + 2 * ( D + toff ) ) )
    {
        if( Kq_gain >= 0 )
        {
            Kq_gain -= 0.001;
        }
        else
        {
            Kq_gain = 0;
        }
    }

    if( t >= ( ti + 3 * ( D + toff ) ) && t <= ( ti + 3 * ( D + toff ) + 0.5*D ) )
    {
        if( Kq_gain <= 1 )
        {
            Kq_gain += 0.001;
        }
        else
        {
            Kq_gain = 1;
        }
    }

    if( t >= ( ti + 3 * ( D + toff ) + 0.5*D ) && t <= ( ti + 4 * ( D + toff ) ) )
    {
        if( Kq_gain >= 0 )
        {
            Kq_gain -= 0.001;
        }
        else
        {
            Kq_gain = 0;
        }
    }

    tau_imp1 = Jp.transpose( ) * ( Kp * ( p0 - p_curr ) + Bp * ( dp0 - dp_curr ) );
    tau_imp2 = Kq_gain * Kq * ( q0_init - q ) + Bq * ( -dq );
    tau_imp3 = Jr.transpose( ) * ( 50 * R_curr * w_axis * theta - 5 * Jr * dq );

    // Superposition of Mechanical Impedances
    tau_ctrl = tau_imp1 + tau_imp2 + tau_imp3;

    // Saving not every time but every

    // If the counter reaches the threshold, print to console
    if ( n_step == 5 )
    {
        f << "Time: " << std::fixed << std::setw( 5 ) << t;
        f << "  q values: " << q.transpose( ).format( fmt );
        f << " p0 values: " << p0.transpose( ).format( fmt ) << std::endl;
        end = std::chrono::steady_clock::now( );

        std::cout << "Elapsed time for The Torque Calculation "
                  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
                  << " us" << std::endl;
        n_step = 0;
    }



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


    // Add the sample time to the current time
    t += ts;
    n_step++;


}
