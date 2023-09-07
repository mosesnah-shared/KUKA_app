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
#include <random>


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

Eigen::Vector3d generateRandom3DVector( )
{
    // Seed the random number generator
    static std::random_device rd;
    static std::mt19937 gen( rd( ) );
    static std::uniform_real_distribution<double> distribution( 0.0, 1.0 );

    Eigen::Vector3d randomVector;
    for (int i = 0; i < 3; ++i )
    {
        randomVector( i ) = distribution( gen );
    }

    return randomVector;
}

bool isInRange( const Eigen::Vector3d& vector, const Eigen::Vector3d& minVals, const Eigen::Vector3d& maxVals )
{
    for ( int i = 0; i < vector.size(); ++i )
    {
        if ( vector( i ) < minVals( i ) || vector( i ) > maxVals( i ) )
        {
            return false;
        }
    }
    return true;
}

void clampVector(Eigen::Vector3d& vector, const Eigen::Vector3d& minVals, const Eigen::Vector3d& maxVals )
{
    for (int i = 0; i < vector.size(); ++i)
    {
        if ( vector( i ) < minVals( i ) )
        {
            vector( i ) = minVals( i );
        }
        else if ( vector(i) > maxVals( i ) )
        {
            vector( i ) = maxVals( i );
        }
    }
}


//******************************************************************************
MyLBRClient::MyLBRClient(double freqHz, double amplitude)
{

    /** Initialization */
    // !! WARNING !!
    // THESE JOINT POSITION VALUES MUST BE THE SAME WITH THE JAVA APPLICATION!!
    q_init[0] =   0.00 * M_PI/180;
    q_init[1] =  28.56 * M_PI/180;
    q_init[2] =  17.54 * M_PI/180;
    q_init[3] = -87.36 * M_PI/180;
    q_init[4] = -7.820 * M_PI/180;
    q_init[5] = 75.560 * M_PI/180;
    q_init[6] = -9.010 * M_PI/180;

    // Use Explicit-cpp to create your robot
    myLBR = new iiwa14( 1, "Dwight" );

    // Initialization must be called!!
    myLBR->init( );

    // Initialization Of current position and velocity
    // These two variables are used as "Eigen" objects rather than a double array
    q  = Eigen::VectorXd::Zero( myLBR->nq );
    dq = Eigen::VectorXd::Zero( myLBR->nq );

    // Initialize joint torques and joint positions (also needed for waitForCommand()!)
    for( int i=0; i < myLBR->nq; i++ )
    {
        q( i ) = q_init[ i ];

        q_curr[ i ] = q_init[ i ];
         q_old[ i ] = q_init[ i ];

         // The Actual command of the joint-position and torque
          q_command[ i ] = 0.0;
        tau_command[ i ] = 0.0;
    }

    // Time variables for control loop
    t      = 0;     // The current Time
    ts     = 0;     // The  sample Time, i.e., dt,
    n_step = 0;     // The number of time steps, integer

    // Number of trials of the movement
    n_trials = 0;

    // Once Initialized, get the initial end-effector position
    // Forward Kinematics and the current position
    H = myLBR->getForwardKinematics( q );
    p_curr = H.block< 3, 1 >( 0, 3 );
    R_curr = H.block< 3, 3 >( 0, 0 );

    // These values will be the initial values
    p0i = p_curr;
    R0i = R_curr;

    // The current end-effector position
    dp_curr = Eigen::VectorXd::Zero( 3 );

    // Defining the Minimum Jerk Trajectory
    double  D1     =  2.0;
    double  D2     =  2.0;
    double  alpha  =  0.5;   // Time when the goal appears, multiply with D1 such that D1 * alpha = tg
    double  ti     =    1;

    del1 = Eigen::Vector3d( 0.1, 0.0, 0.0 );
    del2 = Eigen::Vector3d( 0.0, 0.1, 0.0 );

    //    // Two Submovements are Defined
    mjt1 = new MinimumJerkTrajectory( 3,                              p0i,    p0i + del1, D1, ti              );
    mjt2 = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ),          del2, D2, ti + D1 * alpha );

    // Set the tmax from these values
    t_max = std::max( ti + D1, ti + D1*alpha + D2 );

    // The taus (or torques) for the command
    tau_ctrl   = Eigen::VectorXd::Zero( myLBR->nq );    // The torque from the controller design,
    tau_prev   = Eigen::VectorXd::Zero( myLBR->nq );    // The previous torque         , i.e., tau_{n-1} where tau_n is current value
    tau_pprev  = Eigen::VectorXd::Zero( myLBR->nq );    // The previous-previous torque, i.e., tau_{n-2} where tau_n is current value
    tau_total  = Eigen::VectorXd::Zero( myLBR->nq );    // The total tau which will be commanded

    tau_imp1   = Eigen::VectorXd::Zero( myLBR->nq );    // Position    Task-space  Impedance Control
    tau_imp2   = Eigen::VectorXd::Zero( myLBR->nq );    // Orientation Task-space  Impedance Control
    tau_imp3   = Eigen::VectorXd::Zero( myLBR->nq );    //             Joint-space Impedance Control

    // Axis angle of rotation, se(3) form and R3 form
    w_axis_mat = Eigen::Matrix3d::Zero( 3, 3 );
    w_axis     = Eigen::Vector3d::Zero( 3 );

    // For the Task-space impedance control, the Forward Kinematics (H) and Hybrid Jacobian Matrix (JH) is Required
    H  = Eigen::Matrix4d::Zero( 4, 4 );
    J  = Eigen::MatrixXd::Zero( 6, myLBR->nq );
    Jp = Eigen::MatrixXd::Zero( 3, myLBR->nq );
    Jr = Eigen::MatrixXd::Zero( 3, myLBR->nq );

    // The Translational stiffness/damping matrices
    Kp = 800 * Eigen::MatrixXd::Identity( 3, 3 );
    Bp =  80 * Eigen::MatrixXd::Identity( 3, 3 );
    Bq = 1.0 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );

    kr = 50;
    br = 5;

    // Set the playground cube of the robot
    pmax( 0 ) = p_curr( 0 ) + 0.3;
    pmax( 1 ) = p_curr( 1 ) + 0.1;
    pmax( 2 ) = p_curr( 2 ) + 0.2;

    pmin( 0 ) = p_curr( 0 ) - 0.3;
    pmin( 1 ) = p_curr( 1 ) - 0.1;
    pmin( 2 ) = p_curr( 2 ) - 0.2;

    // Initial print
    printf( "Exp[licit](c)-cpp-FRI, https://explicit-robotics.github.io \n\n" );
    printf( "Robot '" );
    printf( "%s", myLBR->Name );
    printf( "' initialised. Ready to rumble! \n" );
    printf( "The current script run sequence of submovements \n" );

    // Open a file
    f.open( "tmp.txt" );
    fmt = Eigen::IOFormat(5, 0, ", ", "\n", "[", "]");

    // Save the imporant parameters for the first line
    f << " D1: " << D1 << "D2: " << D2;
    f << " p0i: " << p0i.transpose( ).format( fmt );
    f << " del1: " << del1.transpose( ).format( fmt );
    f << " del2: " << del2.transpose( ).format( fmt );
    f << " ti: "   << ti;
    f << " tg: "   << D1 * alpha << std::endl;

    // set true of false
    is_continue = false;

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

    for ( int i=0; i < myLBR->nq; i++)
    {
        q[ i ] = q_curr[ i ];
    }

    for ( int i=0; i < 7; i++ )
    {
        dq[ i ] = ( q_curr[ i ] - q_old[ i ] ) / ts;
    }


    // ************************************************************ //
    // ********************* CONTROLLER START ********************* //
    // ************************************************************ //

    start = std::chrono::steady_clock::now( );


    // Get the current H matrices
    H = myLBR->getForwardKinematics( q );
    p_curr = H.block< 3, 1 >( 0, 3 );
    R_curr = H.block< 3, 3 >( 0, 0 );

    // Get the Jacobian Matrices
    // Hybrid Jacobian Matrix (6x7)
    J  = myLBR->getHybridJacobian( q );
    Jp = J.block( 0, 0, 3, myLBR->nq );     // linear  velocity part Jp (3x7)
    Jr = J.block( 3, 0, 3, myLBR->nq );     // angular velocity part Jp (3x7)

    // The difference between the two rotation matrices
    R_del = R_curr.transpose( ) * R0i;

    // Get the current end-effector velocity.
    dp_curr = Jp * dq;

    // Module1 - Joint-space Impedance Control
    tau_imp1 = Bq * ( -dq );

    // Module2 - Task-space Impedance Control, Position
    // Get the virtual trajectory
    p01  = mjt1->getPosition( t );
    dp01 = mjt1->getVelocity( t );
    p02  = mjt2->getPosition( t );
    dp02 = mjt2->getVelocity( t );

    p0  =  p01 +  p02;
    dp0 = dp01 + dp02;
    \
    tau_imp2 = Jp.transpose( ) * ( Kp * ( p0 - p_curr ) + Bp * ( dp0 - dp_curr ) );
//        tau_imp2 = Jp.transpose( ) * ( Kp * ( p0i - p_curr ) + Bp * ( - dp_curr ) );

    // Module3 - Task-space Impedance Control, Orientation
    // Get the Axis Angle of Rotation
    theta = acos( ( R_del.trace( ) - 1 )/2 );

    if( theta >= 0.01 )
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
\
    tau_imp3 = Jr.transpose( ) * ( kr * R_curr * w_axis * theta - br * Jr * dq );

    // Superposition of Mechanical Impedances
    tau_ctrl = tau_imp1 + tau_imp2 + tau_imp3;

    // Write the q Values For regenerating the simulation
    f << "Time: " << std::fixed << std::setw( 5 ) << t;
    f << "   q values: " <<   q.transpose( ).format( fmt );
    f << "  p0 values: " <<  p0.transpose( ).format( fmt );
    f << " dp0 values: " << dp0.transpose( ).format( fmt ) << std::endl;

    end = std::chrono::steady_clock::now( );

    //    std::cout << "Elapsed time for The Torque Calculation "
    //              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
    //              << " us" << std::endl;

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

    // If previous movement is done, then change the movements
    // Reset is also required.
    // Develop for later
    if( is_continue )
    {
        if ( t >= t_max + 1.0)
        {

            // Initialization
            t = 0;
            n_step = 0;

            // Get the current p0 location
            Eigen::Vector3d w_arr;

            w_arr << 0.3, 0.2, 0.0;

            Eigen::Vector3d goal1;
            Eigen::Vector3d goal2;

            int i_tmp = 0;

            goal1 = p0i   + w_arr.cwiseProduct( generateRandom3DVector( ) - 0.5 * Eigen::Vector3d::Ones( ) );
            goal2 = goal1 + w_arr.cwiseProduct( generateRandom3DVector( ) - 0.5 * Eigen::Vector3d::Ones( ) );

            std::cout << " Goal Location: "  << goal1.transpose( ).format( fmt ) << std::endl;
            std::cout << "Final Location: " << goal2.transpose( ).format( fmt ) << std::endl;
            std::cout << "p minimum: "  << pmin.transpose( ).format( fmt ) << std::endl;
            std::cout << "p maximum: "  << pmax.transpose( ).format( fmt ) << std::endl;

            // Clipping the value
            clampVector( goal1, pmin, pmax );
            clampVector( goal2, pmin, pmax );

            std::cout << "Goal  Location (Clipped): " << goal1.transpose( ).format( fmt ) << std::endl;
            std::cout << "Final Location (Clipped): " << goal2.transpose( ).format( fmt ) << std::endl;

            // Reset the initial position and movement
            // Simply the final value is q0i
            mjt1->q0i = p0;
            mjt1->q0f = goal1;
            mjt2->q0f = goal2 - goal1;
        }

    }

}

