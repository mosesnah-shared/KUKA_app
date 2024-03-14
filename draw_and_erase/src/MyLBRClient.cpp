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

// For reading a CSV file
// [REF] https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
// [REF] Chat gpt also works well
Eigen::MatrixXd csv_to_mat( const std::string & path )
{
    std::ifstream csvFile( path );
    if (!csvFile.is_open())
    {
        std::cerr << "Error: Failed to open CSV file." << std::endl;
        return Eigen::MatrixXd( );
    }

    // Read CSV data and convert to Eigen matrix
    Eigen::MatrixXd eigenMatrix;
    std::vector<std::vector<double>> data;

    std::string line;
    while ( std::getline( csvFile, line ) )
    {
        std::vector<double> row;
        std::stringstream lineStream( line );
        std::string cell;

        while ( std::getline( lineStream, cell, ',' ) )
        {
            row.push_back( std::stod( cell ) );
        }

        data.push_back( row );
    }

    // Convert data to Eigen matrix
    int rows = data.size( );
    int cols = data[0].size( );
    eigenMatrix.resize( rows, cols );

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            eigenMatrix( i, j ) = data[ i ][ j ];
        }
    }

    csvFile.close();

    return eigenMatrix;

}


//******************************************************************************
MyLBRClient::MyLBRClient(double freqHz, double amplitude)
{

    /** Initialization */
    // !! WARNING !!
    // THESE JOINT POSITION VALUES MUST BE THE SAME WITH THE JAVA APPLICATION!!
    q_init[0] =   10.03 * M_PI/180;
    q_init[1] =   54.30 * M_PI/180;
    q_init[2] =    0.54 * M_PI/180;
    q_init[3] =  -83.51 * M_PI/180;
    q_init[4] =  -16.14 * M_PI/180;
    q_init[5] =  -45.99 * M_PI/180;
    q_init[6] =  102.20 * M_PI/180;

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


    // Once Initialized, get the initial end-effector position
    // Forward Kinematics and the current position
    H = myLBR->getForwardKinematics( q );
    p_curr = H.block< 3, 1 >( 0, 3 );
    R_curr = H.block< 3, 3 >( 0, 0 );

    tmp_freq = 0;

    // These values will be the initial values
    p0i = p_curr;
    R0i = R_curr;

    // Set the starting positoin
    // Manually discovered with iiwa KUKA
    // The good starting position and orientaiton is given by the angle as follows:

    q_start  = Eigen::VectorXd::Zero( myLBR->nq );
    q_start( 0 ) = -40.38 * M_PI/180;
    q_start( 1 ) =  54.65 * M_PI/180;
    q_start( 2 ) =  49.73 * M_PI/180;
    q_start( 3 ) = -87.16 * M_PI/180;
    q_start( 4 ) = -45.61 * M_PI/180;
    q_start( 5 ) =  45.46 * M_PI/180;
    q_start( 6 ) =     80 * M_PI/180;

    H_start = myLBR->getForwardKinematics( q_start );

    p0_start = H_start.block< 3, 1 >( 0, 3 );
    p0_start = p0_start + Eigen::Vector3d( 0.0, -0.1, 0.0 );
    p0_write = Eigen::VectorXd::Zero( 3 );
    R0_start = H_start.block< 3, 3 >( 0, 0 );
    Rmat0 = R0_start;

    delz = Eigen::Vector3d( 0, 0, -0.36 );
    p0_write = p0_start + delz;

    // The first submovement to move to set position
    //                                                    D, ti
    mjt1 = new MinimumJerkTrajectory( 3,  p0i,  p0_start, 4, 4  );
    mjt2 = new MinimumJerkTrajectory( 3,  p0_start,  p0_write, 6, 2  );

    // The current end-effector position
    dp_curr = Eigen::VectorXd::Zero( 3 );

    // Time for submovement
    t_sub = 0;
    t_rot_tmp = 0;

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
    w_axis_tmp = Eigen::Matrix3d::Zero( 3, 3 );
    w_axis     = Eigen::Vector3d::Zero( 3 );

    // For the Task-space impedance control, the Forward Kinematics (H) and Hybrid Jacobian Matrix (JH) is Required
    H  = Eigen::Matrix4d::Zero( 4, 4 );
    J  = Eigen::MatrixXd::Zero( 6, myLBR->nq );
    Jp = Eigen::MatrixXd::Zero( 3, myLBR->nq );
    Jr = Eigen::MatrixXd::Zero( 3, myLBR->nq );

    // The Translational stiffness/damping matrices
    Kp = 400 * Eigen::MatrixXd::Identity( 3, 3 );
    Bp =  40 * Eigen::MatrixXd::Identity( 3, 3 );
    Bq = 1.0 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );

    kr = 50;
    br = 5;

    // Parameters of the oscillatory movement
    r_osc     = 0.02;
    omega_osc = 2 * M_PI;

    // Initial print
    printf( "Exp[licit](c)-cpp-FRI, https://explicit-robotics.github.io \n\n" );
    printf( "Robot '" );
    printf( "%s", myLBR->Name );
    printf( "' initialised. Ready to rumble! \n" );
    printf( "The current script runs a superposition of submovement + oscillation \n" );

    // Open a file
    f.open( "/home/baxterplayground/Documents/kinematic_modularity/data/imitation_learning_results/data1.txt" );
    fmt = Eigen::IOFormat(5, 0, ", ", "\n", "[", "]");

    // Read the Data
    pos_data = csv_to_mat( "/home/baxterplayground/Documents/DMP-MATLAB/data/example6/pos_data.csv" );
    vel_data = csv_to_mat( "/home/baxterplayground/Documents/DMP-MATLAB/data/example6/vel_data.csv" );

    if ( pos_data.rows() == 0 || vel_data.rows( ) == 0 )
    {
        std::cerr << "Error: Matrix creation failed." << std::endl;
    }

    std::cout << "Matrix size: " << pos_data.rows() << " rows x " << pos_data.cols() << " columns" << std::endl;
    std::cout << "Matrix size: " << vel_data.rows() << " rows x " << vel_data.cols() << " columns" << std::endl;


    // Get thenumber of data
    assert( pos_data.cols( ) == vel_data.cols( ) );
    N_imit_pos = pos_data.cols( );
    N_curr_pos = 0;

    // Initialization of flags
    is_erase         = false;
    is_pressed       = false;
    is_all_done      = false;
    is_lift_up       = false;
    is_run_imit_pos  = false;
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

    // Get the current end-effector velocity.
    dp_curr = Jp * dq;

    // Module1 - Joint-space Impedance Control
    tau_imp1 = Bq * ( -dq );

    if( !is_pressed ) // If not pressed
    {
        // Simply move to the p_right position
        p0  = mjt1->getPosition( t );
        dp0 = mjt1->getVelocity( t );

        // The difference between the two rotation matrices
        Rtmp1 = R0i.transpose( ) * R0_start;

        // Module3 - Task-space Impedance Control, Orientation
        // Get the Axis Angle of Rotation
        theta_tmp = acos( ( Rtmp1.trace( ) - 1 )/2 );

        if( theta_tmp >= 0.01 )
        {
            w_axis_tmp = ( Rtmp1 - Rtmp1.transpose( ) ) / ( 2 * sin( theta_tmp ) );
        }
        else
        {
            w_axis_tmp = Eigen::Matrix3d::Zero( 3, 3 );
        }

        // The rodrigues formula
        Rtmp2 = Eigen::Matrix3d::Identity( ) + sin( theta_tmp * t_rot_tmp ) * w_axis_tmp + (1 - cos( theta_tmp * t_rot_tmp ) ) * w_axis_tmp * w_axis_tmp;

        // Printout value

        Rmat0 = R0i * Rtmp2;


        if( t_rot_tmp <= 1 )
        {
            t_rot_tmp += 0.0005;
        }

    } // If pressed
    else
    {
        // If run Imitation Learning
        if( is_run_imit_pos )
        {

            double t_off = 6;
            if( t <= t_off ) //if Contact
            {
                // Simply move to the p_right position
                p0  = mjt2->getPosition( t );
                dp0 = mjt2->getVelocity( t );

            }
            else
            {
                p0  = p0_write + pos_data.col( N_curr_pos );
                p0( 2 ) = p0_write( 2 );
                dp0 = vel_data.col( N_curr_pos );

                if( N_curr_pos < N_imit_pos - 1)
                {
                    N_curr_pos++;
                }
                else
                {
                    is_lift_up = true;
                    is_run_imit_pos = false;

                    // Set minimum jerk traejctory
                    mjt3 = new MinimumJerkTrajectory( 3,  p0,  p0 - delz, 4, 2  );
                    mjt4 = new MinimumJerkTrajectory( 3,  Eigen::Vector3d::Zero( ),  delz, 4, 10  );

                    // reset
                    t = 0;
                    n_step = 0;
                }


                // Write the q Values For regenerating the simulation
                f << "Time: " << std::fixed << std::setw( 5 ) << t;
                f << "   q values: " <<   q.transpose( ).format( fmt );
                f << "  p0 values: " <<  p0.transpose( ).format( fmt ) << std::endl;
            }
        }

        if( is_lift_up )
        {
            // Simply move to the p_right position
            p0  = mjt3->getPosition( t ) + mjt4->getPosition( t );
            dp0 = mjt3->getVelocity( t ) + mjt4->getVelocity( t );

            if( t >= 17 )
            {
                is_lift_up = false;
                is_erase = true;

                t = 0;
                n_step = 0;

                Kp( 0, 0 ) = 800;
                Kp( 1, 1 ) = 800;
            }

        }


        if( is_erase )
        {
            if( t >= 0 )
            {
                p0  = p0_write + pos_data.col( N_curr_pos );
                p0( 2 ) = p0_write( 2 );
                dp0 = vel_data.col( N_curr_pos );

                tmp_freq++;

                if( N_curr_pos >= 1 && ( tmp_freq % 4 ) == 0 )
                {
                    N_curr_pos--;
                }

                // Superimpose an oscillation
                // Add oscillation
                p0  += r_osc * Eigen::Vector3d( cos( omega_osc * t ), sin( omega_osc * t ), 0  );
                dp0 += r_osc * omega_osc * Eigen::Vector3d( -sin( omega_osc * t ), cos( omega_osc * t ), 0  );

            }

            // Write the q Values For regenerating the simulation
            f << "Time: " << std::fixed << std::setw( 5 ) << t;
            f << "   q values: " <<   q.transpose( ).format( fmt );
            f << "  p0 values: " <<  p0.transpose( ).format( fmt ) << std::endl;

        }


    }

    // Module2 - Task-space Impedance Control
    tau_imp2 = Jp.transpose( ) * ( Kp * ( p0 - p_curr ) + Bp * ( dp0 - dp_curr ) );


    // The difference between the two rotation matrices
    R_del = R_curr.transpose( ) * Rmat0;


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

    end = std::chrono::steady_clock::now( );

    std::cout << "Elapsed time for The Torque Calculation "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
              << " us" << std::endl;

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

    // Check button pressed
    if ( robotState().getBooleanIOValue( "MediaFlange.UserButton" ) && !is_pressed )
    {
        is_pressed = true;

        // Reset the time and number of steps
        t = 0;
        n_step = 0;

        // Turn on imitation learning
        is_run_imit_pos    = true;
        std::cout << "Button Pressed!" << std::endl;

    }



}

