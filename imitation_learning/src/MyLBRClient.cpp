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
#include <string.h>
#include <math.h>
#include <chrono>
#include <iomanip>
#include <vector>


#include "MyLBRClient.h"
#include "exp_robots.h"

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef NCoef
#define NCoef 1
#endif

static double filterOutput[7][NCoef+1]; //output samples. Static variables are initialised to 0 by default.
static double filterInput[7][NCoef+1]; //input samples. Static variables are initialised to 0 by default.


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
MyLBRClient::MyLBRClient(double freqHz, double amplitude){

    /** Initialization */

    // THIS CONFIGURATION MUST BE THE SAME AS FOR THE JAVA APPLICATION!!
    qInitial[0] =   0.00  * M_PI/180;
    qInitial[1] =  28.56  * M_PI/180;
    qInitial[2] =  17.54  * M_PI/180;
    qInitial[3] = -87.36  * M_PI/180;
    qInitial[4] =  -7.82  * M_PI/180;
    qInitial[5] =  75.56  * M_PI/180;
    qInitial[6] =  -9.01  * M_PI/180;

    // Use Explicit-cpp to create your robot
    myLBR = new iiwa14( 1, "Dwight");
    myLBR->init( );

    // Current position and velocitz
    q  = Eigen::VectorXd::Zero( myLBR->nq );
    dq = Eigen::VectorXd::Zero( myLBR->nq );

    // Time variables for control loop
    currentTime = 0;
    sampleTime  = 0;

    // Initialize joint torques and joint positions (also needed for waitForCommand()!)
    for( int i=0; i < myLBR->nq; i++ )
    {
        qCurr[i] = qInitial[i];
        qOld[i] = qInitial[i];
        qApplied[i] = 0.0;
        torques[i] = 0.0;

        q[ i ] = qInitial[i];
    }

    tau_imp1      = Eigen::VectorXd::Zero( myLBR->nq );
    tau_imp2      = Eigen::VectorXd::Zero( myLBR->nq );
    tau_imp3      = Eigen::VectorXd::Zero( myLBR->nq );

    tau_current   = Eigen::VectorXd::Zero( myLBR->nq );
    tau_previous  = Eigen::VectorXd::Zero( myLBR->nq );
    tau_prev_prev = Eigen::VectorXd::Zero( myLBR->nq );
    tau_total     = Eigen::VectorXd::Zero( myLBR->nq );

    // ************************************************************
    // INITIALIZE YOUR VECTORS AND MATRICES HERE
    // ************************************************************
    H  = Eigen::Matrix4d::Zero( 4, 4 );
    J  = Eigen::MatrixXd::Zero( 6, myLBR->nq );
    Jp = Eigen::MatrixXd::Zero( 3, myLBR->nq );
    Jr = Eigen::MatrixXd::Zero( 3, myLBR->nq );

    // The stiffness/damping matrices
    Kp = 800 * Eigen::MatrixXd::Identity( 3, 3 );
    Bp =  80 * Eigen::MatrixXd::Identity( 3, 3 );
    Bq = 1.0 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );

    // Open a file
    f.open( "tmp.txt" );
    fmt = Eigen::IOFormat(5, 0, ", ", "\n", "[", "]");

    // Initial print
    printf( "Exp[licit](c)-cpp-FRI, https://explicit-robotics.github.io \n\n" );
    printf( "Robot '" );
    printf( "%s", myLBR->Name );
    printf( "' initialised. Ready to rumble! \n\n" );

    // Read a csv file for position
    // Copy paste the link
    pos_data = csv_to_mat( "/home/baxterplayground/Documents/DMP-MATLAB/data/example4/pos_data.csv" );
    vel_data = csv_to_mat( "/home/baxterplayground/Documents/DMP-MATLAB/data/example4/vel_data.csv" );

    if ( pos_data.rows() == 0 || vel_data.rows( ) == 0 )
    {
        std::cerr << "Error: Matrix creation failed." << std::endl;
    }

    std::cout << "Matrix size: " << pos_data.rows() << " rows x " << pos_data.cols() << " columns" << std::endl;
    std::cout << "Matrix size: " << vel_data.rows() << " rows x " << vel_data.cols() << " columns" << std::endl;


    // Once Initialized, get the initial end-effector position
    // Forward Kinematics and the current position
    H = myLBR->getForwardKinematics( q );
    p_curr  = H.block< 3, 1 >( 0, 3 );
    R_curr  = H.block< 3, 3 >( 0, 0 );
    dp_curr = Eigen::VectorXd::Zero( 3 );

    p0i    = p_curr;
    R_init = R_curr;

    // Set the desired
    R_des = Eigen::Matrix3d::Zero( );

    // Check is run imitation learning
    is_run_imit_pos     = false;
    is_run_imit_orient  = false;
    is_pressed          = false;

    // Get thenumber of data
    assert( pos_data.cols( ) == vel_data.cols( ) );
    N_imit_pos = pos_data.cols( );
    N_curr_pos = 0;

    // Read a csv file for rotation.
    R_data = csv_to_mat( "/home/baxterplayground/Documents/DMP-MATLAB/data/example3/orientation_del_data.csv" );
    std::cout << "Matrix size: " << R_data.rows() << " rows x " << R_data.cols() << " columns" << std::endl;

    // Number of data points, orientation
    N_imit_orient = R_data.cols( )/3;
    N_curr_orient = 0 ;

    std::cout << "Data Length for Position: " << N_imit_pos    << std::endl;
    std::cout << "Data Length for Orientation: " << N_imit_orient << std::endl;
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
void iir(double NewSample[7])
{
    double ACoef[ NCoef+1 ] = {
        0.05921059165970496400,
        0.05921059165970496400
    };

    double BCoef[ NCoef+1 ] = {
        1.00000000000000000000,
        -0.88161859236318907000
    };

    int n;

    // Shift the old samples
    for ( int i=0; i<7; i++ )
    {
        for( n=NCoef; n>0; n-- )
        {
            filterInput[i][n] = filterInput[i][n-1];
            filterOutput[i][n] = filterOutput[i][n-1];
        }
    }

    // Calculate the new output
    for (int i=0; i<7; i++)
    {
        filterInput[i][0] = NewSample[i];
        filterOutput[i][0] = ACoef[0] * filterInput[i][0];
    }

    for (int i=0; i<7; i++)
    {
        for(n=1; n<=NCoef; n++)
            filterOutput[i][0] += ACoef[n] * filterInput[i][n] - BCoef[n] * filterOutput[i][n];
    }
}

//******************************************************************************
void MyLBRClient::onStateChange(ESessionState oldState, ESessionState newState)
{
    LBRClient::onStateChange(oldState, newState);
    // react on state change events
    switch (newState)
    {
    case MONITORING_WAIT:
    {
        break;
    }
    case MONITORING_READY:
    {
        sampleTime = robotState().getSampleTime();
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
    robotCommand().setJointPosition(robotState().getCommandedJointPosition());

    // Copy measured joint positions (radians) to _qcurr, which is a double

    memcpy( qCurr, robotState().getMeasuredJointPosition(), 7*sizeof(double) );

    // Initialise the q for the previous NCoef timesteps

    for( int i=0; i<NCoef+1; i++ )
    {
        iir(qCurr);
    }
}

//******************************************************************************
void MyLBRClient::waitForCommand()
{
    // If we want to command torques, we have to command them all the time; even in
    // waitForCommand(). This has to be done due to consistency checks. In this state it is
    // only necessary, that some torque vlaues are sent. The LBR does not take the
    // specific value into account.

    if(robotState().getClientCommandMode() == TORQUE){

        robotCommand().setTorque(torques);
        robotCommand().setJointPosition(robotState().getIpoJointPosition());            // Just overlaying same position
    }

}

//******************************************************************************
void MyLBRClient::command()
{

    // ************************************************************
    // Get robot measurements

    memcpy( qOld, qCurr, 7*sizeof(double) );
    memcpy( qCurr, robotState().getMeasuredJointPosition(), 7*sizeof(double) );

    for (int i=0; i < myLBR->nq; i++)
    {
        q[i] = qCurr[i];
    }

    for (int i=0; i < 7; i++)
    {
        dq[i] = (qCurr[i] - qOld[i]) / sampleTime;
    }

    // ************************************************************
    // Calculate kinematics and dynamics

    //if(currentTime < sampleTime)
    //{
    // To get initial values for first time you are in the control loop
    //}

    // Print Out the Joint Values
    if ( robotState( ).getBooleanIOValue( "MediaFlange.UserButton" ) && is_pressed == false )
    {
        // Turn on/off
        is_run_imit_pos    = true;
        is_run_imit_orient = true;

        // Get the position of current
        H = myLBR->getForwardKinematics( q );
        p0i = H.block< 3, 1 >( 0, 3 );
        R_init = H.block< 3, 3 >( 0, 0 );

        std::cout << "Button Presseed!" << std::endl;

        is_pressed = true;

    }

    H = myLBR->getForwardKinematics( q );
    p_curr = H.block< 3, 1 >( 0, 3 );
    R_curr = H.block< 3, 3 >( 0, 0 );

    // Get the current end-effector velocity
    // Hybrid Jacobian Matrix (6x7) and its linear velocity part (3x7)
    J  = myLBR->getHybridJacobian( q );
    Jp = J.block( 0, 0, 3, myLBR->nq );
    Jr = J.block( 3, 0, 3, myLBR->nq );

    // Calculate the current end-effector's position
    dp_curr = Jp * dq;

    // ************************************************************
    // YOUR CODE ENDS HERE!
    // ************************************************************
    // If button is pressed.
    // Calculate the tau
    //    tau_imp1 = Jp.transpose( ) * ( Kp * ( p0 - p_curr ) + Bp * ( dp0 - dp_curr ) ) + Bq * ( -dq );


    if( is_run_imit_pos )
    {
        p0  = p0i + pos_data.col( N_curr_pos );
        dp0 = vel_data.col( N_curr_pos );

//        tau_imp1 = Jp.transpose( ) * ( Kp * ( p0i - p_curr ) + Bp * ( - dp_curr ) ) + Bq * ( -dq );
        tau_imp1 = Jp.transpose( ) * ( Kp * ( p0 - p_curr ) + Bp * ( dp0 - dp_curr ) ) + Bq * ( -dq );

        if( N_curr_pos < N_imit_pos - 1)
        {
            N_curr_pos++;
        }
    }
    else
    {
        tau_imp1 = Eigen::VectorXd::Zero( myLBR->nq );
    }

    if( is_run_imit_orient )
    {
        // The difference between the two rotation matrices
        // Maintain R_init Posture

        // Get the desired orientation from data
        R_des = R_init * R_data.block< 3, 3 >( 0, 3*( N_curr_orient ) );

//        R_des = R_init;
        R_del = R_curr.transpose( ) * R_des;


        if( abs( R_del.trace( ) - 3 ) >= 0.01 )
        {
            theta = acos( ( R_del.trace( ) - 1 )/2 );
            w_axis_mat = ( R_del - R_del.transpose( ) ) / ( 2 * sin( theta ) );
        }
        else
        {
            w_axis_mat = Eigen::Matrix3d::Zero( );
            theta = 0;
        }

        w_axis( 0 ) = -w_axis_mat( 1, 2 );
        w_axis( 1 ) =  w_axis_mat( 0, 2 );
        w_axis( 2 ) = -w_axis_mat( 0, 1 );


        tau_imp2 = Jr.transpose( ) * ( 50 * R_curr * w_axis * theta - 5 * Jr * dq ) + Bq * ( -dq );


        if( N_curr_orient < N_imit_orient - 1)
        {
            N_curr_orient++;
        }

    }
    else
    {
        tau_imp2 = Eigen::VectorXd::Zero( myLBR->nq );
    }

    tau_current = tau_imp1 + tau_imp2 ;

    // Update
    if ( currentTime == 0.0 )
    {
        tau_previous  = tau_current;
        tau_prev_prev = tau_current;
    }

    if( is_pressed )
    {
        f << "Time: " << std::fixed << std::setw( 5 ) << currentTime;
        f << "  q values: "  << q.transpose( ).format( fmt );
        f << "  p values: "  << p_curr.transpose( ).format( fmt );
//        f << "  p0 values: " << p0.transpose( ).format( fmt ) << std::endl;
    }

    tau_total = ( tau_current + tau_previous + tau_prev_prev ) / 3;

    for ( int i=0; i<7; i++ )
    {
        qApplied[i] = filterOutput[i][0];
        torques[i] = tau_total[i];
    }

    // Command values (must be double arrays!)
    if (robotState().getClientCommandMode() == TORQUE)
    {
        robotCommand().setJointPosition(qApplied);
        robotCommand().setTorque(torques);
    }



    // IIR filter input
    iir( qCurr );

    tau_previous  = tau_current;
    tau_prev_prev = tau_previous;

    currentTime = currentTime + sampleTime;


}
