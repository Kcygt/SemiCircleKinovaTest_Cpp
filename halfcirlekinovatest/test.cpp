#include <iostream>
#include "KinovaGen3.h"
#include "smoothTrajectory.h"
#include "logger.h"
#include <chrono>
#include <string>

#include <fstream> ///// //////
#include <cmath>  ///// //////
#include <vector>  ///// //////


#define BUFFER_SIZE 4001

typedef std::chrono::high_resolution_clock Clock;

constexpr double PI = 3.14159265359;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;


/////////////////////////////
/////////////////////////////
/////////////////////////////
//
void hypodromefn(double s, double radius, double line_length, double& P_z_derivative, double& P_x_derivative, double& P_z, double& P_x) {
    const double M_PI = 3.1415;
    double alpha = (2 * line_length) / (radius * M_PI) + 1;

    if (s <= 1) {
        P_z = -radius * cos(s * M_PI / 2);
        P_x = -radius * sin(s * M_PI / 2);

        P_z_derivative = radius * (M_PI / 2) * sin(s * M_PI / 2);
        P_x_derivative = -radius * (M_PI / 2) * cos(s * M_PI / 2);
    }
    else if (s <= alpha && s > 1) {
        P_z = (s - 1) *  (radius * M_PI) / 2;
        P_x = -radius;

        P_z_derivative = radius * M_PI / 2;
        P_x_derivative = 0;
    }
    else {
        P_z = radius * sin((s - alpha) * M_PI / 2) + line_length;
        P_x = -radius * cos((s - alpha) * M_PI / 2);

        P_z_derivative = radius * M_PI / 2 * cos((s - alpha) * M_PI / 2);
        P_x_derivative = radius * M_PI / 2 * sin((s - alpha) * M_PI / 2);
    }
}

void sFun(double W, double tBlend, double tInstant, double tFinal, double& s, double& sDot) {

    if (tInstant <= tBlend) {
        s = W * pow(tInstant, 2) / 2;
        sDot = W * tInstant;
    }
    else if (tInstant > tBlend && tInstant < tFinal - tBlend) {
        s = W * tBlend * (tInstant - tBlend) + W * pow(tBlend, 2) / 2;
        sDot = W * tBlend;
    }
    else {
        s = W * pow(tBlend, 2) - W * pow((tInstant - tFinal), 2) / 2 - W * tBlend * (2 * tBlend - tFinal);
        sDot = W * (tFinal - tInstant);
    }
}

double brushRotation(const double alpha, double s) {
    double radian;

    if (s <= 1) {
        radian = -3.1415/8 + s * 3.1415/8;
    }
    else if (s > 1 && s <= alpha) {
        radian = 0;
    }
    else {
        radian = 3.1514 / 8 * (s - alpha);
    }

    return radian;
}


/////////////////////////
/////////////////////////
/////////////////////////
double shiftPositionInterval(const double q)
{
    return q < 180.0 ? q : q - 360.0;
}

int main()
{
    std::cout << "Testing Kinova kinematics and trajectory generation" << std::endl;

    KinovaGen3 robot;

    Logger log("data.csv");

    // Initial joint configuration
    const Eigen::Vector<double, 7> q0 = { 0.0,
                                         shiftPositionInterval(296.5) * DEG2RAD,
                                         0.0,
                                         shiftPositionInterval(127.0) * DEG2RAD,
                                         0.0,
                                         shiftPositionInterval(35.0) * DEG2RAD,
                                         0.0 };

    Eigen::Vector<double, 7> q = q0;
    Eigen::Vector<double, 7> qp = Eigen::Vector<double, 7>::Zero();
    Eigen::Vector<double, 7> qpd = Eigen::Vector<double, 7>::Zero();
    Eigen::Vector<double, 3> x = Eigen::Vector<double, 3>::Zero();
    Eigen::Vector<double, 6> xp = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 3> xd = Eigen::Vector<double, 3>::Zero();
    Eigen::Vector<double, 6> xpd = Eigen::Vector<double, 6>::Zero();
    //Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Ones();
    Eigen::Vector<double, 3> P = Eigen::Vector<double, 3>::Zero();
    Eigen::Matrix3d R;
    /*
    R << 0.8660254, 0.5000000, 0.0000000,
        -0.5000000, 0.8660254, 0.0000000,
        0.0000000, 0.0000000, 1.0000000;
        */

    R << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    int i(0);

    // Initial end-effector position
    Eigen::Vector3d xStart;
    Eigen::Matrix3d RStart;
    robot.forwardKinematics(q0, xStart, RStart);
    

    double  P_z_derivative = 0.0;
    double  P_x_derivative = 0.0; 
    double P_z = 0.0; 
    double P_x = 0.0;


    // Final end-effector position
    //Eigen::Vector3d xEnd = xStart + Eigen::Vector3d(0.0, 0.0, 0.15);

    //// Create the desired trajectory
    //SmoothTrajectory tr(0.0, 1.0, xStart(2), xEnd(2));

    // Desired time-step for the control loop
    const std::chrono::duration<int, std::milli> tStep(1);

    // Actual time step for the control loop    
    std::chrono::duration<double, std::milli> h(0);

    // Elapsed time from the beginning of the loop
    std::chrono::duration<double, std::milli> tElapsed(0.0);

    // Current timestamp
    auto t = Clock::now();

    // Start time of the loop
    auto tLoopStart = t;

    // Start time of an iteration of the loop
    auto tIterationStart = t;

    // Start time of the previous iteration of the loop
    auto tPrevIterationStart = t;

    double radius = 0.08;
    double line_length = 0.1;
    

    double alpha = (2 * line_length) / (radius * 3.1415) + 1;
    
    double s = 0.0;
    double sDot = 0.0;
    double tBlend = 1.0 ;
    double tFinal = 4.0;
    double W =  (alpha + 1) / (tBlend * (tFinal - tBlend));
    

    while (tElapsed.count() < BUFFER_SIZE - 1)
    {
        tIterationStart = Clock::now();
        tElapsed = tIterationStart - tLoopStart;

        hypodromefn(s, radius, line_length, P_z_derivative, P_x_derivative, P_z, P_x);
        sFun(W, tBlend, tElapsed.count() * 1e-3, tFinal, s, sDot);
        P << P_x_derivative, 0.0, P_z_derivative;
        P = P.transpose() * R;
        xpd(2) = P(2) * sDot;
        xpd(0) = P(0) * sDot;
        /*if (tElapsed.count()/1000 < 1.0) {
            q(5) = q(5) - 2e-4;
            std::cout << tElapsed.count() / 1000<< std::endl;
        }*/


        // Solve inverse kinematics
        robot.inverseKinematics(q, xpd, qp);

        // Calculate the actual time-step
        h = tIterationStart - tPrevIterationStart;
        
        
        q += qp * static_cast<double>(h.count()) * 1e-3;
        robot.forwardKinematics(q, x, R);
        robot.jacobian(q, J);
        xp = J * qpd;

        // Write simulation data to buffer
        log.writeToBuffer(static_cast<double>(tElapsed.count()) * 1e-3, q, qp, xd, xpd, x, xp, R);

        while (t - tIterationStart < tStep)
            t = Clock::now();

        tPrevIterationStart = tIterationStart;

        i++;
    }
    
    return 0;
}