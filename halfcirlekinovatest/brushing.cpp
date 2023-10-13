#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#include <list>

#include "KinovaGen3.h"
#include "FTDriver.h"

void writeListToCSV(const std::list<Eigen::VectorXd>&, const std::string&);
void writeListToCSV(const std::list<double>&, const std::string&);

namespace k_api = Kinova::Api;

typedef std::chrono::high_resolution_clock Clock;

//long long TIME_DURATION = 4000000000;


// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{ 20 };

//#include "KinovaGen3.h"
//#include <Eigen/Dense>


constexpr double PI{ 3.14159265359 };
constexpr double DEG2RAD{ PI / 180.0 };
constexpr double RAD2DEG{ 180.0 / PI };

/*
void hypodromefn(double s, double radius, double line_length, double& P_z_derivative, double& P_x_derivative, double& P_z, double& P_x) {
    double M_PI = 3.1415;
    double alpha = (2 * line_length) / (radius * M_PI) + 1;

    if (s <= 1) {
        P_z = -radius * cos(s * M_PI / 2);
        P_x = -radius * sin(s * M_PI / 2);

        P_z_derivative = radius * (M_PI / 2) * sin(s * M_PI / 2);
        P_x_derivative = -radius * (M_PI / 2) * cos(s * M_PI / 2);
    }
    else if (s <= alpha && s > 1) {
        P_z = (s - 1) / (alpha - 1) * line_length;
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

double sddot(double W, double tBlend, double tInstant, double tFinal) {
    double sDot;

    if (tInstant <= tBlend)
        sDot = W * tInstant;
    else if (tInstant > tBlend && tInstant < tFinal - tBlend)
        sDot = W * tBlend;
    else
        sDot = W * (tFinal - tInstant);

    return sDot;
}
*/

void hypodromefn(double s, double radius, double line_length, double& P_x_derivative, double& P_y_derivative, double& P_x, double& P_y) {
    const double M_PI = 3.1415;
    double alpha = (2 * line_length) / (radius * M_PI) + 1;

    if (s <= 1) {
        P_x = -radius * cos(s * M_PI / 2);
        P_y = -radius * sin(s * M_PI / 2);

        P_x_derivative = radius * (M_PI / 2) * sin(s * M_PI / 2);
        P_y_derivative = -radius * (M_PI / 2) * cos(s * M_PI / 2);
    }
    else if (s <= alpha && s > 1) {
        P_x = (s - 1) * (radius * M_PI) / 2;
        P_y = -radius;

        P_x_derivative = radius * M_PI / 2;
        P_y_derivative = 0;
    }
    else {
        P_x = radius * sin((s - alpha) * M_PI / 2) + line_length;
        P_y = -radius * cos((s - alpha) * M_PI / 2);

        P_x_derivative = radius * M_PI / 2 * cos((s - alpha) * M_PI / 2);
        P_y_derivative = radius * M_PI / 2 * sin((s - alpha) * M_PI / 2);
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

void rotation(double radian, Eigen::Matrix3d& Rx, Eigen::Matrix3d& Ry, Eigen::Matrix3d& Rz) {
    Rx << 1, 0, 0,
        0, cos(radian), -sin(radian),
        0, sin(radian), cos(radian);

    Ry << cos(radian), 0, sin(radian),
        0, 1, 0,
        -sin(radian), 0, cos(radian);

    Rz << cos(radian), -sin(radian), 0,
        sin(radian), cos(radian), 0,
        0, 0, 1;
}

template<Eigen::size_t N>
void writeListToCSV(const std::list<Eigen::Vector<double, N> >& dataList, const std::string& filename) {
    std::ofstream outputFile(filename);

    if (!outputFile) {
        std::cerr << "Error o2pening file: " << filename << std::endl;
        return;
    }

    for (const auto& data : dataList) {
        for (Eigen::Index i = 0; i < N; ++i) {
            outputFile << data(i);
            if (i != N - 1)
                outputFile << ",";
        }
        outputFile << std::endl;
    }

    outputFile.close();
    std::cout << "Data written to " << filename << " successfully." << std::endl;
}

void writeListToCSV(const std::list<double>& dataList, const std::string& filename) {
    std::ofstream outputFile(filename);

    if (!outputFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (const auto& data : dataList) {
        outputFile << data << std::endl;
    }

    outputFile.close();
    std::cout << "Data written to " << filename << " successfully." << std::endl;
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

double shiftPositionInterval(const double q)
{
    return q < 180.0 ? q : q - 360.0;
}

bool torqueControl(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    //
    // KINOVA GEN3
    //
    KinovaGen3 robot;

    //
    // Force/torque sensor
    //
    FTDriver forceSensor;

    // Joint positions and velocities
    Eigen::Vector<double, 7> q; // joint positions (wsh)
    Eigen::Vector<double, 7> qp; // joint velocities (wsh)
    Eigen::Vector<double, 7> qd; // joint position demands (wsh)
    // Eigen::Vector<double, 7> qdPrev = Eigen::Vector<double, 7>::Zero();
    Eigen::Vector<double, 7> qpd = Eigen::Vector<double, 7>::Zero();  // joint velocity demands (wsh)

    // Initial joint configuration
    /*const Eigen::Vector<double, 7> q0 = {0.0,
                                         shiftPositionInterval(296.5) * DEG2RAD,
                                         0.0,
                                         shiftPositionInterval(127.0) * DEG2RAD,
                                         0.0,
                                         shiftPositionInterval(35.0) * DEG2RAD,
                                         0.0 };*/
    const Eigen::Vector<double, 7> q0 = { 0.0,
                                         shiftPositionInterval(270.0) * DEG2RAD,
                                         0.0,
                                         shiftPositionInterval(138.0) * DEG2RAD,
                                         0.0,
                                         shiftPositionInterval(50.0) * DEG2RAD,
                                         0.0 }; // Initial joint position (wsh)


    qd = q0;

    // End-effector pose in homogenous transformation
    Eigen::Transform<double, 3, Eigen::Affine> T;

    // Actual end-effector position
    Eigen::Vector<double, 3> x;

    // Actual end-effector velocity
    Eigen::Vector<double, 6> xp;

    // Actual end-effector rotation
    Eigen::Matrix<double, 3, 3> R;

    // Jacobian of the manipulator
    Eigen::Matrix<double, 6, 7> J;

    // Wrench
    Eigen::Vector<double, 6> wrench = Eigen::Vector<double, 6>::Zero();

  

    // Control output (torque)
    Eigen::Vector<double, 7> u;

    // Input torque to the robot
    Eigen::Vector<double, 7> tau;

    // Position and velocity error
    Eigen::Vector<double, 7> e;
    Eigen::Vector<double, 7> ep;

    // Gravity term
    Eigen::Vector<double, 7> G;

    // Coriolis term
    Eigen::Vector<double, 7> C;

    // Initial end-effector position
    Eigen::Vector3d xStart;
    Eigen::Matrix3d RStart;
    T = robot.forwardKinematics(q0);
    xStart = T.translation();
    RStart = T.rotation();


    // Initial the function parameters
    double P_z_derivative = 0.0;
    double P_x_derivative = 0.0;
    double P_z = 0.0;
    double P_x = 0.0;


    // Final end-effector position
    Eigen::Vector3d xEnd = xStart + Eigen::Vector3d(0.0, 0.0, 0.15);

    Eigen::Vector3d xd = xStart;
    Eigen::Vector<double, 6> xpd = Eigen::Vector<double, 6>::Zero();

    std::list<double> log_t;
    std::list<Eigen::Vector<double, 3>> log_xd;
    std::list<Eigen::Vector<double, 6>> log_xpd;
    std::list<Eigen::Vector<double, 7>> log_qd;
    std::list<Eigen::Vector<double, 7>> log_qpd;
    std::list<Eigen::Vector<double, 7>> log_q;
    std::list<Eigen::Vector<double, 7>> log_qp;
    std::list<Eigen::Vector<double, 6>> log_f;

    std::string fileName;

    //
    // SIMULATION TIME
    //

    // Duration of the control loop
    const std::chrono::duration<long long, std::nano> tDuration(3000000000);

    bool return_status = true;

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch (...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }


    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set all actuators to torque mode
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        for (unsigned int i = 1; i <= actuator_count; ++i)
            actuator_config->SetControlMode(control_mode_message, i);

        //std::cout << "Running torque control example for " << TIME_DURATION << " seconds" << std::endl;

        // Desired time-step for the control loops
        // Fast loop: torque control
        const std::chrono::duration<long long, std::nano> hFastLoop(1000000);

        // Slow loop: trajectory deformation
        const std::chrono::duration<long long, std::nano> hSlowLoop(10000000);

        const unsigned int sampleTimeRatio = static_cast<unsigned int>(hSlowLoop / hFastLoop);

        // Indices for fast and slow loops in the control
        unsigned int fastLoopIndex{ 0 };
        unsigned int slowLoopIndex{ 0 };

        // Actual time step for the control loop    
        std::chrono::duration<long long, std::nano> h(0);

        // Elapsed time from the beginning of the loop
        std::chrono::duration<long long, std::nano> tElapsed(0);

        //Parameters 
        
        double radius = 0.055;
        double line_length = 0.20;

        double alpha = (2 * line_length) / (radius * 3.1415) + 1;

        double s = 0.0;
        double sDot = 0.0;
        double tBlend = 1.0;
        double tFinal = 3.0;
        double W = (alpha + 1) / (tBlend * (tFinal - tBlend));
        int iterationNumber = 3;
        
        fileName = "Flat_R" + to_string(radius) + "_T" + to_string(tFinal) + "_" + to_string(iterationNumber) + "_";
        
        // PD control parameters
        const double Kp{ 3400.0 };
        const double Kd{ 50.0 };
        
        // Current timestamp
        auto t = Clock::now();

        // Start time of the loop
        auto tLoopStart = t;
        //std::cout << "Current timestamp: " << std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch()).count() << " nanoseconds since epoch." << std::endl;
        // Start time of an iteration of the loop
        auto tIterationStart = t;

        // Start time of the previous iteration of the loop
        auto tPrevIterationStart = t;
        
        Eigen::Vector<double, 3> Pp = Eigen::Vector<double, 3>::Zero();
        double radian = 0.0;
        //double radian = 3.1415 / 5.6;
        Eigen::Matrix3d Rx, Ry, Rz;
        rotation(radian, Rx, Ry, Rz);
        // 
        // 
        
        // Real-time loop
        while (tElapsed < tDuration)
        {
            // Update the start time of the iteration
            tIterationStart = Clock::now();

            // Calculate the elapsed time from the loop start
            tElapsed = tIterationStart - tLoopStart;

            // Calculate the actual time-step
            h = tIterationStart - tPrevIterationStart;

            // Position command to first actuator is set to measured one to avoid following error to trigger
            // Bonus: When doing this instead of disabling the following error, if communication is lost and first
            //        actuator continues to move under torque command, resulting position error with command will
            //        trigger a following error and switch back the actuator in position command to hold its position
            for (unsigned int i = 0; i < actuator_count; ++i)
            {
                base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                q(i) = shiftPositionInterval(base_feedback.actuators(i).position()) * DEG2RAD;
                qp(i) = base_feedback.actuators(i).velocity() * DEG2RAD;
            }

            log_q.push_back(q);
            log_qp.push_back(qp);

            // Calculate the point on the desired trajectory
            //xd = trajectory.jointTrajectoryPos(tElapsed.count() * 1e-9
            //xpd(Eigen::seq(0, 2)) = trajectory.jointTrajectoryVel(tElapsed.count() * 1e-9);

            sFun(W, tBlend, tElapsed.count() * 1e-9, tFinal, s, sDot);
            hypodromefn(s, radius, line_length, P_z_derivative, P_x_derivative, P_z, P_x);
            Pp << P_x_derivative, 0.0, P_z_derivative;
            Pp = Ry * Pp ;
            xpd(2) = Pp(2) * sDot;
            xpd(0) = -Pp(0) * sDot;
            /*xpd(2) = P_z_derivative * sDot;
            xpd(0) = -P_x_derivative * sDot;*/
            

            // Solve inverse kinematics via Jacobian
            qpd = robot.inverseKinematics(qd, xpd);

            // Integrate the desired joint velocity
            // TODO Calculate the exact duration of a single iteration and use it in the integration
            qd += qpd * static_cast<double>(h.count()) * 1e-9;

            // Read force/torque sensor
            wrench = forceSensor.readSensor();

            // Log data
            log_t.push_back(tElapsed.count() * 1e-9);
            log_xd.push_back(xd);
            log_xpd.push_back(xpd);
            log_qd.push_back(qd);
            log_qpd.push_back(qpd);
            log_f.push_back(wrench);
            // Calculate the gravity term
            G = robot.gravity(q);

            // Calculate the Coriolis term
            C = robot.coriolis(q, qp);

            // Calculate error
            e = qd - q;
            ep = qpd - qp;

            // PD control
            u = Kp * e + Kd * ep;
            //u = Kp * (q0 - q);

            // Torque to the actuators
            tau = u + G + C;

            T = robot.forwardKinematics(q);
            x = T.translation();
            R = T.rotation();

            J = robot.jacobian(q);
            xp = J * qpd;

            for (unsigned int i = 0; i < actuator_count; ++i)
                base_command.mutable_actuators(i)->set_torque_joint(static_cast<float>(tau(i)));

            // Slow loop
            if (slowLoopIndex == fastLoopIndex)
            {
                slowLoopIndex += sampleTimeRatio;
            }
            fastLoopIndex++;

            // Incrementing identifier ensures actuators can reject out of time frames
            base_command.set_frame_id(base_command.frame_id() + 1);
            if (base_command.frame_id() > 65535)
                base_command.set_frame_id(0);

            for (unsigned int idx = 0; idx < actuator_count; idx++)
            {
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
            }

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            }
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex exception: " << ex.what() << std::endl;

                std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            }
            catch (std::runtime_error& ex2)
            {
                std::cout << "runtime error: " << ex2.what() << std::endl;
            }
            catch (...)
            {
                std::cout << "Unknown error." << std::endl;
            }

            // Hold the iteratation for hFastLoop seconds in total
            while (t - tIterationStart < hFastLoop)
                t = Clock::now();

            // Update the start time of the previous iteration of the loop
            tPrevIterationStart = tIterationStart;
        }

        std::cout << "Torque control example completed" << std::endl;

        // Set all actuators to position mode
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (unsigned int i = 1; i <= actuator_count; ++i)
            actuator_config->SetControlMode(control_mode_message, i);

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    
    // Save data to file
    writeListToCSV(log_t, fileName + "simulationTime.csv");
    writeListToCSV<3>(log_xd, fileName + "desiredEndEffectorPosition.csv");
    writeListToCSV<6>(log_xpd, fileName + "desiredEndEffectorVelocity.csv");
    writeListToCSV<7>(log_qd, fileName + "desiredJointPosition.csv");
    writeListToCSV<7>(log_qpd, fileName + "desiredJointVelocity.csv");
    writeListToCSV<7>(log_q, fileName + "jointPosition.csv");
    writeListToCSV<7>(log_qp, fileName + "jointVelocity.csv");
    writeListToCSV<6>(log_f, fileName + "wrench.csv");



    return return_status;
}

int main(int argc, char** argv)
{
    // IP address and port number of the robot
    const std::string IP("192.168.1.10");
    const int PORT(10000);
    const int PORT_REAL_TIME(10001);

    // Create API objects
    auto error_callback = [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); };

    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Example core
    bool success = true;
    success &= torqueControl(base, base_cyclic, actuator_config);
    if (!success)
    {
        std::cout << "There has been an unexpected error." << endl;
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return success ? 0 : 1;
}