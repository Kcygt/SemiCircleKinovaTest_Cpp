#include <fstream>
#include "logger.h"


Logger::Logger(const std::string fileName)
{
    outputFileName = fileName;
}

Logger::~Logger()
{
    int i(0);

    // Save data to file on exit
    std::ofstream outputFile;

    // Open the output file
    outputFile.open(outputFileName);

    // Write the tags
    outputFile << tags << std::endl;

    // Write the buffer to the file
    for (const auto& v : buffer)
    {
        for (i = 0; i < BUFFER_SIZE_1 - 1; ++i)
            outputFile << v[i] << ",";

        outputFile << v[BUFFER_SIZE_1 - 1] << std::endl;
    }

    // Close the output file
    outputFile.close();
}

void Logger::writeToBuffer(const double t,
    const Eigen::Vector<double, 7>& q,
    const Eigen::Vector<double, 7>& qp,
    const Eigen::Vector<double, 3>& xd,
    const Eigen::Vector<double, 6>& xpd,
    const Eigen::Vector<double, 6>& e_err,
    const Eigen::Vector<double, 6>& x,
    const Eigen::Vector<double, 6>& xp
     

)
{
    std::array<double, BUFFER_SIZE_1> buf;
    //xp,yp,zp,vx,vy,vz,wx,wy,wz    
    // Time
    buf[0] = t;

    // Joint position
    buf[1] = q(0);
    buf[2] = q(1);
    buf[3] = q(2);
    buf[4] = q(3);
    buf[5] = q(4);
    buf[6] = q(5);
    buf[7] = q(6);

    // Joint velocity
    buf[8] = qp(0);
    buf[9] = qp(1);
    buf[10] = qp(2);
    buf[11] = qp(3);
    buf[12] = qp(4);
    buf[13] = qp(5);
    buf[14] = qp(6);

    // Desired end-effector position
    buf[15] = xd(0);
    buf[16] = xd(1);
    buf[17] = xd(2);

    // Desired end-effector velocity
    buf[18] = xpd(0);
    buf[19] = xpd(1);
    buf[20] = xpd(2);
    buf[21] = xpd(3);
    buf[22] = xpd(4);
    buf[23] = xpd(5);

    // End-effector position from forward kinematics
    buf[24] = x(0);
    buf[25] = x(1);
    buf[26] = x(2);

    // End-effector velocity from forward kinematics
    buf[27] = xp(0);
    buf[28] = xp(1);
    buf[29] = xp(2);
    buf[30] = xp(3);
    buf[31] = xp(4);
    buf[32] = xp(5);

    buffer.push_back(buf);
}
