#pragma once
#include <array>
#include <list>
#include <string>
#include <Eigen/Dense>


#define BUFFER_SIZE_1 33

class Logger
{
public:
    Logger(const std::string fileName);
    ~Logger();

    void writeToBuffer(const double t,
        const Eigen::Vector<double, 7>& q,
        const Eigen::Vector<double, 7>& qp,
        const Eigen::Vector<double, 3>& xd,
        const Eigen::Vector<double, 6>& xpd,
        const Eigen::Vector<double, 3>& x,
        const Eigen::Vector<double, 6>& xp,
        const Eigen::Matrix<double, 3, 3>& R
    );

private:
    std::string outputFileName;
    std::list<std::array<double, BUFFER_SIZE_1> > buffer;

    const std::string tags{ "t,q1,q2,q3,q4,q5,q6,q7,qp1,qp2,qp3,qp4,qp5,qp6,qp7,xd,yd,zd,vxd,vyd,vzd,wxd,wyd,wzd,x,y,z,vx,vy,vz,wx,wy,wz" };
};