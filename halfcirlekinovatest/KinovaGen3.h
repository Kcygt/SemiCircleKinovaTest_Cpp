#include <Eigen/Dense>

class KinovaGen3
{
public:
    KinovaGen3();
    ~KinovaGen3();

    void forwardKinematics(const Eigen::Vector<double, 7> q,
        Eigen::Vector<double, 3>& x,
        Eigen::Matrix<double, 3, 3>& R);

    void inverseKinematics(const Eigen::Vector<double, 7> q,
        const Eigen::Vector<double, 3> x,
        const Eigen::Vector<double, 6> xp,
        Eigen::Vector<double, 7>& qp);

    void jacobian(const Eigen::Vector<double, 7> q,
        Eigen::Matrix<double, 6, 7>& J);

    void massMatrix(const Eigen::Vector<double, 7> q,
        Eigen::Matrix<double, 7, 7>& M);

    void coriolis(const Eigen::Vector<double, 7> q,
        const Eigen::Vector<double, 7> qp,
        Eigen::Vector<double, 7>& C);

    void gravity(const Eigen::Vector<double, 7> q,
        Eigen::Vector<double, 7>& G);

};