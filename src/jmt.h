#ifndef JMT_H
#define JMT_H

#include <math.h>
#include <string>
#include <vector>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Jerk Minimal Trajectory
class JMT
{
private:
    vector <double> coeffs;

public:
    void set_coeffs(const vector<double>& start,
                    const vector<double>& end, double T);
    double position(double x) const;
    double velocity(double x) const;
    double acceleration(double x) const;

};

#endif //JMT_H