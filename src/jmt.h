#ifndef JMT_H
#define JMT_H

#include <math.h>
#include <string>
#include <vector>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

namespace jmt{

enum coordinate { S_COORD = 1, D_COORD = 2};

// Jerk Minimal Trajectory
class JerkMinimalTrajectory
{
private:
    double distance(double x1, double y1, double x2, double y2);
    vector <double> s_coeffs, d_coeffs;
    vector <double> get_coeffs(coordinate c);
    double get_position(double x, coordinate c);
    double get_velocity(double x, coordinate c);
    double get_acceleration(double x, coordinate c);
    vector <double> s_velocity_buffer, s_acceleration_buffer,
                    d_velocity_buffer, d_acceleration_buffer;
    

public:
    vector <double> s_position_buffer, d_position_buffer;
    void set_coeffs(const vector<double>& start,
                    const vector<double>& end, double T);
    void fill_buffers(double update_rate, double buffer_size);
    void clear_buffers();
    vector <double> get_initial_boundary_conditions(double current_position_s, double current_position_d);
};

} // end namespace jmt

#endif //JMT_H
