#ifndef JMT_H
#define JMT_H

#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "helpers.h"
#include "spline.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

namespace jmt{

enum coordinate { S_COORD = 1, D_COORD = 2};

// Jerk Minimal Trajectory
class JerkMinimalTrajectory
{
private:
    vector <double> s_coeffs, d_coeffs;
    vector <double> get_coeffs(coordinate c);
    double get_position(double x, coordinate c);
    double get_velocity(double x, coordinate c);
    double get_acceleration(double x, coordinate c);
    double get_jerk(double x, coordinate c);
    void fill_buffers(double buffer_size);
    void clear_buffers();
    vector <double> s_position, s_velocity, s_acceleration, s_jerk,
                     d_position, d_velocity, d_acceleration, d_jerk;
    tk::spline s_x, s_y, s_dx, s_dy;
    

public:
    vector <double> x_position, y_position;
    void set_coeffs(const vector<double>& start, const vector<double>& end, double T);
    vector <double> get_initial_boundary_conditions(double current_position_s, double current_position_d);
    bool valid_trajectory(int num_points);
    void set_trajectory(const vector<double> &boundary_i, const vector<double> &boundary_f, double Tmin, int num_points);
    void set_splines(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);

};

} // end namespace jmt

#endif //JMT_H
