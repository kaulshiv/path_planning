#include <iostream>
#include <string>
#include "jmt.h"
// #include "helpers.h"

namespace jmt{

void JerkMinimalTrajectory::set_coeffs(const vector<double> &start, const vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A(3,3);
  VectorXd b_s(3), b_d(3), ans_s(3), ans_d(3);
  double si=start[0], si_d=start[1], si_dd=start[2];
  double sf=end[0],   sf_d=end[1],   sf_dd=end[2];
  double di=start[3], di_d=start[4], di_dd=start[5];
  double df=end[3],   df_d=end[4],   df_dd=end[5];

  this->s_coeffs = {si, si_d, 0.5*si_dd, 0, 0, 0};
  this->d_coeffs = {di, di_d, 0.5*di_dd, 0, 0, 0};
   
  A << pow(T,3), pow(T,4), pow(T,5),
        3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
        6*T, 12*pow(T,2), 20*pow(T,3);
  b_s << sf - (si + si_d*T + 0.5*si_dd*pow(T,2)), sf_d - (si_d + si_dd*T), sf_dd - si_dd;
  b_d << df - (di + di_d*T + 0.5*di_dd*pow(T,2)), df_d - (di_d + di_dd*T), df_dd - di_dd;
  
  ans_s << A.inverse()*b_s;
  this->s_coeffs[3] = ans_s[0];
  this->s_coeffs[4] = ans_s[1];
  this->s_coeffs[5] = ans_s[2];
  ans_d << A.inverse()*b_d;
  this->d_coeffs[3] = ans_d[0];
  this->d_coeffs[4] = ans_d[1];
  this->d_coeffs[5] = ans_d[2];

}

vector <double> JerkMinimalTrajectory::get_coeffs(coordinate c){
  vector <double> coeffs;
  switch(c){
    case S_COORD:
      coeffs = this->s_coeffs;
      break;
    case D_COORD:
      coeffs = this->d_coeffs;
      break;
    default:
      std::cout << "Invalid coordinate input get coeffs" << std::endl;
      return {};
  }
  return coeffs;
}

double JerkMinimalTrajectory::get_position(double x, coordinate c){
  vector <double> coeffs=this->get_coeffs(c);
  double ans = 0;
  for (int i=0; i<coeffs.size(); i++)
    ans += coeffs[i] * pow(x,i);

  return ans;
}

double JerkMinimalTrajectory::get_velocity(double x, coordinate c){
  vector <double> coeffs=this->get_coeffs(c);
  return coeffs[1] + 2*coeffs[2]*x + 3*coeffs[3]*pow(x,2) + 4*coeffs[4]*pow(x,3) + 5*coeffs[5]*pow(x,4);
}


double JerkMinimalTrajectory::get_acceleration(double x, coordinate c){
  vector <double> coeffs=this->get_coeffs(c);
  return 2*coeffs[2] + 6*coeffs[3]*x + 12*coeffs[4]*pow(x,2) + 20*coeffs[5]*pow(x,3);
}


void JerkMinimalTrajectory::clear_buffers()
{
  this->s_position_buffer.clear();
  this->s_velocity_buffer.clear();
  this->s_acceleration_buffer.clear();
  this->d_position_buffer.clear();
  this->d_velocity_buffer.clear();
  this->d_acceleration_buffer.clear();
}

void JerkMinimalTrajectory::fill_buffers(double update_rate, double buffer_size){
  for(int i=0;i<buffer_size;i++){
      this->s_position_buffer.push_back(this->get_position((i+1)*update_rate, S_COORD));
      this->s_velocity_buffer.push_back(this->get_velocity((i+1)*update_rate, S_COORD));
      this->s_acceleration_buffer.push_back(this->get_acceleration((i+1)*update_rate, S_COORD));
      this->d_position_buffer.push_back(this->get_position((i+1)*update_rate, D_COORD));
      this->d_velocity_buffer.push_back(this->get_velocity((i+1)*update_rate, D_COORD));
      this->d_acceleration_buffer.push_back(this->get_acceleration((i+1)*update_rate, D_COORD));
  }
}

vector <double> JerkMinimalTrajectory::get_initial_boundary_conditions(double current_position_s, double current_position_d){
  if(!this->s_position_buffer.size())
    return {current_position_s,0,0,current_position_d,0,0};
  
  double min_dist = this->distance(current_position_s, current_position_d, this->s_position_buffer[0], this->d_position_buffer[0]);
  double min_ind = 0, new_dist;
  for(int i=0; i<this->s_position_buffer.size(); i++){
    new_dist = this->distance(current_position_s, current_position_d, this->s_position_buffer[i], this->d_position_buffer[i]);
    if(new_dist < min_dist){
        min_dist = new_dist;
        min_ind = i;
    }
  }
  
  return {this->s_position_buffer[min_ind], this->s_velocity_buffer[min_ind], this->s_acceleration_buffer[min_ind],
          this->d_position_buffer[min_ind], this->d_velocity_buffer[min_ind], this->d_acceleration_buffer[min_ind]};
}


// Calculate distance between two points
double JerkMinimalTrajectory::distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

} // end namespace jmt