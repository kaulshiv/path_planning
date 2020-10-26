#include "jmt.h"

namespace jmt{

vector <vector <double>> JerkMinimalTrajectory::generate_trajectory(const vector<double> &boundary_i, const vector<double> &boundary_f, double Tmin){
  double s, d;
  vector <vector <double>> xy_trajectory = {};
  vector <double> xy_coordinates;

  this->set_coeffs(boundary_i, boundary_f, Tmin);
  while(!this->valid_trajectory()){
    Tmin += UPDATE_RATE;
    this->set_coeffs(boundary_i, boundary_f, Tmin);
  }

  // std::cout << "Tmin: " << Tmin << std::endl;
  this->clear_buffers();
  this->fill_buffers();

  for(int i=0;i<NUM_SIMULATOR_POINTS;i++){
    xy_coordinates = {};
    xy_coordinates.push_back(this->x_position[i]);
    xy_coordinates.push_back(this->y_position[i]);
    xy_trajectory.push_back(xy_coordinates);
  }

  return xy_trajectory;
}


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

double JerkMinimalTrajectory::get_jerk(double x, coordinate c){
  vector <double> coeffs=this->get_coeffs(c);
  return 6*coeffs[3] + 24*coeffs[4]*x + 60*coeffs[5]*pow(x,2);
}

void JerkMinimalTrajectory::clear_buffers()
{
  this->s_position.clear();
  this->s_velocity.clear();
  this->s_acceleration.clear();
  this->d_position.clear();
  this->d_velocity.clear();
  this->d_acceleration.clear();
  this->x_position.clear();
  this->y_position.clear();   
}

void JerkMinimalTrajectory::fill_buffers(){
  double s, d;

  for(int i=0;i<NUM_SIMULATOR_POINTS;i++){
    s = fmod(this->get_position((i+1)*UPDATE_RATE, S_COORD), TRACK_LENGTH);
    d = this->get_position((i+1)*UPDATE_RATE, D_COORD);

    this->s_position.push_back(s);
    this->s_velocity.push_back(this->get_velocity((i+1)*UPDATE_RATE, S_COORD));
    this->s_acceleration.push_back(this->get_acceleration((i+1)*UPDATE_RATE, S_COORD));
    this->s_jerk.push_back(this->get_jerk((i+1)*UPDATE_RATE, S_COORD));

    this->d_position.push_back(d);
    this->d_velocity.push_back(this->get_velocity((i+1)*UPDATE_RATE, D_COORD));
    this->d_acceleration.push_back(this->get_acceleration((i+1)*UPDATE_RATE, D_COORD));
    this->d_jerk.push_back(this->get_jerk((i+1)*UPDATE_RATE, D_COORD));

    this->x_position.push_back(this->s_x(s) + d*this->s_dx(s));
    this->y_position.push_back(this->s_y(s) + d*this->s_dy(s));
    // std::cout << "final velocity s:\t" << s_velocity[i] << std::endl;
  }

}

vector <double> JerkMinimalTrajectory::get_initial_boundary_conditions(double current_position_x, double current_position_y){
//   if(!this->s_position.size())
//     return {current_position_,0,0,current_position_d,0,0};
//   std::cout << "get init boundary conds " << std::endl;
  double min_dist = distance(current_position_x, current_position_y, this->x_position[0], this->y_position[0]);
//   std::cout << "get initial dist " << std::endl;
  double min_ind = 0, new_dist;
  for(int i=0; i<this->x_position.size(); i++){
    new_dist = distance(current_position_x, current_position_y, this->x_position[i], this->y_position[i]);
    if(new_dist < min_dist){
        min_dist = new_dist;
        min_ind = i;
    }
  }

  return {this->s_position[min_ind], this->s_velocity[min_ind], this->s_acceleration[min_ind],
          this->d_position[min_ind], this->d_velocity[min_ind], this->d_acceleration[min_ind]};
}


// Make sure the path doesn't violate any velocity, acceleration, or jerk constraints
bool JerkMinimalTrajectory::valid_trajectory(){

  bool is_valid = true;
  double total_jerk=0, total_velocity=0, total_acceleration=0; 
  for(int i=0; i<NUM_SIMULATOR_POINTS; i++){
    total_velocity = magnitude(this->get_velocity((i+1)*UPDATE_RATE, S_COORD), this->get_velocity((i+1)*UPDATE_RATE, D_COORD));
    total_acceleration = magnitude(this->get_acceleration((i+1)*UPDATE_RATE, S_COORD), this->get_acceleration((i+1)*UPDATE_RATE, D_COORD));
    total_jerk = magnitude(this->get_jerk((i+1)*UPDATE_RATE, S_COORD), this->get_jerk((i+1)*UPDATE_RATE, D_COORD));
    if(MAX_ACC*0.95 <= total_acceleration || MAX_SPEED_MPH*MPH_TO_MPS*0.95 <= total_velocity || MAX_JERK*0.95 <= total_jerk){
      is_valid = false;
      // std::cout << " ACC: " << total_acceleration << ",  VEL: " << total_velocity << ", JERK: " << total_jerk <<  std::endl;
      break;
    }
  }

  return is_valid;
}


void JerkMinimalTrajectory::set_splines(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, 
                                        vector<double> map_waypoints_dx, vector<double> map_waypoints_dy){
    this->s_x.set_points(map_waypoints_s, map_waypoints_x);
    this->s_y.set_points(map_waypoints_s, map_waypoints_y);
    this->s_dx.set_points(map_waypoints_s, map_waypoints_dx);
    this->s_dy.set_points(map_waypoints_s, map_waypoints_dy);
}

} // end namespace jmt