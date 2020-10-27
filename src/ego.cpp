#include "ego.h"

EgoVehicle::EgoVehicle(vehicle_state initial_state, int lane_num){
  this->state = initial_state;
  this->current_lane = lane_num;
  this->destination_lane = -1;
}

void EgoVehicle::set_splines(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, 
                             vector<double> map_waypoints_dx, vector<double> map_waypoints_dy){
  this->jmt.set_splines(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
}

vector <vehicle_state> EgoVehicle::get_next_states(vector <vector <double>> sensor_fusion){
  vector <vehicle_state> next_states = {KEEP_LANE};
  switch(this->state){
      case KEEP_LANE:
        if(valid_turn(LANE_CHANGE_RIGHT, sensor_fusion))
          next_states.push_back(LANE_CHANGE_RIGHT);
        if(valid_turn(LANE_CHANGE_LEFT, sensor_fusion))
          next_states.push_back(LANE_CHANGE_LEFT);
        break;
      case LANE_CHANGE_RIGHT:
        if(valid_turn(LANE_CHANGE_RIGHT, sensor_fusion))
          next_states.push_back(LANE_CHANGE_RIGHT);
        break;
      case LANE_CHANGE_LEFT:
        if(valid_turn(LANE_CHANGE_LEFT, sensor_fusion))
          next_states.push_back(LANE_CHANGE_LEFT);
        break;
      default:
        std::cout << "Invalid State, get_next_state" << std::endl;
        break;
  }
  return next_states;
}

bool EgoVehicle::valid_turn(vehicle_state turn_state, vector <vector <double>> sensor_fusion){
  double back_buffer = 15, front_buffer = 15;
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  int destination = (turn_state == LANE_CHANGE_LEFT) ? this->current_lane-1: this->current_lane+1;

  if((turn_state==LANE_CHANGE_RIGHT && this->current_lane<2) || (turn_state==LANE_CHANGE_LEFT && this->current_lane>0)){
    // for(auto nonego_data: sensor_fusion){
    //   check_x = nonego_data[1];
    //   check_y = nonego_data[2];
    //   check_vx = nonego_data[3];
    //   check_vy = nonego_data[4];
    //   check_s = nonego_data[5];
    //   check_d = nonego_data[6];

    //   if(check_d > LANE_WIDTH*destination && check_d < LANE_WIDTH*(destination+1))
    //     if((check_s>this->current_s  && (check_s-this->current_s)<front_buffer) || 
    //        (check_s<this->current_s  && (this->current_s-check_s)<back_buffer))
    //        return false;
    // }
    return true;
  }

  return false;

}

vector <vector <double>> EgoVehicle::transition_function(vector <vector <double>> sensor_fusion){
  // only consider states which can be reached from current FSM state.
  vector <vehicle_state> possible_successor_states = this->get_next_states(sensor_fusion);
  vector <string> statenames = {"KEEP_LANE", "LANE_CHANGE_RIGHT", "LANE_CHANGE_LEFT"};

  // get initial boundary conditions
  vector <double> boundary_i;
  if(previous_path_size){
    boundary_i = jmt.get_initial_boundary_conditions(this->previous_path_x[previous_path_size-1], this->previous_path_y[previous_path_size-1]);
  }
  else{
    boundary_i = {this->current_s, 0.0, 0.0, this->current_d, 0.0, 0.0};
  }

   // keep track of the total cost of each state.
  vector <vector <double>> trajectory_for_state = {};
  map<int, double> costs = {};
  int destination; 
  std::cout << "current state: " << statenames[this->state] << std::endl;
  for(const auto state : possible_successor_states) {
      // generate a rough idea of what trajectory we would
      // follow IF we chose this state.
    trajectory_for_state = this->get_trajectory(state, sensor_fusion, boundary_i);
    if(state == LANE_CHANGE_LEFT || state == LANE_CHANGE_RIGHT){
      destination = (state == LANE_CHANGE_LEFT) ? this->current_lane-1: this->current_lane+1;
      costs[state] = calculate_cost(sensor_fusion, this->jmt, destination, true, this->current_x, this->current_y, this->current_s);
    }
    else
      costs[state] = calculate_cost(sensor_fusion, this->jmt, this->current_lane, false, this->current_x, this->current_y, this->current_s);
    std::cout << statenames[state] << ", cost: " << costs[state] << std::endl;

  }

  vehicle_state new_state = static_cast<vehicle_state>(get_min_cost(costs));
  if((new_state == LANE_CHANGE_LEFT || this->state == LANE_CHANGE_RIGHT) && this->state != new_state)
    this->destination_lane = (new_state == LANE_CHANGE_LEFT) ? this->current_lane-1: this->current_lane+1;
  else if(!(new_state == LANE_CHANGE_LEFT || this->state == LANE_CHANGE_RIGHT))
    this->destination_lane = -1;

  this->state = new_state;

  return this->get_trajectory(new_state, sensor_fusion, boundary_i);
}

vector <vector <double>> EgoVehicle::get_trajectory(vehicle_state new_state, vector <vector <double>> sensor_fusion, vector <double> boundary_i){
  vector <vector <double>> trajectory = {};

  switch(new_state){   
    case KEEP_LANE:
      return this->keep_lane_trajectory(sensor_fusion, boundary_i);
    case LANE_CHANGE_RIGHT:
      return this->lane_change_trajectory(LANE_CHANGE_RIGHT, sensor_fusion, boundary_i);
    case LANE_CHANGE_LEFT:
      return this->lane_change_trajectory(LANE_CHANGE_LEFT, sensor_fusion, boundary_i);
    default:
      std::cout << "Invalid State, get_trajectory" << std::endl;
      return {};
  }

  return trajectory;
}

void EgoVehicle::set_pose(double x, double y, double s, double d, const vector <double> &prev_x, const vector <double>  &prev_y){
  this->current_x = x;
  this->current_y = y;
  this->current_s = s;
  this->current_d = d;
  this->previous_path_x = prev_x;
  this->previous_path_y = prev_y;
  this->previous_path_size = previous_path_x.size();

  // std::cout << "prev size: " << prev_x.size() << std::endl;
//   int prev_lane = this->current_lane;

  if(this->current_d >= 0 && this->current_d <= LANE_WIDTH){
    this->current_lane = 0;
  }
  else if(this->current_d > LANE_WIDTH && this->current_d <= 2*LANE_WIDTH){
    this->current_lane = 1;
  }
  else if(this->current_d > 2*LANE_WIDTH && this->current_d <= 3*LANE_WIDTH){
    this->current_lane = 2;
  }

  std::cout << "\n__________________" << std::endl;
  std::cout << "current lane: " << this->current_lane << std::endl;

//   bool change_lanes = (prev_lane != this->current_lane) ? true: false;
}


vector <vector<double>> EgoVehicle::keep_lane_trajectory(vector <vector <double>> sensor_fusion, vector <double> initial_conditions){
  double final_s_vel = MPH_TO_MPS*MAX_SPEED_MPH*0.95;
  double add_on_dist = 30, Tmin;
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  vector <double> boundary_f;
  double closest_dist = add_on_dist;

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    check_vx = nonego_data[3];
    check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];

    if(check_d > LANE_WIDTH*this->current_lane && check_d < LANE_WIDTH*(this->current_lane+1)){
      if(check_s > this->current_s && ((check_s - this->current_s) < (closest_dist))){
        final_s_vel = magnitude(check_vx, check_vy);
        closest_dist = (check_s - this->current_s);
      }
    }
  }
  
  if(previous_path_size){
    boundary_f = {fmod(initial_conditions[0]+add_on_dist, TRACK_LENGTH), final_s_vel, 0.0, 2+LANE_WIDTH*this->current_lane, 0.0, 0.0};
  }
  else{
    boundary_f = {fmod(this->current_s+add_on_dist, TRACK_LENGTH), final_s_vel, 0.0, 2+LANE_WIDTH*this->current_lane, 0.0, 0.0};
  }

  Tmin = add_on_dist/final_s_vel;

  return jmt.generate_trajectory(initial_conditions, boundary_f, Tmin);
}


vector <vector<double>> EgoVehicle::lane_change_trajectory(vehicle_state new_state, vector <vector <double>> sensor_fusion, vector <double> initial_conditions){
  double final_s_vel = MPH_TO_MPS*MAX_SPEED_MPH*0.95;
  double add_on_dist = 30, Tmin;
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  vector <double> boundary_f;
  double closest_dist = add_on_dist;
  double destination = (new_state == LANE_CHANGE_LEFT) ? this->current_lane-1: this->current_lane+1;

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    check_vx = nonego_data[3];
    check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];

    if(check_d > LANE_WIDTH*destination && check_d < LANE_WIDTH*(destination+1)){
        if(check_s < this->current_s && ((this->current_s - check_s) < closest_dist)){
            final_s_vel = magnitude(check_vx, check_vy);
            closest_dist = (this->current_s - check_s);
        }
    }
  }
  
  if(previous_path_size){
    boundary_f = {fmod(initial_conditions[0]+add_on_dist, TRACK_LENGTH), final_s_vel, 0.0, 2+LANE_WIDTH*destination, 0.0, 0.0};
  }
  else{
    boundary_f = {fmod(this->current_s+add_on_dist, TRACK_LENGTH), final_s_vel, 0.0, 2+LANE_WIDTH*destination, 0.0, 0.0};
  }
  Tmin = add_on_dist/final_s_vel;

  return jmt.generate_trajectory(initial_conditions, boundary_f, Tmin);
}