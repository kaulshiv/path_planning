#include "ego.h"

EgoVehicle::EgoVehicle(vehicle_state initial_state, int lane_num, JerkMinimalTrajectory *pjmt){
  this->state = initial_state;
  this->current_lane = lane_num;
  this->p_jmt = pjmt;
  this->destination_lane = -1;
}

vector <vehicle_state> EgoVehicle::get_next_states(){
  vector <vehicle_state> next_states = {KEEP_LANE};
  switch(this->state){
      case KEEP_LANE:
        if(this->current_lane<2)
          next_states.push_back(LANE_CHANGE_RIGHT);
        if(this->current_lane>0)
          next_states.push_back(LANE_CHANGE_LEFT);
        break;
      case LANE_CHANGE_RIGHT:
        if(this->current_lane<2)
          next_states.push_back(LANE_CHANGE_RIGHT);
        break;
      case LANE_CHANGE_LEFT:
        if(this->current_lane>0)
          next_states.push_back(LANE_CHANGE_LEFT);
        break;
      default:
        std::cout << "Invalid State, get_next_state" << std::endl;
        break;
  }
  return next_states;
}

vector <vector <double>> EgoVehicle::transition_function(vector <vector <double>> sensor_fusion){
  // only consider states which can be reached from current FSM state.
  vector <vehicle_state> possible_successor_states = this->get_next_states();
  vector <string> statenames = {"KEEP_LANE", "LANE_CHANGE_RIGHT", "LANE_CHANGE_LEFT"};

   // keep track of the total cost of each state.
  vector <vector <double>> trajectory_for_state = {};
  map<int, double> costs = {};
  int destination; 
  std::cout << "current state: " << statenames[this->state] << std::endl;
  for(const auto state : possible_successor_states) {
      // generate a rough idea of what trajectory we would
      // follow IF we chose this state.
    trajectory_for_state = this->get_trajectory(state, sensor_fusion, false);
    if(this->destination_lane != -1){
      destination = (state == LANE_CHANGE_LEFT) ? this->current_lane-1: this->current_lane+1;
      costs[state] = calculate_cost(trajectory_for_state, sensor_fusion, destination);
    }
    else
      costs[state] = calculate_cost(trajectory_for_state, sensor_fusion, this->current_lane);
    std::cout << statenames[state] << ", cost: " << costs[state] << std::endl;

  }

  return this->get_trajectory(static_cast<vehicle_state>(get_min_cost(costs)), sensor_fusion, true);
}

vector <vector <double>> EgoVehicle::get_trajectory(vehicle_state new_state, vector <vector <double>> sensor_fusion, bool set_trajectory){
  vector <vector <double>> trajectory = {};

  if(set_trajectory){
    if((new_state == LANE_CHANGE_LEFT || this->state == LANE_CHANGE_RIGHT) && this->state != new_state)
      this->destination_lane = (new_state == LANE_CHANGE_LEFT) ? this->current_lane-1: this->current_lane+1;
    else if(!(new_state == LANE_CHANGE_LEFT || this->state == LANE_CHANGE_RIGHT))
      this->destination_lane = -1;

    this->state = new_state;
  }

  switch(new_state){   
    case KEEP_LANE:
      return this->keep_lane_trajectory(sensor_fusion, set_trajectory);
    // case PREPARE_LANE_CHANGE_RIGHT:
    //   // trajectory = this->prepare_lane_change_trajectory(PREPARE_LANE_CHANGE_RIGHT, predictions);
    //   break;
    // case PREPARE_LANE_CHANGE_LEFT:
    //   return this->prepare_lane_change_trajectory(sensor_fusion, set_trajectory);
    case LANE_CHANGE_RIGHT:
      return this->lane_change_trajectory(LANE_CHANGE_RIGHT, sensor_fusion, set_trajectory);
    case LANE_CHANGE_LEFT:
      return this->lane_change_trajectory(LANE_CHANGE_LEFT, sensor_fusion, set_trajectory);
    default:
      std::cout << "Invalid State, get_trajectory" << std::endl;
      return {};
  }

  return trajectory;
}

void EgoVehicle::set_pose(double x, double y, double s, double d, double xprev, double yprev){
  this->current_x = x;
  this->current_y = y;
  this->current_s = s;
  this->current_d = d;
  this->initial_x = xprev;
  this->initial_y = yprev;
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

  std::cout << "current lane: " << this->current_lane << std::endl;

//   bool change_lanes = (prev_lane != this->current_lane) ? true: false;
}


vector <vector<double>> EgoVehicle::keep_lane_trajectory(vector <vector <double>> sensor_fusion, bool set_trajectory){
  double final_s_vel = MPH_TO_MPS*MAX_SPEED_MPH*0.95;
  double add_on_dist = 30, Tmin;
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  vector <double> boundary_i, boundary_f;
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
        final_s_vel = magnitude(check_vx, check_vy) - 2;
        closest_dist = (check_s - this->current_s);
      }
    }
  }
  
  if(this->initial_x!=-1 && this->initial_y!=-1){
    boundary_i = this->p_jmt->get_initial_boundary_conditions(this->initial_x, this->initial_y);
    boundary_f = {boundary_i[0]+add_on_dist, final_s_vel, 0.0, 2+LANE_WIDTH*this->current_lane, 0.0, 0.0};
  }
  else{
    boundary_i = {this->current_s, 0.0, 0.0, this->current_d, 0.0, 0.0};
    boundary_f = {this->current_s+add_on_dist, final_s_vel, 0.0, 2+LANE_WIDTH*this->current_lane, 0.0, 0.0};
  }

  Tmin = add_on_dist/final_s_vel;

  return this->p_jmt->generate_trajectory(boundary_i, boundary_f, Tmin, set_trajectory);
}

vector <vector<double>> EgoVehicle::prepare_lane_change_trajectory(vector <vector <double>> sensor_fusion, bool set_trajectory){
  double final_s_vel = MPH_TO_MPS*MAX_SPEED_MPH*0.95;
  double add_on_dist = 30, Tmin;
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  vector <double> boundary_i, boundary_f;
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
  
  if(this->initial_x!=-1 && this->initial_y!=-1){
    boundary_i = this->p_jmt->get_initial_boundary_conditions(this->initial_x, this->initial_y);
    boundary_f = {boundary_i[0]+add_on_dist, final_s_vel, 0.0, 2+LANE_WIDTH*this->current_lane, 0.0, 0.0};
  }
  else{
    boundary_i = {this->current_s, 0.0, 0.0, this->current_d, 0.0, 0.0};
    boundary_f = {this->current_s+add_on_dist, final_s_vel, 0.0, 2+LANE_WIDTH*this->current_lane, 0.0, 0.0};
  }
  if(set_trajectory){
    std::cout << "initial s" << boundary_i[1] << std::endl;
    std::cout << "s vel " << final_s_vel << std::endl;
  }
  Tmin = add_on_dist/final_s_vel;
//   vector <vector<double>> trajectory = {};
//   if(set_trajectory){
//     trajectory = this->p_jmt->generate_trajectory(boundary_i, boundary_f, Tmin, set_trajectory);
//   }

  return this->p_jmt->generate_trajectory(boundary_i, boundary_f, Tmin, set_trajectory);
}

vector <vector<double>> EgoVehicle::lane_change_trajectory(vehicle_state new_state, vector <vector <double>> sensor_fusion, bool set_trajectory){
  double final_s_vel = MPH_TO_MPS*MAX_SPEED_MPH*0.95;
  double add_on_dist = 30, Tmin;
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  vector <double> boundary_i, boundary_f;
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
  
  if(this->initial_x!=-1 && this->initial_y!=-1){
    boundary_i = this->p_jmt->get_initial_boundary_conditions(this->initial_x, this->initial_y);
    boundary_f = {boundary_i[0]+add_on_dist, final_s_vel, 0.0, 2+LANE_WIDTH*destination, 0.0, 0.0};
  }
  else{
    boundary_i = {this->current_s, 0.0, 0.0, this->current_d, 0.0, 0.0};
    boundary_f = {this->current_s+add_on_dist, final_s_vel, 0.0, 2+LANE_WIDTH*destination, 0.0, 0.0};
  }
  Tmin = add_on_dist/magnitude(final_s_vel, boundary_f[4]);

  return this->p_jmt->generate_trajectory(boundary_i, boundary_f, Tmin, set_trajectory);
}