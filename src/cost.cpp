#include "cost.h"

double calculate_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane, bool is_lane_change, double ego_x, double ego_y, double ego_s){
  return weight_following*following_cost(sensor_fusion, jmt, lane, ego_x, ego_y, ego_s)
        + weight_lane_change*lane_change_cost(sensor_fusion, jmt, lane, is_lane_change, ego_x, ego_y, ego_s);
        // + weight_final_speed*final_speed_cost(sensor_fusion, jmt, lane) 
        // + weight_lane_speed*lane_speed_cost(sensor_fusion, jmt, lane)
        // + weight_outofbounds*outofbounds_cost(sensor_fusion, jmt, lane)
        // + weight_final_lane*final_lane_cost(sensor_fusion, jmt, lane) ;
}


double following_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane, double ego_x, double ego_y, double ego_s){
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  // double car_x, car_y, car_s, x_disp, y_disp;
  double following_distance = 30;
  double min_dist = following_distance;

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    // check_vx = nonego_data[3];
    // check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];

    // car_x = jmt.x_position[0];
    // car_y = jmt.y_position[0];
    // car_s = jmt.s_position[0];
    if(check_d > LANE_WIDTH*lane && check_d < LANE_WIDTH*(lane+1) && ego_s<check_s){
      if(distance(check_x, check_y, ego_x, ego_y) < min_dist)
        min_dist = distance(check_x, check_y, ego_x, ego_y);

    }
    
  }
  
  std::cout << "following cost: " << (following_distance-min_dist)/following_distance << std::endl;
  return (following_distance-min_dist)/following_distance;
}


double lane_change_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane, bool is_lane_change, double ego_x, double ego_y, double ego_s){
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  double car_x, car_y, x_disp, y_disp;

  if(!is_lane_change)
    return 0;

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    check_vx = nonego_data[3];
    check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];

    // car_x = jmt.x_position[0];
    // car_y = jmt.y_position[0];
    if(check_d > LANE_WIDTH*lane && check_d < LANE_WIDTH*(lane+1) && abs(ego_s-check_s)<15)
      return 1;
  }
  return 0;
}


double final_speed_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane){
  double target_speed = MPH_TO_MPS*MAX_SPEED_MPH;
  double final_speed =  magnitude(jmt.s_velocity[NUM_SIMULATOR_POINTS-1], jmt.d_velocity[NUM_SIMULATOR_POINTS-1]);
  // std::cout << "speed cost: " << (target_speed-final_speed)/target_speed << std::endl;
  return (target_speed-final_speed)/target_speed;
}

double final_lane_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane){
  double target_speed = MPH_TO_MPS*MAX_SPEED_MPH;
  double car_d = jmt.d_position[NUM_SIMULATOR_POINTS-1];

  if(car_d > LANE_WIDTH*lane && car_d < LANE_WIDTH*(lane+1))
    return 0;
  return 1;
}

double lane_speed_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane){
  double target_speed = MPH_TO_MPS*MAX_SPEED_MPH;
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  double car_s = jmt.s_position[NUM_SIMULATOR_POINTS-1];
  double car_x = jmt.x_position[NUM_SIMULATOR_POINTS-1];
  double car_y = jmt.y_position[NUM_SIMULATOR_POINTS-1];
  double lane_speed = target_speed;
  double closest_distance = std::numeric_limits<double>::max();

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    check_vx = nonego_data[3];
    check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];

    if(check_d > LANE_WIDTH*lane && check_d < LANE_WIDTH*(lane+1)){
      if(distance(check_x, check_y, car_x, car_y) < closest_distance){
        closest_distance = distance(check_x, check_y, car_x, car_y);
        if((check_s + magnitude(check_vx, check_vy)*UPDATE_RATE*NUM_SIMULATOR_POINTS) > car_s 
            && magnitude(check_vx, check_vy) < lane_speed){
          lane_speed = magnitude(check_vx, check_vy);
        }
      }

    }
  }
  return (target_speed-lane_speed)/target_speed;
}


double outofbounds_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane){
  double car_d;
  for(int i=0; i<NUM_SIMULATOR_POINTS; i++){
    car_d = jmt.d_position[i];
    if(car_d<0.5|| car_d>(3*LANE_WIDTH-0.5))
      return 1;
  }
    
  return 0;
}



bool compare_costs(pair<int, double> i, pair<int, double> j){ 
  return i.second < j.second; 
}

int get_min_cost(map<int, double> mymap){
  pair<int, double> min = *min_element(mymap.begin(), mymap.end(), compare_costs);
  return min.first; 
}