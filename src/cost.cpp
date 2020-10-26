#include "cost.h"

double calculate_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane, double prev_size){
  return weight_following*following_cost(sensor_fusion, jmt, lane)
        + weight_collision*collision_cost(sensor_fusion, jmt, lane, prev_size)
        + weight_final_speed*final_speed_cost(sensor_fusion, jmt, lane) 
        + weight_lane_speed*lane_speed_cost(sensor_fusion, jmt, lane)
        + weight_outofbounds*outofbounds_cost(sensor_fusion, jmt, lane);
}

double final_speed_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane){
  double target_speed = MPH_TO_MPS*MAX_SPEED_MPH;
  double init_x, init_y, final_x, final_y;
  double final_speed =  magnitude(jmt.s_velocity[NUM_SIMULATOR_POINTS-1], jmt.d_velocity[NUM_SIMULATOR_POINTS-1]);
  // std::cout << "speed cost: " << (target_speed-final_speed)/target_speed << std::endl;
  return (target_speed-final_speed)/target_speed;
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

double following_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane){
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  double car_x, car_y;
  double following_distance = 30;
  double min_dist = following_distance;

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    check_vx = nonego_data[3];
    check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];

    car_x = jmt.x_position[0];
    car_y = jmt.y_position[0];
    if(check_d > LANE_WIDTH*lane && check_d < LANE_WIDTH*(lane+1))
      if(distance(check_x, check_y, car_x, car_y) < min_dist)
        min_dist = distance(check_x, check_y, car_x, car_y);
    
  }
  
  std::cout << "following cost: " << (following_distance-min_dist)/following_distance << std::endl;
  return (following_distance-min_dist)/following_distance;
}


double collision_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane, double prev_size){
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  double car_x, car_y, x_disp, y_disp;
  double collision_distance = LANE_WIDTH*0.75;
  double min_dist = collision_distance;

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    check_vx = nonego_data[3];
    check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];

    for(int i=0; i<NUM_SIMULATOR_POINTS; i++){
      x_disp = check_vx*UPDATE_RATE*(prev_size+i+1);
      y_disp = check_vy*UPDATE_RATE*(prev_size+i+1);
      car_x = jmt.x_position[i];
      car_y = jmt.y_position[i];
      if(distance(check_x+x_disp, check_y+y_disp, car_x, car_y) < min_dist)
        min_dist = distance(check_x + x_disp, check_y + y_disp, car_x, car_y);
    }
  }
  std::cout << "collision cost: " << (collision_distance-min_dist)/collision_distance << std::endl;
  return (collision_distance-min_dist)/collision_distance;
}


double outofbounds_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane){
  double car_d;
  for(int i=0; i<NUM_SIMULATOR_POINTS; i++){
    car_d = jmt.d_position[i];
    if(car_d<0 || car_d>(3*LANE_WIDTH))
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