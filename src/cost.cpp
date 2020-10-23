#include "cost.h"

double calculate_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int lane){
  return weight_collision*collision_cost(trajectory, sensor_fusion, lane) + weight_average_speed*average_speed_cost(trajectory, sensor_fusion, lane) 
        + weight_lane_speed*lane_speed_cost(trajectory, sensor_fusion, lane);
}

double average_speed_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int lane){
  double target_speed = MPH_TO_MPS*MAX_SPEED_MPH;
  double init_x, init_y, final_x, final_y;
  init_x = trajectory[0][0];
  init_y = trajectory[0][1];
  final_x = trajectory[NUM_SIMULATOR_POINTS-1][0];
  final_y = trajectory[NUM_SIMULATOR_POINTS-1][1];
  double average_speed = distance(init_x, init_y, final_x, final_y)/(UPDATE_RATE*NUM_SIMULATOR_POINTS);
  std::cout << "speed cost: " << (target_speed-average_speed)/target_speed << std::endl;
  return (target_speed-average_speed)/target_speed;
}

double lane_speed_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int lane){
  double target_speed = MPH_TO_MPS*MAX_SPEED_MPH;
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  double car_x = trajectory[NUM_SIMULATOR_POINTS-1][0];
  double car_y = trajectory[NUM_SIMULATOR_POINTS-1][1];
  double lane_speed = target_speed;

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    check_vx = nonego_data[3];
    check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];

    if(distance(check_x, check_y, car_x, car_y) < 50 && check_d > LANE_WIDTH*lane && check_d < LANE_WIDTH*(lane+1) )
      if(magnitude(check_vx, check_vy) < lane_speed){
        lane_speed = magnitude(check_vx, check_vy);
      }
  }
  return (target_speed-lane_speed)/target_speed;
}

double collision_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int lane){
  double check_x, check_y, check_vx, check_vy, check_s, check_d;
  double car_x, car_y, x_disp, y_disp;
  double min_dist = 30;

  for(auto nonego_data: sensor_fusion){
    check_x = nonego_data[1];
    check_y = nonego_data[2];
    check_vx = nonego_data[3];
    check_vy = nonego_data[4];
    check_s = nonego_data[5];
    check_d = nonego_data[6];



    for(int i=0; i<NUM_SIMULATOR_POINTS; i++){
      x_disp = check_vx*UPDATE_RATE*i;
      y_disp = check_vy*UPDATE_RATE*i;
      car_x = trajectory[i][0];
      car_y = trajectory[i][1];
      if(distance(check_x+x_disp, check_y+y_disp, car_x, car_y) < min_dist)
        min_dist = distance(check_x + x_disp, check_y + y_disp, car_x, car_y);
    }

    // if(check_d > LANE_WIDTH*lane && check_d < LANE_WIDTH*(lane+1)){
    // std::cout << "min dist: " << min_dist << std::endl;
    // }
  }
  
  std::cout << "collision cost: " << (30-min_dist)/30 << std::endl;
  return (30-min_dist)/30;
}

bool compare_costs(pair<int, double> i, pair<int, double> j){ 
  return i.second < j.second; 
}

int get_min_cost(map<int, double> mymap){
  pair<int, double> min = *min_element(mymap.begin(), mymap.end(), compare_costs);
  return min.first; 
}