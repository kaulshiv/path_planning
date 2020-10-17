#include "cost.h"

double calculate_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int lane){
  return weight_collision*collision_cost(trajectory, sensor_fusion, lane);// + weight_speed*speed_cost(trajectory);
}

double speed_cost(vector <vector <double>> trajectory){
  double target_speed = MPH_TO_MPS*MAX_SPEED_MPH*0.95;
  double init_x, init_y, final_x, final_y;
  init_x = trajectory[0][0];
  init_y = trajectory[0][1];
  final_x = trajectory[NUM_SIMULATOR_POINTS-1][0];
  final_y = trajectory[NUM_SIMULATOR_POINTS-1][1];
  double average_speed = distance(init_x, init_y, final_x, final_y)/(UPDATE_RATE*NUM_SIMULATOR_POINTS);
  return (target_speed-average_speed)/target_speed;
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
    car_x = trajectory[NUM_SIMULATOR_POINTS-1][0];
    car_y = trajectory[NUM_SIMULATOR_POINTS-1][1];
    x_disp = check_vx*UPDATE_RATE*NUM_SIMULATOR_POINTS;
    y_disp = check_vy*UPDATE_RATE*NUM_SIMULATOR_POINTS;

    if(check_d > LANE_WIDTH*lane && check_d < LANE_WIDTH*(lane+1)){
      if(distance(check_x+x_disp, check_y+y_disp, car_x, car_y) < min_dist){
        min_dist = distance(check_x + x_disp, check_y + y_disp, car_x, car_y);
        // std::cout << "min dist: " << min_dist << std::endl;
      }
    }
  }
  
  return 1-exp(-1/min_dist);
}

bool compare_costs(pair<int, double> i, pair<int, double> j){ 
  return i.second < j.second; 
}

int get_min_cost(map<int, double> mymap){
  pair<int, double> min = *min_element(mymap.begin(), mymap.end(), compare_costs);
  return min.first; 
}