#ifndef EGO_H
#define EGO_H

#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include "cost.h"
#include "helpers.h"
#include "jmt.h"

using jmt::JerkMinimalTrajectory;

enum vehicle_state {KEEP_LANE = 0, 
                    LANE_CHANGE_RIGHT = 1,
                    LANE_CHANGE_LEFT = 2};


class EgoVehicle{
  public:
    void set_pose(double x, double y, double s, double d, const vector <double> &prev_x, const vector <double>  &prev_y);
    vector <vehicle_state> get_next_states();
    vector <vector <double>> transition_function(vector <vector <double>> sensor_fusion);
    void set_splines(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, 
                  vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
    EgoVehicle(vehicle_state initial_state, int lane_num);

  private:
    vehicle_state state;
    JerkMinimalTrajectory jmt;
    int current_lane, destination_lane;
    double current_x, current_y, current_s, current_d;
    vector <double> previous_path_x, previous_path_y;
    double previous_path_size;
    vector <vector <double>> get_trajectory(vehicle_state new_state, vector <vector <double>> sensor_fusion, vector <double> boundary_i);
    vector <vector<double>> keep_lane_trajectory(vector <vector <double>> sensor_fusion, vector <double> initial_conditions);
    vector <vector<double>> lane_change_trajectory(vehicle_state new_state, vector <vector <double>> sensor_fusion, vector <double> initial_conditions);


};

#endif // EGO_H