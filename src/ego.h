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
                    PREPARE_LANE_CHANGE_RIGHT = 1,
                    PREPARE_LANE_CHANGE_LEFT = 2,
                    LANE_CHANGE_RIGHT = 3,
                    LANE_CHANGE_LEFT = 4};


class EgoVehicle{
  public:
    void set_pose(double x, double y, double s, double d, double xprev, double yprev);
    vector <vehicle_state> get_next_states();
    vector <vector <double>> transition_function(vector <vector <double>> sensor_fusion);
    EgoVehicle(vehicle_state initial_state, int lane_num, JerkMinimalTrajectory *p_jmt);

  private:
    vehicle_state state;
    JerkMinimalTrajectory *p_jmt; //point to the jmt structure
    int current_lane, destination_lane;
    double current_x, current_y, current_s, current_d, initial_x, initial_y;
    vector <vector <double>> get_trajectory(vehicle_state new_state, vector <vector <double>> sensor_fusion, bool set_trajectory);
    vector <vector<double>> keep_lane_trajectory(vector <vector <double>> sensor_fusion, bool set_trajectory);
    vector <vector<double>> prepare_lane_change_trajectory(vector <vector <double>> sensor_fusion, bool set_trajectory);
    vector <vector<double>> lane_change_trajectory(vehicle_state new_state, vector <vector <double>> sensor_fusion, bool set_trajectory);


};

#endif // EGO_H