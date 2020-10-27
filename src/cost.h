#ifndef COST_H
#define COST_H
#include <algorithm>  
#include <map>
#include <tuple>
#include <iostream>
#include <limits>
#include "helpers.h"
#include "jmt.h"

using std::map;
using std::pair;
using std::string;
using std::vector;
using jmt::JerkMinimalTrajectory;

const double weight_following = 1;
const double weight_lane_change = 1;
// const double weight_final_speed = 0;
// const double weight_lane_speed = 0;
// const double weight_final_lane = 0;
// const double weight_outofbounds = 0;


double calculate_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int cur_lane, bool is_lane_change, double ego_x, double ego_y, double ego_s);
double following_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane, double ego_x, double ego_y, double ego_s);
double lane_change_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane, bool is_lane_change, double ego_x, double ego_y, double ego_s);



// double final_speed_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int cur_lane);
// double lane_speed_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int cur_lane);
// double outofbounds_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane);
// double final_lane_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane);

bool compare_costs(pair<int, double> i, pair<int, double> j);
int get_min_cost(map<int, double> mymap);

#endif  // COST_H