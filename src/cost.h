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

const double weight_outofbounds = 10000;
const double weight_following = 1;
const double weight_collision = 1000;
const double weight_final_speed = 0;
const double weight_lane_speed = 0;


double calculate_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int cur_lane, double prev_size);
double following_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane);
double collision_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane, double prev_size);
double final_speed_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int cur_lane);
double lane_speed_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int cur_lane);
double outofbounds_cost(vector <vector <double>> sensor_fusion, const JerkMinimalTrajectory &jmt, int lane);

bool compare_costs(pair<int, double> i, pair<int, double> j);
int get_min_cost(map<int, double> mymap);

#endif  // COST_H