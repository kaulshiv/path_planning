#ifndef COST_H
#define COST_H
#include <algorithm>  
#include <map>
#include <tuple>
#include <iostream>
#include "helpers.h"

using std::map;
using std::pair;
using std::string;
using std::vector;

const double weight_collision = 1;
const double weight_average_speed = 1;
const double weight_lane_speed = 0;


double calculate_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int cur_lane);
double collision_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int cur_lane);
double average_speed_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int cur_lane);
double lane_speed_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int cur_lane);
bool compare_costs(pair<int, double> i, pair<int, double> j);
int get_min_cost(map<int, double> mymap);

#endif  // COST_H