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
const double weight_speed = 1;

double calculate_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int cur_lane);
double collision_cost(vector <vector <double>> trajectory, vector <vector <double>> sensor_fusion, int cur_lane);
double speed_cost(vector <vector <double>> trajectory);
bool compare_costs(pair<int, double> i, pair<int, double> j);
int get_min_cost(map<int, double> mymap);

#endif  // COST_H