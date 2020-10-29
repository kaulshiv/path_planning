#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>

#define MAX_SPEED_MPH 50
#define MAX_ACC 10.0
#define MAX_JERK 50.0
#define MPH_TO_MPS 0.44704
#define UPDATE_RATE 0.02
#define LANE_WIDTH 4.0
#define TRACK_LENGTH 6945.554
#define NUM_SIMULATOR_POINTS 50

// for convenience
using std::string;
using std::vector;

string hasData(string s);

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// compute sum of squares
inline double magnitude(double x, double y) { return pow(pow(x,2) + pow(y,2), 0.5); }


// Calculate distance between two points
inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);

vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y);

void extend_waypoints(vector<double> &map_waypoints_x, 
                   vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, 
                   vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy);

#endif  // HELPERS_H