#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "jmt.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int num_consumed =0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &num_consumed]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    int lane = 1;
    // double ref_vel = 49.5; // reference velocity in mph
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals = {};
          vector<double> next_y_vals = {};

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();
          num_consumed += (3.0/UPDATE_RATE)-prev_size;
          std::cout << "number of points consumed:" << num_consumed <<std::endl;

          vector <double> previous_path_s, previous_path_d;
          double prev_x = car_x, prev_y=car_y;
          // std::cout << "initial yaw: " << car_yaw << std::endl;
          for(int i=0; i<prev_size;i++){
            double x = previous_path_x[i];
            double y = previous_path_y[i];
            double theta = atan2( x - prev_x, y - prev_y);
            prev_x = x; prev_y=y;
            vector <double> frenet = getFrenet(x, y, theta, map_waypoints_x, map_waypoints_y);
            previous_path_s.push_back(frenet[0]);
            previous_path_d.push_back(frenet[1]);
            // std::cout << "theta: " << theta << ", s: "<< frenet[0] << ", d: "<< frenet[1] <<  std::endl;
          }

          vector <double> boundary_si, boundary_sf; //initial and final boundary conditions for s coordinate
          vector <double> boundary_di, boundary_df; //initial and final boundary conditions for d coordinate

          if(prev_size>2){
            double s_vel_0 = (previous_path_s[prev_size-1]-previous_path_s[prev_size-2])/UPDATE_RATE;
            double prev_s_vel_0 = (previous_path_s[prev_size-2]-previous_path_s[prev_size-3])/UPDATE_RATE;
            double s_acc_0 = (s_vel_0 - prev_s_vel_0)/UPDATE_RATE;
            
            double d_vel_0 = (previous_path_d[prev_size-1]-previous_path_d[prev_size-2])/UPDATE_RATE;
            double prev_d_vel_0 = (previous_path_d[prev_size-2]-previous_path_d[prev_size-3])/UPDATE_RATE;
            double d_acc_0 = (d_vel_0 - prev_d_vel_0)/UPDATE_RATE;

            std::cout << "s[-1] " << previous_path_s[prev_size-1] << std::endl;
            // std::cout << "s[-2] " << previous_path_s[prev_size-2] << std::endl;
            // std::cout << "s[-3] " << previous_path_s[prev_size-3] << std::endl;

            // std::cout << "s_vel_0: " << s_vel_0 << ", s_acc_0: " << s_acc_0 << std::endl;
            // std::cout << "d_vel_0: " << d_vel_0 << ", d_acc_0: " << d_acc_0 << std::endl;

            boundary_si.push_back(previous_path_s[prev_size-1]); boundary_si.push_back(s_vel_0);  boundary_si.push_back(0);
            boundary_sf.push_back(previous_path_s[prev_size-1]+20); boundary_sf.push_back(MPH_TO_MPS*MAX_SPEED_MPH*0.5);  boundary_sf.push_back(0.0);
            boundary_di.push_back(previous_path_d[prev_size-1]); boundary_di.push_back(d_vel_0);  boundary_di.push_back(0);
            boundary_df.push_back(2+LANE_WIDTH*lane); boundary_df.push_back(0.0);  boundary_df.push_back(0.0);
          }
          else{
            std::cout << "NO PREV POINTS" << std::endl;
            boundary_si.push_back(car_s); boundary_si.push_back(car_speed);  boundary_si.push_back(0.0);
            boundary_sf.push_back(car_s+20); boundary_sf.push_back(MPH_TO_MPS*MAX_SPEED_MPH*0.5);  boundary_sf.push_back(0.0);
            boundary_di.push_back(car_d); boundary_di.push_back(0.0);  boundary_di.push_back(0.0);
            boundary_df.push_back(2+LANE_WIDTH*lane); boundary_df.push_back(0.0);  boundary_df.push_back(0.0);
          }

          double T = 3.0;
          JMT jmt_s, jmt_d;
          jmt_s.set_coeffs(boundary_si, boundary_sf, T); 
          jmt_d.set_coeffs(boundary_di, boundary_df, T); 

          for(int i=0;i<prev_size;i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          std::cout << "NEXT_VALS_________________" << std::endl;
          for(int i=0;i<(int)(T/UPDATE_RATE)-prev_size;i++){
            if(i==(int)(T/UPDATE_RATE)-prev_size-1)
              std::cout << "spos: " <<  jmt_s.position((i+1)*UPDATE_RATE) << ", dpos: " << jmt_d.position((i+1)*UPDATE_RATE) << std::endl;
            // std::cout << "svel: " <<  jmt_s.velocity((i+1)*UPDATE_RATE) << ", dvel: " << jmt_d.velocity((i+1)*UPDATE_RATE) << std::endl;
            // std::cout << "sacc: " <<  jmt_s.acceleration((i+1)*UPDATE_RATE) << ", dacc: " << jmt_d.acceleration((i+1)*UPDATE_RATE) << std::endl;

            vector <double> xy = getXY(jmt_s.position((i+1)*UPDATE_RATE), jmt_d.position((i+1)*UPDATE_RATE), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            // std::cout << "x: " <<  xy[0] << ", y: " << xy[1] << std::endl;
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          // std::cout << "NEXT_VALS_________________" << std::endl;
          // for(int i=0; i<next_x_vals.size() ;i++){
          //   std::cout << next_x_vals[i] << ", " << next_y_vals[i] << std::endl;
          // }
          // std::cout << "CUR_________________" << std::endl;
          // std::cout << "s:" << car_s << ", d:" << car_d << std::endl;
          // std::cout << "speed:"<< car_speed << ", converted:" << car_speed*MPH_TO_MPS <<  ", vel_approx:" << vel_approx << ", accel_approx: "<<  accel_approx << ", prev_size:" << prev_size << std::endl;



          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}