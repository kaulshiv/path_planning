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
  jmt::JerkMinimalTrajectory trajectory;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &num_consumed,
               &trajectory]
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
          // std::cout << "number of points consumed:" << num_consumed <<std::endl;

          vector <double> previous_path_s, previous_path_d;
          double prev_x = car_x, prev_y=car_y;
          vector <double> boundary_i, boundary_f; //initial and final boundary conditions for s coordinate

          double T = 3.0;
          double final_s_vel = MPH_TO_MPS*MAX_SPEED_MPH*0.9;
          if(prev_size){
            boundary_i = trajectory.get_initial_boundary_conditions(end_path_s, end_path_d);
            // std::cout << "boundary conds" << std::endl;
            for(auto i=boundary_i.begin(); i!=boundary_i.end(); ++i)
              printf("%f, ", *i);
            std::cout << std::endl;
            double add_on_dist = (final_s_vel + boundary_i[1])*T*0.5;
            boundary_f = {end_path_s+add_on_dist,final_s_vel, 0.0, 2+LANE_WIDTH*lane, 0.0, 0.0};
          }
          else{
            boundary_i = trajectory.get_initial_boundary_conditions(car_s, car_d);
            double add_on_dist = (final_s_vel + boundary_i[1])*T*0.5;
            boundary_f = {car_s+add_on_dist+10, final_s_vel, 0.0, 2+LANE_WIDTH*lane, 0.0, 0.0};
          }

          trajectory.set_coeffs(boundary_i, boundary_f, T); 
          trajectory.clear_buffers();
          trajectory.fill_buffers(UPDATE_RATE, (int)(T/UPDATE_RATE));

          for(int i=0;i<prev_size;i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // std::cout << "NEXT_VALS_________________" << std::endl;
          for(int i=0;i<(int)(T/UPDATE_RATE)-prev_size;i++){
            vector <double> xy = getXY(trajectory.s_position_buffer[i], trajectory.d_position_buffer[i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            std::cout << "s: " <<  trajectory.s_position_buffer[i] << ", d: " << trajectory.d_position_buffer[i] << std::endl;
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          // for(int i=0; i<next_x_vals.size() ;i++)
          //   std::cout << next_x_vals[i] << ", " << next_y_vals[i] << std::endl;

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