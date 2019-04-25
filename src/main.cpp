#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

#define TARGET_SPEED_MPH ((double)45)
#define DELTA_T ((double)0.02)
#define MPH_TO_MPS ((double)0.44704)
#define TARGET_SPEED_MPS (TARGET_SPEED_MPH * MPH_TO_MPS)



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

  tk::spline s_to_x;
  tk::spline s_to_y;
  tk::spline s_to_dx;
  tk::spline s_to_dy;
  s_to_x.set_points(map_waypoints_s, map_waypoints_x);
  s_to_y.set_points(map_waypoints_s, map_waypoints_y);
  s_to_dx.set_points(map_waypoints_s, map_waypoints_dx);
  s_to_dy.set_points(map_waypoints_s, map_waypoints_dy);

  enum state {
    FOLLOW,
    KEEP_SPEED,
  } car_state = KEEP_SPEED;

  double car_last_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &s_to_x, &s_to_y,
               &s_to_dx, &s_to_dy, &car_state, &car_last_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //std::cout << "Got message" << std::endl;
    //fwrite(data, length, 1, stdout);
    //std::cout << std::endl;
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
          vector<double> previous_path_x = j[1]["previous_path_x"];
          double car_xx = 0;
          double car_xxx = 0;
          if (previous_path_x.size() > 3) {
            car_xx = previous_path_x[1] - previous_path_x[0];
            double car_xx2 = previous_path_x[2] - previous_path_x[1];
            car_xxx = car_xx2 - car_xx;
          }
          vector<double> car_state_x = {car_x, car_xx, car_xxx};

          vector<double> previous_path_y = j[1]["previous_path_y"];
          double car_yy = 0;
          double car_yyy = 0;
          if (previous_path_y.size() > 3) {
            car_yy = previous_path_y[1] - previous_path_y[0];
            double car_yy2 = previous_path_y[2] - previous_path_y[1];
            car_yyy = car_yy2 - car_yy;
          }
          vector<double> car_state_y = {car_y, car_yy, car_yyy};

          vector<double> car_state_s = {car_s, car_speed * MPH_TO_MPS, car_speed * MPH_TO_MPS - car_last_vel};
          car_last_vel = car_speed * MPH_TO_MPS;
          vector<double> car_state_d = {car_d, 0, 0};


          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int car_in_front;
          vector<double> desired_state_s;
          vector<double> desired_state_d;
          vector<double> next_s;
          vector<double> next_d;

          switch (car_state) {
          case KEEP_SPEED:
            desired_state_s.push_back(car_s + (TARGET_SPEED_MPS * 3.0));
            desired_state_s.push_back(TARGET_SPEED_MPS);
            desired_state_s.push_back(0);
            desired_state_d = {2, 0, 0};
            next_s = get_jmt_trajs(car_state_s, desired_state_s, 3.0, DELTA_T);
            next_d = get_jmt_trajs(car_state_d, desired_state_d, 3.0, DELTA_T);
            for (int i=0; i < next_s.size(); i++) {
              next_x_vals.push_back(s_to_x(next_s[i]) + 2 * s_to_dx(next_s[i]));
              next_y_vals.push_back(s_to_y(next_s[i]) + 2 * s_to_dy(next_s[i]));
            }
            car_in_front = get_car_in_front(car_s, car_d, sensor_fusion);
            if (car_in_front >= 0) {
              double cif_s = sensor_fusion[car_in_front][5];
              if ((cif_s - car_s) < 5) {
                car_state = FOLLOW;
                std::cout << "Following " << car_in_front << std::endl;
              }
            }
            break;
          case FOLLOW:
            car_in_front = get_car_in_front(car_s, car_d, sensor_fusion);
            if (car_in_front < 0) {
              car_state = KEEP_SPEED;
              break;
            }
            double cif_s = sensor_fusion[car_in_front][5];
            if ((cif_s - car_s) > 5) {
              car_state = KEEP_SPEED;
              break;
            }
            double vx = sensor_fusion[car_in_front][3];
            double vy = sensor_fusion[car_in_front][4];
            double car_in_front_v = sqrt(vx*vx+vy*vy);
            for (int i=0; i < (int)(2.25 / DELTA_T); i++) {
              double s = car_s + car_in_front_v * DELTA_T * i;
              double x = s_to_x(s)+2*s_to_dx(s);
              double y = s_to_y(s)+2*s_to_dy(s);
              next_x_vals.push_back(x);
              next_y_vals.push_back(y);
            }
            break;
          }

          

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

  h.onConnection([&h, &s_to_x, &s_to_y](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
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