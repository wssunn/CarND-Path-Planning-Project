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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


/*
keep the vehicle inside its lane
avoid hitting other cars
pass slower moving traffic by using localisation, sensor fusion and map data
*/

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

  // Start on lane 1
  int intend_lane = 1;
  // Reference velocity (mph)
  double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &intend_lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // store the size of previous path that has not been used
          int prev_path_size = previous_path_x.size();
          if (prev_path_size > 0){car_s = end_path_s;}

          bool front_car_too_close = false;         //becomes true is front car is too close
          bool prepare_for_lane_change = false;
          bool ready_for_lane_change = false;
          bool is_left_lane_free = true;            //set to true, if there is a false, change to false
          bool is_right_lane_free = true;

          // iterate through all detected cars from sensor fusion
          for (size_t i = 0; i < sensor_fusion.size(); ++i){
            // access the data and store it in an object
            Vehicle vehicle(sensor_fusion[i]);

            //if there is a car in front of us
            if (is_in_same_lane(vehicle.d, intend_lane)){
              vehicle.s += (double)prev_path_size * 0.02 * vehicle.speed;
              bool is_in_front_of_us = vehicle.s > car_s;
              bool is_closer_than_safety_margin = vehicle.s - car_s < safety_margin;

              if (is_in_front_of_us && is_closer_than_safety_margin){
                front_car_too_close = true;
                prepare_for_lane_change = true;
              }
            }
          }//end for iteration over all sensor Fusion

          //if a car in front, slowly down without collision with front car
          if (front_car_too_close){ref_vel -= 0.224;}
          //if there isn't a front car and not above max speed
          else if (ref_vel < max_speed){ref_vel += 0.224;}
          std::cout << ref_vel << std::endl;

          //prepare for lane change
          if (prepare_for_lane_change){
            int num_vehicles_left = 0;
            int num_vehicles_right = 0;
            //check if left and right lanes are free
            for (size_t i = 0; i < sensor_fusion.size(); ++i){
              Vehicle vehicle(sensor_fusion[i]);
              //check left lane
              if (is_in_same_lane(vehicle.d, intend_lane - 1)){
                ++num_vehicles_left;
                vehicle.s += (double)prev_path_size * 0.02 * vehicle.speed;
                //**if there is a car is too close, set the entire condition to false
                bool too_close_to_change = abs(car_s - vehicle.s) < safety_margin;
                if (too_close_to_change){is_left_lane_free = false;}
              }
              //check right lane
              else if (is_in_same_lane(vehicle.d, intend_lane + 1)){
                ++num_vehicles_right;
                vehicle.s += (double)prev_path_size * 0.02 * vehicle.speed;
                bool too_close_to_change = abs(car_s - vehicle.s) < safety_margin;
                if (too_close_to_change){is_right_lane_free = false;}
              }
            }//finishing check left and right lanes
            //if either left or right lane is free, set to true
            if (is_left_lane_free || is_right_lane_free){ready_for_lane_change = true;}
            std::cout << "left: " << num_vehicles_left
                      << "  right " << num_vehicles_right << std::endl;
          }//end for prepare for lane change

          //change lanes
          if (ready_for_lane_change && is_left_lane_free){intend_lane -= 1;}
          else if (ready_for_lane_change && is_right_lane_free){intend_lane += 1;}

      



          // List of widely spaced (x, y) waypoints. These will be later interpolated
    			// with a spline, filling it with more points which control speed.
    			vector<double> pts_x;
    			vector<double> pts_y;

    			// Reference x, y, yaw state
    			double ref_x = car_x;
    			double ref_y = car_y;
    			double ref_yaw = deg2rad(car_yaw);

          //calculate history trajectory, minimise jerk and acceleration

          //If there is no path computed or prev computed path is exhausted
              //use current data and infer history data
          //if prev_path_size is too small, prev_car_x will not be useful
          if (prev_path_size < 2){
            //use formula to infer previous point
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            pts_x.push_back(prev_car_x); pts_x.push_back(car_x);
            pts_y.push_back(prev_car_y); pts_y.push_back(car_y);
          }
          //if there is previous point exists
              //use prev data [-1, -2] as history data
          else{
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];
            double ref_x_prev = previous_path_x[prev_path_size - 2];
            double ref_y_prev = previous_path_y[prev_path_size - 2];
            double ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            pts_x.push_back(ref_x_prev); pts_x.push_back(ref_x);
            pts_y.push_back(ref_y_prev); pts_y.push_back(ref_y);
          }

          // In Frenet coordinates, add evenly 30m spaced points ahead of the starting reference
          // getXY(s, d, ...): d = lane_number * lane_width + lane_width / 2
    			vector<double> next_wp0 = getXY(car_s + 30, (lane_width * intend_lane + lane_width / 2),
                                                        map_waypoints_s, map_waypoints_x, map_waypoints_y);
    			vector<double> next_wp1 = getXY(car_s + 60, (lane_width * intend_lane + lane_width / 2),
                                                        map_waypoints_s, map_waypoints_x, map_waypoints_y);
    			vector<double> next_wp2 = getXY(car_s + 90, (lane_width * intend_lane + lane_width / 2),
                                                        map_waypoints_s, map_waypoints_x, map_waypoints_y);

    			pts_x.push_back(next_wp0[0]); pts_x.push_back(next_wp1[0]); pts_x.push_back(next_wp2[0]);
    			pts_y.push_back(next_wp0[1]); pts_y.push_back(next_wp1[1]); pts_y.push_back(next_wp2[1]);

          // Rototranslate into car's reference system to make the math easier
    			for (size_t i = 0; i < pts_x.size(); ++i) {
    				double shift_x = pts_x[i] - ref_x;
    				double shift_y = pts_y[i] - ref_y;
    				pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    				pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    			}

          // Create a spline
          tk::spline s;
          s.set_points(pts_x, pts_y);

          // Define the actual (x, y) points will be used for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all previous points from last time
    			for (size_t i = 0; i < prev_path_size; ++i) {
    				next_x_vals.push_back(previous_path_x[i]);
    				next_y_vals.push_back(previous_path_y[i]);
    			}

          // Calculate how to break up spline points to travel at ref velocity
          double target_x     = 30.0; //in meters
          double target_y     = s(target_x);
          double target_dist  = sqrt(target_x*target_x + target_y*target_y);
          // Find number of split points to get desired speed
          // 0.02s for each point
          // 2.24 to convert mph into meters
          double N            = target_dist / (0.02 * ref_vel / 2.24);

          double x_point_car_coord = 0.0;
          double y_point_car_coord;
          for (size_t i = 0; i < 50 - prev_path_size; ++i){

            x_point_car_coord += target_x / N;
            y_point_car_coord = s(x_point_car_coord);

            // Retransformation
            double x_point_global_coord;
            double y_point_global_coord;
            x_point_global_coord = (x_point_car_coord * cos(ref_yaw) - 
                                        y_point_car_coord * sin(ref_yaw)) + ref_x;
            y_point_global_coord = (y_point_car_coord * sin(ref_yaw) + 
                                        y_point_car_coord * cos(ref_yaw)) + ref_y;
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
