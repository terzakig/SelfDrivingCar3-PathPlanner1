#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"
#include "Planner.h"

using namespace std;
using namespace tk;




// for convenience
using json = nlohmann::json;






// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}



Planner planner;



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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  vector<VehicleTracker> trackers;
  
  
  h.onMessage([&planner, &trackers, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          	double car_yaw_deg = j[1]["yaw"];
          	double car_yaw = deg2rad(car_yaw_deg);
		
		double car_speed_MPH = j[1]["speed"];
		double car_speed = 0.44704 * car_speed_MPH;
		
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	
		// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	
		
		
		
		vector<double> next_x_vals;
          	vector<double> next_y_vals;

		
		
		
		// A. Adding the ENTIRE previous plan residual (WHY the whole thing???? Ask the instructors...................)
		//                                               probably because its small enough to be devoured early on...
		for (int i = 0; i < previous_path_x.size(); i++)
		{
		  next_x_vals.push_back(previous_path_x[i]);
		  next_y_vals.push_back(previous_path_y[i]);
		}
		
		// B. Updater the tracker list
		if (trackers.size() == 0)
		{
		  VehicleTracker tracker;
		  for (int i = 0; i < sensor_fusion.size(); i++)
		  {
		    tracker.id = (int)sensor_fusion[i][0];
		    tracker.x = (double)sensor_fusion[i][1];
		    tracker.y = (double)sensor_fusion[i][2];
		    tracker.vx = (double)sensor_fusion[i][3];
		    tracker.vy = (double)sensor_fusion[i][4];
		    tracker.s = (double)sensor_fusion[i][5];
		    tracker.d = (double)sensor_fusion[i][6];
		    tracker.speed = sqrt(tracker.vx * tracker.vx + tracker.vy * tracker.vy);
		    
		    trackers.push_back(tracker);
		  }
		}
		else
		{
		  for (int i = 0; i < sensor_fusion.size(); i++)
		  {
		    trackers[i].id = (int)sensor_fusion[i][0];
		    trackers[i].x = (double)sensor_fusion[i][1];
		    trackers[i].y = (double)sensor_fusion[i][2];
		    trackers[i].vx = (double)sensor_fusion[i][3];
		    trackers[i].vy = (double)sensor_fusion[i][4];
		    trackers[i].s = (double)sensor_fusion[i][5];
		    trackers[i].d = (double)sensor_fusion[i][6];
		    trackers[i].speed = sqrt(trackers[i].vx * trackers[i].vx + trackers[i].vy * trackers[i].vy);
		  
		  }
		}
		
		// C. Plannig
		pair<vector<double>, vector<double>> plan = planner.OptimalPolicy(trackers, 
										 previous_path_x,
										 previous_path_y,
										 car_x,
										 car_y,
										 car_speed,
										 car_yaw,
										 car_s,
									         car_d,
										 map_waypoints_s,
										 map_waypoints_x,
										 map_waypoints_y
										);
		
		
		// adding the plan top the previous
		for (int i = 0; i < plan.first.size(); i++)
		{
		  next_x_vals.push_back(plan.first[i]);
		  next_y_vals.push_back(plan.second[i]);
		}
		
		/* This is my solution using the approximate arc length. It is MORE CORRECT, but it fails keeping jerk tamed...
		 * 
		 */
		/*spline spline_x, spline_y;
		//cout<<"The Y%&R&IR arc length : "<<endl;
		//printVector(arc_length);
		
		spline_x.set_points(arc_length, points_x);
		spline_y.set_points(arc_length, points_y);
		
		double delta_l = planner.Dt * planner.target_speed;
		for (int i = 1; i < 50 - previous_path_x.size(); i++)
		{
		  double l = i*delta_l;
		   next_x_vals.push_back( spline_x(l) );
		  next_y_vals.push_back( spline_y(l) );
		  
		}*/
		
				
		// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
















































































