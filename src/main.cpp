#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using namespace std;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid,pid_thr;
  const double max_steering_angle =1;
  const double MAX_SPEED=40.0; //MAX SPEED 
  pid.Init(0.2, 0.004, 3.0);  // Init PID parameters
  pid_thr.Init(0.316731, 0.0000, 0.0226185);
  h.onMessage([&pid, &pid_thr, &MAX_SPEED,&max_steering_angle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
        double cte = std::stod(j[1]["cte"].get<std::string>());
	double speed = std::stod(j[1]["speed"].get<std::string>());
	double angle = std::stod(j[1]["steering_angle"].get<std::string>());
	double steer_value;
					           
	// Update error values with cte
	pid.UpdateError(cte);                     
	// Calculate steering value (if reasonable error, returns between [-
	steer_value = pid.TotalError();
	// DEBUG				                                                                 
	std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
	if (steer_value > max_steering_angle)
		steer_value = max_steering_angle;
	if (steer_value < -max_steering_angle)
		steer_value = -max_steering_angle;
	// Setting throttle with same PID controller as steer val
	// Formula below switches to between [0, 1], larger steering angle means less throttle
	// Multiplied by 0.5 for safety reasons
	pid_thr.UpdateError(fabs(cte));
       	double	t_str_val =pid_thr.TotalError();
	double thr = (1 - std::abs(t_str_val)) * 0.5 + 0.2;
	std::cout << "thr: " << thr << " Steering Value: " << steer_value << std::endl;
	if(speed>MAX_SPEED){
		thr-=0.2;
	}
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = thr;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
