#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

double run(double Kp, double Ki, double Kd);

bool twiddle = false;
double Kp = 0.24;
double Ki = 0.0012;
double Kd = 3.6;

// Works well on windows not on linux
//Throttle 0.6
//Kp: 0.150141,  Ki: 7.78939e-05,  Kd: 3.68184


double twiddle_tol = 0.002;
int twiddle_counter = 1200;

double best_err;

int main() {
  if (twiddle){

    double params [3] = {Kp, Ki, Kd};
    double dp [3] = {params[0]*0.2, params[1]*0.2, params[2]*0.2};

    run(Kp, Ki, Kd);
    best_err = run(Kp, Ki, Kd);
    std::cout << "New best error: " << best_err << std::endl;

    double err = 0.0;
    while (dp[0] + dp[1] + dp[2] > twiddle_tol) {
      for (int i=0; i < 3; i++){
        params[i] +=  dp[i];
        std::cout << "Trying: " << "Kp: " << params[0] << ",  Ki: " << params[1] << ",  Kd: " << params[2] << std::endl;
        err = run(params[0], params[1], params[2]);
        if (err < best_err){
          best_err = err;
          dp[i] *= 1.1;
          std::cout << "New best error: " << err << std::endl;
          std::cout << "Kp: " << params[0] << ",  Ki: " << params[1] << ",  Kd: " << params[2] << std::endl;
        } else {
          params[i] -= 2 * dp[i];
          std::cout << "Trying: " << "Kp: " << params[0] << ",  Ki: " << params[1] << ",  Kd: " << params[2] << std::endl;
          err = run(params[0], params[1], params[2]);
          if (err < best_err){
            best_err = err;
            dp[i] *= 1.1;
            std::cout << "New best error: " << err << std::endl;
            std::cout << "Kp: " << params[0] << ",  Ki: " << params[1] << ",  Kd: " << params[2] << std::endl;
          } else{
            params[i] += dp[i];
            dp[i] *= 0.9;
          }
        }
      }
    }
    std::cout << "Twiddle finished!" << std::endl;
  } else {
    run(Kp, Ki, Kd);
  }
}

double run(double Kp, double Ki, double Kd) {
  int counter = -30;

  PID pid;
  pid.Init(Kp, Ki, Kd);

  double total_error = 0.0;

  uWS::Hub h;
  h.onMessage([&pid, &counter, &total_error, &h](uWS::WebSocket<uWS::SERVER> ws,
                                                 char *data,
                                                 size_t length,
                                                 uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          if (counter < 0){
            counter++;
            json msgJson;
            msgJson["steering_angle"] = 0;
            msgJson["throttle"] = 0;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
          }
          if (counter > twiddle_counter/10) {
            total_error += pow(cte, 2);
          }
          pid.UpdateError(cte);
          double steer_value = pid.TotalError();

          counter++;
          if (twiddle && counter > twiddle_counter) {
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            h.uWS::Group<uWS::SERVER>::close();
            return;
          }

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << "CTE: " << cte << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.4;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    //ws.close();
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
  return total_error;
}
