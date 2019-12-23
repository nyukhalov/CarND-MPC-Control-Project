#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void convert_to_vehicle_coords(
    const vector<double>& ptsx,
    const vector<double>& ptsy,
    double veh_x, double veh_y, double veh_heading,
    vector<double>& veh_ptsx,
    vector<double>& veh_ptsy) {
  assert(ptsx.size() == ptsy.size());

  double alpha = -veh_heading;
  for (int i=0; i<ptsx.size(); i++) {
    double x1 = ptsx[i] - veh_x;
    double y1 = ptsy[i] - veh_y;

    double x2 = cos(alpha) * x1 - sin(alpha) * y1;
    double y2 = sin(alpha) * x1 + cos(alpha) * y1;

    veh_ptsx[i] = x2;
    veh_ptsy[i] = y2;
  }
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          vector<double> veh_ptsx(ptsx.size());
          vector<double> veh_ptsy(ptsy.size());
          convert_to_vehicle_coords(ptsx, ptsy, px, py, psi, veh_ptsx, veh_ptsy);

          VectorXd state(4);
          state << px, py, psi, v;

          VectorXd v_ptsx = VectorXd::Map(ptsx.data(), ptsx.size());
          VectorXd v_ptsy = VectorXd::Map(ptsy.data(), ptsy.size());
          VectorXd coeffs = polyfit(v_ptsx, v_ptsy, 3);

          MpcSolution solution = mpc.solve(state, coeffs);

          json msgJson;
          msgJson["steering_angle"] = solution.steering / deg2rad(25);
          msgJson["throttle"] = solution.throttle;
          // predicted MPC trajectory
          msgJson["mpc_x"] = solution.trajectory.ptsx;
          msgJson["mpc_y"] = solution.trajectory.ptsy;
          // reference line
          msgJson["next_x"] = veh_ptsx;
          msgJson["next_y"] = veh_ptsy;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
