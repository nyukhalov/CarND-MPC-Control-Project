#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <math.h>

#include <uWS/uWS.h>
#include <opencv2/opencv.hpp>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "mpc/MPC.h"
#include "ui/window.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace carnd;

void convert_to_vehicle_coords(
    const vector<double> &ptsx,
    const vector<double> &ptsy,
    double veh_x, double veh_y, double veh_heading,
    vector<double> &veh_ptsx,
    vector<double> &veh_ptsy)
{
  assert(ptsx.size() == ptsy.size());

  double alpha = -veh_heading;
  for (int i = 0; i < ptsx.size(); i++)
  {
    double x1 = ptsx[i] - veh_x;
    double y1 = ptsy[i] - veh_y;

    double x2 = cos(alpha) * x1 - sin(alpha) * y1;
    double y2 = sin(alpha) * x1 + cos(alpha) * y1;

    veh_ptsx[i] = x2;
    veh_ptsy[i] = y2;
  }
}

MPC init_mpc()
{
  const double target_vel = 50;

  // prediction horizon settings
  const size_t num_states = 10;
  const double dt = 0.1;

  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on
  // a flat terrain.
  //
  // The value was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius
  const double lf = 2.67;

  MPCConfig config(num_states, dt, target_vel, lf);

  std::cout << "Creating MPC with config: " << config << std::endl;
  MPC mpc(config);

  return mpc;
}

class PointTransformer
{
public:
  PointTransformer(double scale_x, double scale_y, size_t window_height, cv::Point origin) : _scale_x(scale_x),
                                                                                             _scale_y(scale_y),
                                                                                             _window_height(window_height),
                                                                                             _origin(origin)
  {
  }

  cv::Point transform(double x, double y) const
  {
    double loc_x = _scale_x * x;
    double loc_y = _scale_y * y;
    double px = _origin.x - loc_y;
    double py = _window_height - _origin.y - loc_x;
    return cv::Point(px, py);
  }

private:
  const double _scale_x;
  const double _scale_y;
  const size_t _window_height;
  const cv::Point _origin;
};

int main()
{
  uWS::Hub h;

  Window window(500, 500, "Main");
  double scale_x = 8;
  double scale_y = scale_x;
  double x_offset = 0.5 * window.width;
  double y_offset = 0.1 * window.height;
  cv::Point origin = cv::Point(x_offset, y_offset);
  PointTransformer pt(scale_x, scale_y, window.height, origin);

  const MPC mpc = init_mpc();

  auto cb_last = std::chrono::steady_clock::now();
  h.onMessage([&mpc, &cb_last, &window, &pt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                        uWS::OpCode opCode) {
    auto cb_elapsed = std::chrono::steady_clock::now() - cb_last;
    cb_last = std::chrono::steady_clock::now();
    std::cout << "Callback took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(cb_elapsed).count()
              << "ms"
              << std::endl;

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
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

          VectorXd v_ptsx = VectorXd::Map(veh_ptsx.data(), veh_ptsx.size());
          VectorXd v_ptsy = VectorXd::Map(veh_ptsy.data(), veh_ptsy.size());
          VectorXd coeffs = polyfit(v_ptsx, v_ptsy, 3);

          // (0, 0) as we converted waypoints to the vehicles coordinate system
          double v_px = 0;
          double v_py = 0;
          double v_psi = 0;

          VectorXd state(4);
          state << v_px, v_py, v_psi, v;

          auto mpc_clock_begin = std::chrono::steady_clock::now();
          MpcSolution solution = mpc.solve(state, coeffs);
          auto mpc_elapsed = std::chrono::steady_clock::now() - mpc_clock_begin;
          std::cout << "MPC solver took "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(mpc_elapsed).count()
                    << "ms"
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = -solution.steering / deg2rad(25);
          msgJson["throttle"] = solution.throttle;

          // predicted MPC trajectory
          std::vector<double> mpc_ptsx;
          std::vector<double> mpc_ptsy;
          for (const Pose& pose : solution.trajectory) {
            mpc_ptsx.push_back(pose.x);
            mpc_ptsy.push_back(pose.y);
          }
          msgJson["mpc_x"] = mpc_ptsx;
          msgJson["mpc_y"] = mpc_ptsy;

          // reference waypoints
          vector<double> veh_ptsx2;
          vector<double> veh_ptsy2;
          for (double x = 0; x < 100; x += 4)
          {
            veh_ptsx2.push_back(x);
            veh_ptsy2.push_back(polyeval(coeffs, x));
          }
          msgJson["next_x"] = veh_ptsx2;
          msgJson["next_y"] = veh_ptsy2;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // drawing the current trajectories
          double ref_node_radius = 2;
          double orig_node_radius = 4;
          double mpc_node_radius = 8;

          // the colors are in BGR
          cv::Scalar ref_node_color = cv::Scalar(200, 255, 255);
          cv::Scalar orig_node_color = cv::Scalar(0, 0, 255);
          cv::Scalar heading_color = cv::Scalar(0, 0, 255);
          for (int i = 0; i < veh_ptsx2.size(); i++)
          {
            cv::Point point = pt.transform(veh_ptsx2[i], veh_ptsy2[i]);
            window.circle(point, ref_node_radius, ref_node_color);
          }
          for (int i=0; i<v_ptsx.size(); i++)
          {
            cv::Point point = pt.transform(v_ptsx[i], v_ptsy[i]);
            window.circle(point, orig_node_radius, orig_node_color);
          }
          for (const Pose& pose : solution.trajectory)
          {
            double vel_progress = 1.0 - (mpc.config.target_vel - pose.velocity) / mpc.config.target_vel;
            cv::Scalar mpc_node_color = cv::Scalar(0, vel_progress * 255, (1 - vel_progress) * 255);
            double heading = M_PI + M_PI/2 - pose.heading;
            cv::Point point = pt.transform(pose.x, pose.y);
            cv::Point heading_point(
              point.x + mpc_node_radius*cos(heading),
              point.y + mpc_node_radius*sin(heading)
            );
            window.circle(point, mpc_node_radius, mpc_node_color, -1);
            window.line(point, heading_point, heading_color);
          }
          string status = solution.success ? "OK" : "FAIL";
          cv::Scalar status_color = solution.success ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
          cv::Point status_point(30, 50);
          window.text(status, status_point, status_color);
          window.draw();

          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          // std::this_thread::sleep_for(std::chrono::milliseconds(100));

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                       char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
