#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

#define ms2sec 0.001f
// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  double pre_steer_value=0;
  double pre_throttle_value=0;
  h.onMessage([&mpc,&pre_steer_value,&pre_throttle_value](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
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
          double steer_value=0;
          double throttle_value=0;
          // This is the length from front to CoG that has a similar radius.
          const double Lf = 2.67;
          int latancy_ms = 100;
     
          // Transforming incomming data to Vehicle Coordinates
          double shift_x , shift_y,rotated_x,rotated_y;
          vector<double> ptsx_veh , ptsy_veh;
          for(size_t i=0;i<ptsx.size();i++)
          {
            shift_x = ptsx[i] - px;
            shift_y = ptsy[i] - py;

            rotated_x = shift_x* cos(-psi) - shift_y * sin(-psi);
            rotated_y = shift_x* sin(-psi) + shift_y * cos(-psi);
            ptsx_veh.push_back(rotated_x);
            ptsy_veh.push_back(rotated_y);
          }
          double* ptr_x = &ptsx_veh[0];
          double* ptr_y = &ptsy_veh[0];

          Eigen::Map<Eigen::VectorXd> ptsx_v(ptr_x, ptsx_veh.size());
          Eigen::Map<Eigen::VectorXd> ptsy_v(ptr_y, ptsy_veh.size());

          // TODO: fit a polynomial to the above x and y coordinates
          auto coeffs = polyfit(ptsx_v, ptsy_v, 3);

          // TODO: calculate the orientation error
          double epsi = - atan(coeffs[1]); 
          // TODO: calculate the cross track error
          double cte = (polyeval(coeffs, 0));
          
          // handling Controller Latancy
          // using same kinematic model with assumption that no change in steering angle 
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          double px_predicted = v * cos(psi) * (latancy_ms*ms2sec);
          // double py_predicted = v * sin(psi) * (latancy_ms*ms2sec); // Didn't used as lateral forces are not modeled
          double v_actual = v + pre_throttle_value * (latancy_ms*ms2sec);
          double epsi_actual = epsi + v / Lf * pre_steer_value * (latancy_ms*ms2sec);
          double cte_actual = cte + v * sin(epsi) * (latancy_ms*ms2sec);
          Eigen::VectorXd state(6);
          state << px_predicted, 0, 0, v_actual, cte_actual, epsi_actual;

          auto vars = mpc.Solve(state, coeffs);

          
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          steer_value = vars[0] / deg2rad(25.0) / Lf;
          throttle_value = vars[1];
          // cout << "steer "<<steer_value<<" throttle_value "<<throttle_value<<" pre_steer_value "
          // <<pre_steer_value<<" pre_throttle_value "<<pre_throttle_value<<endl;

          pre_steer_value = steer_value;
          pre_throttle_value = throttle_value;
          
          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          size_t x_start = 3;
          size_t y_start = vars[2]+x_start;

          for(size_t i=x_start;i<y_start;i++)
          {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+vars[2]]);
          }

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(size_t i=0;i<10;i++)
          {
            double x_stride = i * 2;
            next_x_vals.push_back(x_stride);
            next_y_vals.push_back(polyeval(coeffs, x_stride));
          }

          
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // past_steer_value = steer_value;
          // past_throttle_value = throttle_value;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(latancy_ms));
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
