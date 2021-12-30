/*
 * This file is based on the one in
 * https://github.com/udacity/CarND-MPC-Project/tree/master/src.
 * It ensures communication between the simulator and the MPC.
 * What is modified is parsing the data from the simulator, feeding it to the
 * MPC, taking the output of MPC, and sending it back to the simulator.
 */
#include <uWS/uWS.h> // Make sure this comes before the rest.

#include "Eigen/Core"
#include "Eigen/QR"
#include "MPC.h"
#include "helpers.h"
#include "json.hpp"
#include <chrono>
#include <cppad/cppad.hpp>
#include <iostream>
#include <math.h>
#include <string>
#include <thread>
#include <vector>

using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data,
                       size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message
        // event. The 4 signifies a websocket message The 2 signifies a
        // websocket event
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

                    // Nominal inputs:
                    double steer = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];

                    // Now we have obtained the data.
                    // Next convert (ptsx,ptsy) to vehicle coords:
                    for (int i = 0; i < ptsx.size(); i++) {
                        double diffx = ptsx[i] - px;
                        double diffy = ptsy[i] - py;
                        ptsx[i] = diffx * cos(psi) + diffy * sin(psi);
                        ptsy[i] = diffy * cos(psi) - diffx * sin(psi);
                    }
                    // Convert from std::vector to Eigen::Vector:
                    Eigen::VectorXd ptsxV =
                        Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
                    Eigen::VectorXd ptsyV =
                        Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
                    // Fit a 3rd order polynomial to generate reference
                    // trajectory:
                    Eigen::VectorXd coeffs = polyfit(ptsxV, ptsyV, 3);

                    // Construct current state:
                    Eigen::VectorXd state(6);

                    const double Lf = 2.67;
                    // latency=0.1s
                    double dt = 0.1;
                    double px_l = v * dt;
                    double py_l = 0.0;
                    double psi_l = -v * steer / Lf * dt;
                    double v_l = v + throttle * dt;
                    double cte_l = polyeval(coeffs, 0) +
                                   v * CppAD::sin(-atan(coeffs[1])) * dt;
                    double epsi_l = -atan(coeffs[1]) + psi_l;
                    state << px_l, py_l, psi_l, v_l, cte_l, epsi_l;

                    // Calculate steering angle and throttle using MPC.
                    // NOTE: Remember to divide by deg2rad(25) before you send
                    // the steering value back. Otherwise the values will be in
                    // between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    std::vector<double> mpcOut = mpc.Solve(state, coeffs);
                    double steer_value = mpcOut[0] / (deg2rad(25) * Lf);
                    double throttle_value =
                        mpcOut[1] * (1 - fabs(steer_value)) + 0.1;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    // Display the MPC predicted trajectory
                    int num_dipslay = (mpcOut.size() - 2) / 2;
                    vector<double> mpc_x_vals(num_dipslay);
                    vector<double> mpc_y_vals(num_dipslay);
                    int k = 2; // index of mpcOut, mpcOut[0,1] = input
                    for (int i = 0; i < num_dipslay; i++) {
                        mpc_x_vals[i] = mpcOut[k];
                        mpc_y_vals[i] = mpcOut[k + 1];
                        k += 2;
                    }
                    // Send points back
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    // Display the waypoints/reference line
                    int num_points = 25; // waypoints to display
                    vector<double> next_x_vals(num_points);
                    vector<double> next_y_vals(num_points);
                    double poly_inc = 2.5;
                    for (int i = 0; i < num_points; i++) {
                        next_x_vals[i] = poly_inc * i;
                        next_y_vals[i] = polyeval(coeffs, poly_inc * i);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    //   the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to
                    // drive
                    //   around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                } // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
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