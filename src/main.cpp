#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}

double deg2rad(double x)
{
    return x * pi() / 180;
}

double rad2deg(double x)
{
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if(found_null != std::string::npos)
    {
        return "";
    }
    else if(b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    PID pid_steering_controller;

    // Initialize the pid variable.
    double Kp = 0.1;
    double Ki = 0.00;
    double Kd = 10.0;
    pid_steering_controller.Init(Kp, Ki, Kd);

    h.onMessage([&pid_steering_controller]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if(length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto json_message = hasData(std::string(data).substr(0, length));
            if(json_message != "")
            {
                auto j = json::parse(json_message);
                std::string event = j[0].get<std::string>();

                if(event == "telemetry")
                {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;
                    double total_error;

                    // Calculate the steering value; remember the steering value is in the range [-1, 1]
                    steer_value = pid_steering_controller.UpdateError(cte);
                    total_error = pid_steering_controller.TotalError(cte);

                    std::cout
                              << " P: " << pid_steering_controller.ParameterVector()[0]
                              << " I: " << pid_steering_controller.ParameterVector()[2]
                              << " D: " << pid_steering_controller.ParameterVector()[1]
                              << "\tCTE: " << cte
                              << "\tTotal Error: " << total_error
                              << "\tSpeed: " << speed
                              << "\tSteer Value: " << steer_value
                              << "\tSteering angle: " << angle
                              << std::endl;


                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.50;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });


    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if(h.listen(port))
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
