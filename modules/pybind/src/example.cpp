#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <vector>

using namespace std;

#define SERVER_HOST "192.168.31.6"
#define SERVER_PORT 8899
ServiceInterface robotService;

bool init()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    // 1. Login
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "Login succ." << std::endl;
    }
    else
    {
        std::cerr << "Login failed." << std::endl;
        return -1;
    }

    aubo_robot_namespace::RobotWorkMode mode = aubo_robot_namespace::RobotModeSimulator;
    robotService.robotServiceGetRobotWorkMode(mode);

    // 2. Startup
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam /**Tool dynamics parameter**/, 6 /*Collision level*/, true /*Whether to allow reading poses defaults to true*/,
                                               true,    /*Leave the default to true */
                                               1000,    /*Leave the default to 1000 */
                                               result); /*Robot arm initialization*/
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "Robot arm initialization succ." << std::endl;
    }
    else
    {
        std::cerr << "Robot arm initialization failed." << std::endl;
        return false;
    }
    return true;
}

void movej(vector<double> joint)
{
    if (joint.size() == 6)
    {
        double wp1[6] = {joint[0] * M_PI / 180, joint[1] * M_PI / 180, joint[2] * M_PI / 180, joint[3] * M_PI / 180, joint[4] * M_PI / 180, joint[5] * M_PI / 180};
        robotService.robotServiceJointMove(wp1, true);
    }
}

vector<double> get_status()
{
    aubo_robot_namespace::wayPoint_S currentWaypoint;
    robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    aubo_robot_namespace::Rpy rpy;
    robotService.quaternionToRPY(currentWaypoint.orientation, rpy);
    vector<double> status{currentWaypoint.cartPos.position.x, currentWaypoint.cartPos.position.y, currentWaypoint.cartPos.position.z, rpy.rx * 180 / M_PI, rpy.ry * 180 / M_PI, rpy.rz * 180 / M_PI};
    return status;
}

void log_out() { robotService.robotServiceLogout(); }

PYBIND11_MODULE(robot, m)
{
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("init", &init, "A function that adds two numbers");
    m.def("movej", &movej, "A function that adds two numbers");
    m.def("get_status", &get_status, "A function that adds two numbers");
    m.def("log_out", &log_out, "A function that adds two numbers");
}