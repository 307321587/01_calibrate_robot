#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <cmath>

using namespace std;

<<<<<<< HEAD
#define SERVER_HOST "192.168.31.6"
=======
#define SERVER_HOST "192.168.123.96"
>>>>>>> 0146bb0e4f0d451665d5804e32101ed06dec47b8
#define SERVER_PORT 8899

#define FILE_PATH                                                    \
    "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/" \
    "record_line.offt"

#define ROAD_POINT_RELOAD_SIZE 4 // 每一次下发的路点个数
#define MAC_FIFO_FILL (6 * 15)   // 缓冲路点个数 * 6

class TrajectoryIo
{
public:
    TrajectoryIo(const char *filename) { input_file_.open(filename, std::ios::in); }
    ~TrajectoryIo() { input_file_.close(); }

    std::vector<std::vector<double>> parse()
    {
        std::vector<std::vector<double>> res;
        std::string tmp;
        int linenum = 1;
        while (std::getline(input_file_, tmp, '\n'))
        {
            try
            {
                auto q = split(tmp, ",");
                res.push_back(q);
            }
            catch (const char *p)
            {
<<<<<<< HEAD
                std::cerr << "Line: " << linenum << " \"" << p << "\"" << " is not a number of double" << std::endl;
=======
                std::cerr << "Line: " << linenum << " \"" << p << "\""
                          << " is not a number of double" << std::endl;
>>>>>>> 0146bb0e4f0d451665d5804e32101ed06dec47b8
                break;
            }
            linenum++;
        }
        return res;
    }

    std::vector<double> split(const std::string &str, const char *delim)
    {
        std::vector<double> res;
        if ("" == str)
        {
            return res;
        }
        // 先将要切割的字符串从string类型转换为char*类型
        char *strs = new char[str.length() + 1]; // 不要忘了
        std::strcpy(strs, str.c_str());

        char *p = std::strtok(strs, delim);
        char *endp = nullptr;
        while (p)
        {
            double v = std::strtod(p, &endp);
            if (endp[0] != 0 && endp[0] != '\r')
            {
                delete[] strs;
                strs = nullptr;
                throw p;
            }
            res.push_back(v); // 存入结果数组
            p = std::strtok(nullptr, delim);
        }

        if (strs)
        {
            delete[] strs;
            strs = nullptr;
        }

        return res;
    }

private:
    std::ifstream input_file_;
};

int main()
{
    // 0. Read waypoint file
    TrajectoryIo input(FILE_PATH);

    auto traj = input.parse();
    auto traj_sz = traj.size();
<<<<<<< HEAD
    if (traj_sz == 0)
    {
        std::cerr << "No waypoints" << std::endl;
        return -1;
    }
=======
    // if (traj_sz == 0)
    // {
    //     std::cerr << "No waypoints" << std::endl;
    //     return -1;
    // }
>>>>>>> 0146bb0e4f0d451665d5804e32101ed06dec47b8

    ServiceInterface robotService;

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
        return -1;
    }

    // Avoid not leave tcp2canbus mode last time
    robotService.robotServiceLeaveTcp2CanbusMode();

    // 3. Move joint to home point
    std::cout << "Moving to work point..." << std::endl;
    double wp1[6] = {-90 * M_PI / 180, 0, 90 * M_PI / 180, 0, 90 * M_PI / 180, 0};
    robotService.robotServiceJointMove(wp1, true);
    std::cout << "Move to work point succ." << std::endl;

    // 获取路点信息
    // 获取机械臂当前的路点信息
    aubo_robot_namespace::wayPoint_S currentWaypoint;

    robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    for (auto joint_angle : currentWaypoint.jointpos)
    {
        std::cout << "joint_angle: " << joint_angle << std::endl;
    }
    std::cout << "--------------------当前路点正解--------------------" << std::endl;
    std::cout << "正解得到的路点位置："
              << " x = " << currentWaypoint.cartPos.position.x << " y = " << currentWaypoint.cartPos.position.y << " z = " << currentWaypoint.cartPos.position.z << std::endl;
    std::cout << "正解得到的目标路点的姿态（四元数）："
              << " w = " << currentWaypoint.orientation.w << " x = " << currentWaypoint.orientation.x << " y = " << currentWaypoint.orientation.y << " z = " << currentWaypoint.orientation.z
              << std::endl;
    // 四元数转欧拉角
    aubo_robot_namespace::Rpy rpy;
    robotService.quaternionToRPY(currentWaypoint.orientation, rpy);
    std::cout << "正解得到的目标路点的姿态（欧拉角）:"
              << " RX = " << rpy.rx * 180 / M_PI << " RY = " << rpy.ry * 180 / M_PI << " RZ = " << rpy.rz * 180 / M_PI << std::endl;

    // 根据当前路点的关节角，正解得到目标路点
    aubo_robot_namespace::wayPoint_S targetWaypoint;
    ret = robotService.robotServiceRobotFk(currentWaypoint.jointpos, aubo_robot_namespace::ARM_DOF, targetWaypoint);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "--------------------正解--------------------" << std::endl;
        std::cout << "正解得到的路点位置："
                  << " x = " << targetWaypoint.cartPos.position.x << " y = " << targetWaypoint.cartPos.position.y << " z = " << targetWaypoint.cartPos.position.z << std::endl;
        std::cout << "正解得到的目标路点的姿态（四元数）："
                  << " w = " << targetWaypoint.orientation.w << " x = " << targetWaypoint.orientation.x << " y = " << targetWaypoint.orientation.y << " z = " << targetWaypoint.orientation.z
                  << std::endl;
        // 四元数转欧拉角
        aubo_robot_namespace::Rpy rpy;
        robotService.quaternionToRPY(targetWaypoint.orientation, rpy);
        std::cout << "正解得到的目标路点的姿态（欧拉角）:"
                  << " RX = " << rpy.rx * 180 / M_PI << " RY = " << rpy.ry * 180 / M_PI << " RZ = " << rpy.rz * 180 / M_PI << std::endl;
    }
    else
    {
        std::cerr << "调用正解函数失败" << std::endl;
    }
    // 根据正解得到的位置姿态，来获取逆解集
    aubo_robot_namespace::Pos position = targetWaypoint.cartPos.position; // 正解得到的路点的位置
    position.x = position.x + 0.1;
    aubo_robot_namespace::Ori orientation = targetWaypoint.orientation; // 正解得到的路点的姿态
    std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;
    ret = robotService.robotServiceRobotIk(position, orientation, wayPointVector);
    std::cout << std::endl;
    std::cout << "--------------------逆解集----------------------" << std::endl;
    std::cout << "逆解集的大小： " << wayPointVector.size() << std::endl;
    for (int i = 0; i < wayPointVector.size(); i++)
    {
        std::cout << "第" << i + 1 << "组解：" << std::endl;
        for (int j = 0; j < 6; j++)
        {
            std::cout << "关节" << j + 1 << ": " << wayPointVector[i].jointpos[j] * 180 / M_PI << std::endl;
        }
    }
    // 根据当前路点，获取最优逆解
    aubo_robot_namespace::wayPoint_S wayPoint;
    robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    ret = robotService.robotServiceRobotIk(currentWaypoint.jointpos, position, orientation, wayPoint);
    std::cout << std::endl;
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "----------------------最优逆解-----------------------" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << "关节" << i + 1 << ": " << wayPoint.jointpos[i] * 180 / M_PI << std::endl;
        }
    }
    else
    {
        std::cerr << "调用逆解函数失败" << std::endl;
    }

    robotService.robotServiceLogout();
    return 0;
}
