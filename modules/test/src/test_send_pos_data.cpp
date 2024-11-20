#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>

using namespace std;

#define SERVER_HOST "192.168.123.96"
#define SERVER_PORT 8899

#define FILE_PATH                               \
    "/home/lza/code/01_calibrate_robot/record/" \
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
                std::cerr << "Line: " << linenum << " \"" << p << "\""
                          << " is not a number of double" << std::endl;
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
    if (traj_sz == 0)
    {
        std::cerr << "No waypoints" << std::endl;
        return -1;
    }

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
    std::cout << "Moving to home point..." << std::endl;
    double wp1[6] = {0, 0, 0, 0, 0, 0};
    robotService.robotServiceJointMove(wp1, true);
    std::cout << "Move to home point succ." << std::endl;

    // 4. Enter Tcp2Canbus mode
    ret = robotService.robotServiceEnterTcp2CanbusMode();
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "Enter Tcp2Canbus mode fail, ret = " << ret << std::endl;
        return ret;
    }
    std::cout << "Enter Tcp2Canbus mode succ." << std::endl;

    // 5. Start send waypoint to arm
    int cnt = 0;
    while (cnt < traj_sz)
    {
        std::vector<aubo_robot_namespace::wayPoint_S> waypoint_vector;

        // SimRobot: send 1 waypoint one time
        if (mode == aubo_robot_namespace::RobotModeSimulator)
        {
            aubo_robot_namespace::wayPoint_S waypoint;
            for (int i = 0; i < 6; i++) waypoint.jointpos[i] = traj[cnt][i];

            cnt++;

            waypoint_vector.push_back(waypoint);
        }
        // RealRobot: send several waypoints one time
        else
        {
            // Read the buffer size of the interface board.
            aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
            ret = robotService.robotServiceGetRobotDiagnosisInfo(robotDiagnosisInfo);
            if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                std::cout << "Get robot diagnosis info fail, ret = " << ret << std::endl;
                break;
            }
            std::cout << "Waypoint buffer size : " << robotDiagnosisInfo.macTargetPosDataSize << std::endl;

            // Send waypoint to arm if buffer size less than MAC_FIFO_FILL
            if (robotDiagnosisInfo.macTargetPosDataSize < MAC_FIFO_FILL)
            {
                // Send up to ROAD_POINT_RELOAD_SIZE waypoints one time
                for (int i = 0; i < ROAD_POINT_RELOAD_SIZE && cnt < traj_sz; i++)
                {
                    aubo_robot_namespace::wayPoint_S waypoint;
                    for (int i = 0; i < 6; i++) waypoint.jointpos[i] = traj[cnt][i];
                    cnt++;
                    waypoint_vector.push_back(waypoint);
                }
            }
            else
            {
                std::cout << "Buffer is full, wait." << std::endl;
            }
        }

        if (false == waypoint_vector.empty())
        {
            ret = robotService.robotServiceSetRobotPosData2Canbus(waypoint_vector);
            if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                robotService.robotServiceLeaveTcp2CanbusMode();
                return ret;
            }
        }

        usleep(5 * 1000);
    }

    // 6. Leave Tcp2Canbus mode.
    ret = robotService.robotServiceLeaveTcp2CanbusMode();
    sleep(1);

    std::cout << "Trajectory done." << std::endl;

    // 7. Logout
    robotService.robotServiceLogout();

    return 0;
}
