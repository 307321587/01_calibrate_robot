#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <jsoncpp/json/json.h>
#include <math.h>
#include <string>
#include <thread>

using namespace std;
char volatile key = 'a';
bool volatile read_flag = false; // 不加volatile在release模式下会出现问题
auto start = chrono::system_clock::now();
#define SERVER_HOST "192.168.123.96"
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
                std::cerr << "Line: " << linenum << " \"" << p << "\"" << " is not a number of double" << std::endl;
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

void realTimeJointStatusCallback(const aubo_robot_namespace::JointStatus *jointStatus, int len, void *arg)
{
    (void)arg;
    std::cout << std::endl << "start----------Joint status information-------" << std::endl;
    auto tmp_end = chrono::system_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(tmp_end - start);
    std::cout << "Time:" << duration.count() << "us" << std::endl;

    for (int i = 0; i < len; i++)
    {
        std::cout << "Joint ID:" << i << "  ";
        std::cout << "Current:" << jointStatus[i].jointCurrentI << " ";
        std::cout << "speed:" << jointStatus[i].jointSpeedMoto << " ";
        std::cout << "Joint angle:" << jointStatus[i].jointPosJ << " " << " ~ " << jointStatus[i].jointPosJ * 180.0 / M_PI;
        std::cout << "Voltage   :" << jointStatus[i].jointCurVol << " ";
        std::cout << "Temperature   :" << jointStatus[i].jointCurTemp << " ";
        std::cout << "Target current:" << jointStatus[i].jointTagCurrentI << " ";
        std::cout << "Target motor speed:" << jointStatus[i].jointTagSpeedMoto << " ";
        std::cout << "Target joint angle :" << jointStatus[i].jointTagPosJ << " ";
        std::cout << "Joint error   :" << jointStatus[i].jointErrorNum << std::endl;
    }
    std::cout << std::endl;
}

int main()
{
    // // 实时优先级最大值、最小值
    // int sched_max = sched_get_priority_max(SCHED_FIFO);

    // // 设置实时调度策略及优先级
    // struct sched_param sParam;
    // sParam.sched_priority = sched_max;
    // sched_setscheduler(0, SCHED_FIFO, &sParam);
    // auto i_schedFlag = sched_getscheduler(0);
    // printf("设置调度策略 = [%d]\n", i_schedFlag);

    // // 绑定CPU
    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // CPU_SET(1, &cpuset);

    // // bind process to processor 0
    // if (sched_setaffinity(0, sizeof(cpuset), &cpuset) < 0)
    // {
    //     perror("Sched_setaffinity fail!");
    // }
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

    robotService.robotServiceRegisterRealTimeJointStatusCallback(realTimeJointStatusCallback, NULL);

    aubo_robot_namespace::RobotWorkMode mode = aubo_robot_namespace::RobotModeReal;
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
        double joint_angle[6];
        for (int i = 0; i < 6; i++) joint_angle[i] = traj[cnt][i];
        cnt++;

        auto tmp_start = chrono::system_clock::now();
        {
            ret = robotService.robotServiceSetRobotPosData2Canbus(joint_angle);
        }
        auto tmp_end = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(tmp_end - tmp_start);
        int cost = int(duration.count());
        // cout << "cost:" << cost << "us" << endl;

        if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            robotService.robotServiceLeaveTcp2CanbusMode();
            return ret;
        }

        this_thread::sleep_for(chrono::microseconds(5000 - cost));
    }

    // 6. Leave Tcp2Canbus mode.
    ret = robotService.robotServiceLeaveTcp2CanbusMode();
    sleep(1);

    std::cout << "Trajectory done." << std::endl;

    // 7. Logout
    robotService.robotServiceLogout();
    return 0;
}
