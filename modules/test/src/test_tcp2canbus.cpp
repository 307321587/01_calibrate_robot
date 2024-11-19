#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <thread>

using namespace std;
char key = 'a';
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

int connectToServer(const std::string &ip, int port)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        // ROS_ERROR("Socket creation error");
        return -1;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0)
    {
        // ROS_ERROR("Invalid address/ Address not supported");
        close(sock);
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        // ROS_ERROR("Connection Failed");
        close(sock);
        return -1;
    }

    // ROS_INFO("Connected to server at %s:%d", ip.c_str(), port);
    return sock;
}

vector<double> joint_positions_global = {0, 0, 0, 0, 0, 0};
std::mutex mtx;

void parseJointData(const std::string &data, std::vector<std::string> &joint_names, std::vector<double> &joint_positions)
{
    const std::string PACK_BEGIN = "<PACK_BEGIN";
    const std::string PACK_END = "PACK_END>";

    size_t start_pos = data.find(PACK_BEGIN);
    size_t end_pos = data.find(PACK_END);

    if (start_pos == std::string::npos || end_pos == std::string::npos || start_pos >= end_pos)
    {
        return;
    }

    start_pos += PACK_BEGIN.length();
    size_t length_pos = start_pos;
    start_pos += 8;

    std::string length_str = data.substr(length_pos, 8);
    size_t data_length = std::stoul(length_str);
    if (end_pos - start_pos != data_length)
    {
        // ROS_WARN("Data length mismatch. Expected: %zu, Actual: %zu", data_length, end_pos - start_pos);
        return;
    }

    std::string json_str = data.substr(start_pos, data_length);
    // ROS_INFO("Extracted JSON string: %s", json_str.c_str());

    Json::Reader reader;
    Json::Value root;

    if (reader.parse(json_str, root))
    {
        // ROS_INFO("Parsed JSON successfully.");

        const Json::Value jointPos = root["RobotJointStatus"]["jointPos"];
        if (!jointPos.isNull() && jointPos.isArray())
        {
            std::vector<std::string> joint_names_temp = {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"};
            for (Json::Value::ArrayIndex i = 0; i < jointPos.size(); ++i)
            {
                joint_positions.push_back(jointPos[i].asDouble());
                joint_names.push_back(joint_names_temp[i]);
            }
        }
    }
}

void realTimeCalJointStatus()
{
    std::string tcp_ip = SERVER_HOST;
    int tcp_port = 8891;

    int sock = connectToServer(tcp_ip, tcp_port);
    if (sock < 0)
    {
        return;
    }

    char buffer[1550] = {0};
    std::string accumulated_data;
    while (key != 'q')
    {
        int valread = 0;

        valread = read(sock, buffer, sizeof(buffer));

        bool success = (valread > 0);
        if (success)
        {
            accumulated_data.append(buffer, valread);

            size_t start_pos = accumulated_data.find("<PACK_BEGIN");
            size_t end_pos = accumulated_data.find("PACK_END>");

            while (start_pos != std::string::npos && end_pos != std::string::npos)
            {
                if (end_pos + 8 > accumulated_data.size())
                {
                    break;
                }

                std::string complete_data = accumulated_data.substr(start_pos, end_pos - start_pos + 9);
                accumulated_data.erase(0, end_pos + 8);

                vector<std::string> joint_names;
                vector<double> joint_positions;

                parseJointData(complete_data, joint_names, joint_positions);
                // 加锁
                {
                    std::lock_guard<mutex> lock(mtx);
                    joint_positions_global = joint_positions;
                }
                // 输出计算结果
                std::cout << "Joint positions: ";
                for (size_t i = 0; i < joint_positions.size(); ++i)
                {
                    std::cout << joint_positions[i] << " ";
                }
                std::cout << std::endl;

                start_pos = accumulated_data.find("<PACK_BEGIN");
                end_pos = accumulated_data.find("PACK_END>");
            }
        }
        else
        {
            close(sock);
            sock = connectToServer(tcp_ip, tcp_port);
            while (sock < 0)
            {
                sleep(1);
                sock = connectToServer(tcp_ip, tcp_port);
            }
        }
        memset(buffer, 0, sizeof(buffer));
    }
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

    std::thread joint_status_thread(realTimeCalJointStatus);
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
        double joint_angle[6];
        for (int i = 0; i < 6; i++) joint_angle[i] = traj[cnt][i];
        cnt++;
        {
            // std::lock_guard<mutex> lock(mtx);
            ret = robotService.robotServiceSetRobotPosData2Canbus(joint_angle);
        }

        if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            robotService.robotServiceLeaveTcp2CanbusMode();
            return ret;
        }

        this_thread::sleep_for(chrono::milliseconds(5));
    }

    // 6. Leave Tcp2Canbus mode.
    ret = robotService.robotServiceLeaveTcp2CanbusMode();
    sleep(1);

    std::cout << "Trajectory done." << std::endl;

    // 7. Logout
    robotService.robotServiceLogout();
    key = 'q';
    joint_status_thread.join();
    return 0;
}
