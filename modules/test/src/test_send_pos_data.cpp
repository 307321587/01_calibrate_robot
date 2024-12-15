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
#include <string>
#include <thread>
using namespace std;

#define SERVER_HOST "192.168.123.96"
#define SERVER_PORT 8899

#define FILE_PATH                                                    \
    "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/" \
    "record_line.offt"

#define ROAD_POINT_RELOAD_SIZE 2 // 每一次下发的路点个数
#define MAC_FIFO_FILL (6 * 4)    // 缓冲路点个数 * 6

char volatile key = 'a';
bool volatile read_flag = false; // 不加volatile在release模式下会出现问题

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
std::mutex control_mtx;
std::mutex read_save_mtx;

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

    string path = "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/joints_data.txt";
    ofstream file(path);
    auto start = chrono::system_clock::now();

    char buffer[1550] = {0};
    std::string accumulated_data;
    while (key != 'q')
    {
        int valread = 0;
        {
            // std::lock_guard<mutex> lock(control_mtx);
            valread = read(sock, buffer, sizeof(buffer));
        }

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
                joint_positions_global = joint_positions;

                auto end = chrono::system_clock::now();
                auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
                file << duration.count() << ",";
                // std::cout << duration.count() << ",";
                for (auto pos = joint_positions_global.begin(); pos != joint_positions_global.end(); pos++)
                {
                    if (pos == joint_positions_global.end() - 1)
                    {
                        // std::cout << *pos << endl;
                        file << *pos << endl;
                    }
                    else
                    {
                        // std::cout << *pos << ",";
                        file << *pos << ",";
                    }
                }

                // 输出计算结果
                // std::cout << "Joint positions: ";
                // for (size_t i = 0; i < joint_positions.size(); ++i)
                // {
                //     std::cout << joint_positions[i] << " ";
                // }
                // std::cout << std::endl;

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
        this_thread::sleep_for(chrono::microseconds(1000));
    }
    close(sock);
    file.close();
}

int main()
{
    // std::thread joint_status_thread(realTimeCalJointStatus);

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
    int cnt = 0;
    auto all_start = chrono::system_clock::now();
    // string path = "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/joints_data.txt";
    // ofstream file(path);

    std::thread joint_status_thread(realTimeCalJointStatus);

    while (cnt < traj_sz)
    {
        auto start = chrono::system_clock::now();
        std::vector<aubo_robot_namespace::wayPoint_S> waypoint_vector;

        // SimRobot: send 1 waypoint one time
        {
            // std::lock_guard<mutex> lock(control_mtx);
            // Read the buffer size of the interface board.
            aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
            aubo_robot_namespace::JointParam roboJointParm;
            ret = robotService.robotServiceGetRobotDiagnosisInfo(robotDiagnosisInfo);
            ret = robotService.robotServiceGetJointAngleInfo(roboJointParm);

            // auto end = chrono::system_clock::now();
            // auto duration = chrono::duration_cast<chrono::microseconds>(end - all_start);
            // file << duration.count() << ",";
            // // std::cout << duration.count() << ",";
            // for (int index = 0; index < 6; index++)
            // {
            //     if (index == 5)
            //     {
            //         // std::cout << roboJointParm.jointPos[index] << endl;
            //         file << roboJointParm.jointPos[index] << endl;
            //     }
            //     else
            //     {
            //         // std::cout << roboJointParm.jointPos[index] << ",";
            //         file << roboJointParm.jointPos[index] << ",";
            //     }
            // }

            if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                std::cout << "Get robot diagnosis info fail, ret = " << ret << std::endl;
                break;
            }
            if (robotDiagnosisInfo.macTargetPosDataSize == 0)
            {
                std::cout << "Waypoint buffer size : " << robotDiagnosisInfo.macTargetPosDataSize << std::endl;
            }

            // Send waypoint to arm if buffer size less than MAC_FIFO_FILL
            if (robotDiagnosisInfo.macTargetPosDataSize < MAC_FIFO_FILL)
            {
                // Send up to ROAD_POINT_RELOAD_SIZE waypoints one time
                // if (robotDiagnosisInfo.macTargetPosDataSize == 0)
                // {
                //     for (int i = 0; i < 2 * ROAD_POINT_RELOAD_SIZE && cnt < traj_sz; i++)
                //     {
                //         aubo_robot_namespace::wayPoint_S waypoint;
                //         for (int i = 0; i < 6; i++) waypoint.jointpos[i] = traj[cnt][i];
                //         cnt++;
                //         waypoint_vector.push_back(waypoint);
                //     }
                // }
                // else
                {
                    for (int i = 0; i < ROAD_POINT_RELOAD_SIZE && cnt < traj_sz; i++)
                    {
                        aubo_robot_namespace::wayPoint_S waypoint;
                        for (int i = 0; i < 6; i++) waypoint.jointpos[i] = traj[cnt][i];
                        cnt++;
                        waypoint_vector.push_back(waypoint);
                    }
                }
            }
            else
            {
                // std::cout << "Buffer is full, wait." << std::endl;
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
        }
        auto end = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
        int cost = duration.count();
        // cout << "cost:" << cost << "us" << endl;

        this_thread::sleep_for(chrono::microseconds(5000 - cost));
        // usleep(5 * 1000 - cost);
        // do something...
    }

    // 6. Leave Tcp2Canbus mode.
    ret = robotService.robotServiceLeaveTcp2CanbusMode();
    sleep(1);

    std::cout << "Trajectory done." << std::endl;

    // 7. Logout
    robotService.robotServiceLogout();

    key = 'q';
    // file.close();
    joint_status_thread.join();

    return 0;
}
