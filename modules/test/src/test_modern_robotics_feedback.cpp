
#include "modern_robotics.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iterator>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#define SERVER_HOST "192.168.123.96"
char key = 'a';
using namespace std;
std::condition_variable cv;
volatile bool ready = false;
void readTxt(const string& filename, Eigen::VectorXi size, Eigen::MatrixXd& matrix)
{
    matrix.resize(size(0), size(1));
    ifstream file(filename);
    if (!file.is_open())
    {
        cerr << "Failed to open file" << endl;
        return;
    }

    vector<double> data;
    string line;
    while (getline(file, line))
    {
        istringstream iss(line);
        istream_iterator<double> int_iter(iss);
        istream_iterator<double> eof;
        while (int_iter != eof)
        {
            data.push_back(*int_iter++);
        }
    }
    file.close();

    if (data.empty())
    {
        cerr << "File is empty or data is invalid" << endl;
        return;
    }
    // 根据size构建矩阵，size的元素不定，可能为2或3
    int index = 0;
    for (int i = 0; i < size(0); ++i)
    {
        for (int j = 0; j < size(1); ++j)
        {
            if (index < data.size())
            {
                matrix(i, j) = data[index++];
            }
            else
            {
                cerr << "Data size does not match matrix size" << endl;
                return;
            }
        }
    }
}

void parseJointData(const std::string& data, std::vector<std::string>& joint_names, std::vector<double>& joint_positions)
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

int connectToServer(const std::string& ip, int port)
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

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
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
        int valread = read(sock, buffer, sizeof(buffer));
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
                    ready = true;
                }
                cv.notify_one();
                // // 输出计算结果
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
    }
}

void realTimeControl(Eigen::MatrixXd B_matrix, Eigen::MatrixXd S_matrix, Eigen::MatrixXd M_matrix)
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    // 1. Login
    ret = robotService.robotServiceLogin(SERVER_HOST, 8899, "aubo", "123456");
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "Login succ." << std::endl;
    }
    else
    {
        std::cerr << "Login failed." << std::endl;
        return;
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
        return;
    }

    // Avoid not leave tcp2canbus mode last time
    robotService.robotServiceLeaveTcp2CanbusMode();

    // 3. Move joint to home point
    std::cout << "Moving to home point..." << std::endl;
    double wp1[6] = {M_PI / 2, 0, -M_PI / 2, 0, -M_PI / 2, 0};
    robotService.robotServiceJointMove(wp1, true);
    std::cout << "Move to home point succ." << std::endl;

    // 4. Enter Tcp2Canbus mode
    ret = robotService.robotServiceEnterTcp2CanbusMode();
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "Enter Tcp2Canbus mode fail, ret = " << ret << std::endl;
        return;
    }
    std::cout << "Enter Tcp2Canbus mode succ." << std::endl;

    int acc_sum_count = 40;
    Eigen::VectorXd target_vel(6);
    target_vel << -0.01, 0, 0, 0.01, 0, 0;
    Eigen::VectorXd max_acc(6);
    max_acc = target_vel / acc_sum_count;
    Eigen::VectorXd cur_vel(6);
    cur_vel << 0, 0, 0, 0, 0, 0;
    int count = 0;
    int count_sum = 200;

    auto last = chrono::system_clock::now();
    // 输入不为q
    while (key != 'q')
    {
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock); // 等待通知
            ready = false;
        }

        auto now = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(now - last);
        int cost = duration.count();
        cout << "cost:" << cost << "us" << endl;
        last = now;

        if (count == count_sum + acc_sum_count)
        {
            key = 'q';
            return;
        }

        if (count % count_sum == 0)
        {
            target_vel = -target_vel;
            max_acc = -max_acc;
        }

        if ((target_vel - cur_vel).norm() > 1e-6)
        {
            cur_vel += max_acc;
        }
        // cout << cur_vel << endl;
        Eigen::MatrixXd joint_positions_matrix(6, 1);
        {
            // std::lock_guard<mutex> lock(mtx);
            for (int i = 0; i < 6; i++)
            {
                joint_positions_matrix(i) = joint_positions_global[i];
            }
        }
        Eigen::MatrixXd jacobi_b = mr::JacobianBody(B_matrix, joint_positions_matrix);
        Eigen::MatrixXd inverse_jacobi_b = jacobi_b.completeOrthogonalDecomposition().pseudoInverse();
        // 矩阵乘以向量
        Eigen::VectorXd joint_velocities = inverse_jacobi_b * cur_vel;
        std::cout << "current joint: " << joint_positions_matrix.transpose() << std::endl;
        joint_positions_matrix += joint_velocities;
        double joint_angle[6];
        for (int i = 0; i < 6; i++) joint_angle[i] = joint_positions_matrix(i);
        ret = robotService.robotServiceSetRobotPosData2Canbus(joint_angle);
        // 输出计算结果
        std::cout << "target joint: " << joint_positions_matrix.transpose() << std::endl;

        // Eigen::MatrixXd end_pose = mr::FKinBody(M_matrix, B_matrix, joint_positions_matrix);
        // cout << end_pose << endl;
        count += 1;
        // auto end = chrono::system_clock::now();
        // auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
        // int cost = duration.count();
        // cout << "cost:" << cost << "us" << endl;
        // this_thread::sleep_for(chrono::microseconds(5000 - cost));
    }
    // 6. Leave Tcp2Canbus mode.
    ret = robotService.robotServiceLeaveTcp2CanbusMode();
    sleep(1);

    std::cout << "Trajectory done." << std::endl;

    // 7. Logout
    robotService.robotServiceLogout();
}

void keyWatch()
{
    // cout << "线程开启" << endl;
    cin >> key;
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
    // CPU_SET(2, &cpuset);

    // // bind process to processor 0
    // if (sched_setaffinity(0, sizeof(cpuset), &cpuset) < 0)
    // {
    //     perror("Sched_setaffinity fail!");
    // }

    Eigen::MatrixXd B_matrix, S_matrix, M_matrix;
    string S_filename = "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/Slist.txt";
    string B_filename = "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/Blist.txt";
    string M_filename = "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/M.txt";
    Eigen::VectorXi B_S_size(2);
    B_S_size << 6, 6;
    Eigen::VectorXi M_size(2);
    M_size << 4, 4;

    readTxt(S_filename, B_S_size, S_matrix);
    readTxt(B_filename, B_S_size, B_matrix);
    readTxt(M_filename, M_size, M_matrix);

    std::thread joint_status_thread(realTimeCalJointStatus);
    usleep(5 * 10000);
    std::thread control_thread(realTimeControl, B_matrix, S_matrix, M_matrix);
    std::thread key_watch_thread(keyWatch);

    joint_status_thread.join();
    control_thread.join();
    key_watch_thread.join();
    // cout << S_matrix << endl;
}