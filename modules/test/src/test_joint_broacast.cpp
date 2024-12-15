

#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <thread>
#include <fstream>

#define SERVER_HOST "192.168.123.96"
char key = 'a';
using namespace std;

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
        return;
    }

    std::string json_str = data.substr(start_pos, data_length);
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(json_str, root))
    {
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
        return -1;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0)
    {
        close(sock);
        return -1;
    }

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
    {
        close(sock);
        return -1;
    }

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
                std::cout << duration.count() << ",";
                for (auto pos = joint_positions_global.begin(); pos != joint_positions_global.end(); pos++)
                {
                    if (pos == joint_positions_global.end() - 1)
                    {
                        std::cout << *pos << endl;
                        file << *pos << endl;
                    }
                    else
                    {
                        std::cout << *pos << ",";
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
void keyWatch() { cin >> key; }

int main()
{
    std::thread joint_status_thread(realTimeCalJointStatus);
    std::thread key_watch_thread(keyWatch);

    joint_status_thread.join();
    key_watch_thread.join();
}