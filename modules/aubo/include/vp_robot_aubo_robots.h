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
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpRzyxVector.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>

class vpRobotAuboRobots : public vpRobot
{
private: // Not allowed functions
    /*!
        Copy constructor not allowed.
    */
    vpRobotAuboRobots(const vpRobotAuboRobots &robot);

public:
    vpRobotAuboRobots();
    vpRobotAuboRobots(const std::string &ip_address, bool rt_flag = false);
    virtual ~vpRobotAuboRobots();

    void connect(const std::string &ip_address, bool rt_flag = false);
    void disconnect();
    int connectRtServer(const std::string &ip, int port);
    void parseJointData(const std::string &data, std::vector<std::string> &joint_names, std::vector<double> &joint_positions);
    void realTimeCalJointStatus(const std::string &tcp_ip);

    vpHomogeneousMatrix get_fMe();
    vpHomogeneousMatrix get_fMe(const vpColVector &q);
    vpHomogeneousMatrix get_fMc();
    vpHomogeneousMatrix get_eMc() const;

    void getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force);
    std::string getPolyScopeVersion();
    void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position) vp_override;
    int getRobotMode() const;
    std::string getRobotModel() const;

    void move(const std::string &filename, double velocity_percentage = 10.);

    bool readPosFile(const std::string &filename, vpColVector &q);
    bool savePosFile(const std::string &filename, const vpColVector &q);

    void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position) vp_override;
    void setPositioningVelocity(double velocity);

    vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);
    void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel) vp_override;

    void set_eMc(const vpHomogeneousMatrix &eMc);
    void set_dt(float dt) { m_dt = dt; };

    void stopMotion();

    // doosan机器人zyz到xyz旋转角度转换
    static vpColVector exyz_tToAxisAngle_t(vpColVector exyz_t)
    {
        vpRzyxVector zyx(exyz_t[3], exyz_t[4], exyz_t[5]);
        vpRotationMatrix rot(zyx);
        vpThetaUVector xyz(rot);
        vpColVector AxisAngle_t;
        AxisAngle_t = {exyz_t[0], exyz_t[1], exyz_t[2], xyz[0], xyz[1], xyz[2]};
        return AxisAngle_t;
    };

    static vpColVector AxisAngle_t2exyz_t(vpColVector AxisAngle_t)
    {
        vpThetaUVector xyz(AxisAngle_t[3], AxisAngle_t[4], AxisAngle_t[5]);
        vpRotationMatrix rot(xyz);
        vpRzyxVector zyx(rot);
        vpColVector exyz_t;
        exyz_t = {AxisAngle_t[0], AxisAngle_t[1], AxisAngle_t[2], zyx[2], zyx[1], zyx[0]};
        return exyz_t;
    };

private:
    // Not implemented yet
    void get_eJe(vpMatrix &_eJe) vp_override{};
    void get_fJe(vpMatrix &_fJe) vp_override{};
    void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q) vp_override{};

protected:
    void init();

    vpHomogeneousMatrix m_eMc;
    double m_max_positioning_speed = 1000;
    double m_max_positioning_acceleration = 1000;
    double m_max_joint_speed = 150;
    double m_max_joint_acceleration = 150;
    double m_max_linear_speed = 1000;
    double m_max_linear_acceleration = 1000;
    vpRobot::vpControlFrameType m_vel_control_frame;
    int m_connected = 1;
    int m_rt_connected = 1;
    int m_tp_initailizing_complted = 1;
    float m_dt = 0.05;
    int m_control_port = 8899;
    int m_callback_port = 8891;
    ServiceInterface m_robot_service;
    std::mutex m_control_mutex;
    std::vector<double> rt_joints_postions;
    std::thread m_rt_thread;
};