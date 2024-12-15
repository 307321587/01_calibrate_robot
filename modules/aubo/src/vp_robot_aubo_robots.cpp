

#include "vp_robot_aubo_robots.h"
#include <chrono>
#include <cmath>
#include <opencv2/imgproc/types_c.h>
#include <mutex>
#include <thread>
#include <vector>
#include "modern_robotics.h"
#include <assert.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpRzyxVector.h>
#include <visp3/core/vpTranslationVector.h>

using namespace std;
vpRobotAuboRobots::vpRobotAuboRobots() { init(); }

vpRobotAuboRobots::vpRobotAuboRobots(const string &ip_address, bool rt_flag)
{
    init();
    connect(ip_address, rt_flag);
}

/*!
 * Destructor that shut down the connexion with the robot.
 */

vpRobotAuboRobots::~vpRobotAuboRobots()
{
    disconnect();
    if (m_rt_thread.joinable()) m_rt_thread.join();
    if (m_rt_tcp_thread.joinable()) m_rt_tcp_thread.join();
}

/*!
 * Establishes a connection with the robot and set default behavior.
 * \param[in] ur_address IP/hostname of the robot.
 *
 * \exception vpException::fatalError : When connexion cannot be established.
 */
void vpRobotAuboRobots::connect(const std::string &ip_address, bool rt_flag)
{
    // 1. Login
    m_connected = m_robot_service.robotServiceLogin(ip_address.c_str(), m_control_port, "aubo", "123456");
    if (m_connected == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "Login succ." << std::endl;
    }
    else
    {
        std::cerr << "Login failed." << std::endl;
        return;
    }

    aubo_robot_namespace::RobotWorkMode mode = aubo_robot_namespace::RobotModeSimulator;
    m_robot_service.robotServiceGetRobotWorkMode(mode);

    // 2. Startup
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    m_connected = m_robot_service.rootServiceRobotStartup(toolDynamicsParam /**Tool dynamics parameter**/, 6 /*Collision level*/, true /*Whether to allow reading poses defaults to true*/,
                                                          true,    /*Leave the default to true */
                                                          1000,    /*Leave the default to 1000 */
                                                          result); /*Robot arm initialization*/
    if (m_connected == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "Robot arm initialization succ." << std::endl;
    }
    else
    {
        std::cerr << "Robot arm initialization failed." << std::endl;
        return;
    }

    if (rt_flag)
    {
        rtInit(ip_address);
    }
}

void vpRobotAuboRobots::rtInit(const std::string &ip_address)
{
    // Avoid not leave tcp2canbus mode last time
    m_robot_service.robotServiceLeaveTcp2CanbusMode();
    // 4. Enter Tcp2Canbus mode
    m_rt_connected = m_robot_service.robotServiceEnterTcp2CanbusMode();
    if (m_rt_connected != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "Enter Tcp2Canbus mode fail, m_rt_connected = " << m_rt_connected << std::endl;
        return;
    }
    std::cout << "Enter Tcp2Canbus mode succ." << std::endl;

    m_stop_tcp_flag = false;
    m_rt_thread = std::thread(&vpRobotAuboRobots::realTimeCalJointStatus, this, ip_address);
    m_rt_tcp_thread = std::thread(&vpRobotAuboRobots::realTimeSendTcp, this);
}

/*!
 * Disconnect the robot interfaces.
 */
void vpRobotAuboRobots::disconnect()
{
    // 退出tcp2can.
    if (m_rt_connected == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        m_stop_tcp_flag = true;
        std::unique_lock<std::mutex> lock(m_quit_mutex);
        m_quit_cv.wait(lock);
        m_rt_connected = 1;
        m_robot_service.robotServiceLeaveTcp2CanbusMode();
    }
    sleep(1);

    std::cout << "Trajectory done." << std::endl;

    // 退出登录
    if (m_connected == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        m_robot_service.robotServiceLogout();
        m_connected = 1;
    }
}

/*!
 * Initialize internal vars.
 */
void vpRobotAuboRobots::init()
{
    nDof = 6;
    m_vel_control_frame = vpRobot::JOINT_STATE;
    m_cur_vel.resize(6);
    m_cur_vel.setZero();

    string S_filename = "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/Slist.txt";
    string B_filename = "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/Blist.txt";
    string M_filename = "/home/lza/code/04_uncalibrate_robot/01_calibrate_robot/record/M.txt";
    Eigen::VectorXi B_S_size(2);
    B_S_size << 6, 6;
    Eigen::VectorXi M_size(2);
    M_size << 4, 4;

    readTxt(S_filename, B_S_size, m_S_matrix);
    readTxt(B_filename, B_S_size, m_B_matrix);
    readTxt(M_filename, M_size, m_M_matrix);
}

/*!
 * Set the \f$ ^{e}{\bf M}_c\f$ homogeneous transformation that gives the position
 * of the camera frame (or in general of any tool frame) in the robot end-effector frame.
 *
 * By default, this transformation is set to identity, meaning that the camera (or tool)
 * frame is located on the end-effector.
 *
 * This transformation has to be set before controlling the robot cartesian velocity in
 * the camera frame or getting the position of the robot in the camera frame.
 *
 * \param[in] eMc : End-effector to camera frame transformation.
 */
void vpRobotAuboRobots::set_eMc(const vpHomogeneousMatrix &eMc) { m_eMc = eMc; }

/*!
 * Return the \f$ ^{e}{\bf M}_c\f$ homogeneous transformation that gives the position
 * of the camera frame (or in general of any tool frame) in the robot end-effector frame.
 *
 * By default, this transformation is set to identity, meaning that the camera (or tool)
 * frame is located on the end-effector.
 *
 * To change the position of the camera (or tool) frame on the end-effector frame, use set_eMc().
 *
 */
vpHomogeneousMatrix vpRobotAuboRobots::get_eMc() const { return m_eMc; }

/*!
 * Given the current joint position of the robot, computes the forward kinematics (direct geometric model) as an
 * homogeneous matrix \f${^f}{\bf M}_e\f$ that gives the position of the end-effector in the robot base frame.
 *
 * As described
 * [here](https://docs.pickit3d.com/en/2.4/faq/robot-programming/how-to-define-the-tcp-on-a-universal-robots.html) the
 * end-effector position could be modified setting the Tool Center Point (TCP). When TCP translations and rotations are
 * set to 0, the end-effector corresponds to the robot flange position.
 *
 * \return Position of the end-effector in the robot base frame.
 */
vpHomogeneousMatrix vpRobotAuboRobots::get_fMe()
{
    vpColVector fPe;
    getPosition(vpRobot::END_EFFECTOR_FRAME, fPe);
    vpRzyxVector fPe_r(fPe[5], fPe[4], fPe[3]);
    vpRotationMatrix rot(fPe_r);
    vpTranslationVector fPe_t(fPe[0], fPe[1], fPe[2]);
    return vpHomogeneousMatrix(fPe_t, rot);
}

/*!
 * Given a joint position of the robot, computes the forward kinematics (direct geometric model) as an
 * homogeneous matrix \f${^f}{\bf M}_e\f$ that gives the position of the end-effector in the robot base frame.
 *
 * As described
 * [here](https://docs.pickit3d.com/en/2.4/faq/robot-programming/how-to-define-the-tcp-on-a-universal-robots.html) the
 * end-effector position could be modified setting the Tool Center Point (TCP). When TCP translations and rotations are
 * set to 0, the end-effector corresponds to the robot flange position.
 *
 * \param[in] q : Joint position as a 6-dim vector
 *
 * \return Position of the end-effector in the robot base frame.
 */
vpHomogeneousMatrix vpRobotAuboRobots::get_fMe(const vpColVector &q) { return m_eMc; }

/*!
 * Get robot position.
 * \param[in] frame : Type of position to retrieve. Admissible values are:
 * - vpRobot::JOINT_STATE to get the 6 joint positions.
 * - vpRobot::END_EFFECTOR_FRAME to retrieve the cartesian position of the end-effector frame wrt the robot base frame.
 * - vpRobot::CAMERA_FRAME to retrieve the cartesian position of the camera frame (or more generally a tool frame
 *   vpRobot::TOOL_FRAME) wrt the robot base frame.
 * \param[out] position : Robot position. When joint position is asked this vector is 6-dim. Otherwise for a cartesian
 * position this vector is also 6-dim. Its content is similar to a vpPoseVector, with first the 3 tranlations in meter
 * and then the 3 orientations in radian as a \f$\theta {\bf u}\f$ vector (see vpThetaUVector).
 *
 * If you want to get a cartesian position, use rather
 * getPosition(const vpRobot::vpControlFrameType, vpPoseVector &)
 */
void vpRobotAuboRobots::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position)
{
    if (m_connected != aubo_robot_namespace::InterfaceCallSuccCode && m_rt_connected != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        throw(vpException(vpException::fatalError, "Cannot get doosan robot position: robot is not connected"));
    }

    switch (frame)
    {
        case JOINT_STATE: {
            aubo_robot_namespace::wayPoint_S currentWaypoint;
            m_robot_service.robotServiceGetCurrentWaypointInfo(currentWaypoint);
            position.resize(6);
            for (size_t i = 0; i < 6; i++)
            {
                position[i] = currentWaypoint.jointpos[i];
            }
            break;
        }
        case END_EFFECTOR_FRAME: {
            if (m_rt_connected != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                aubo_robot_namespace::wayPoint_S currentWaypoint;
                m_robot_service.robotServiceGetCurrentWaypointInfo(currentWaypoint);
                aubo_robot_namespace::Rpy rpy;
                m_robot_service.quaternionToRPY(currentWaypoint.orientation, rpy);
                position.resize(6);
                position[0] = currentWaypoint.cartPos.position.x;
                position[1] = currentWaypoint.cartPos.position.y;
                position[2] = currentWaypoint.cartPos.position.z;
                position[3] = rpy.rx;
                position[4] = rpy.ry;
                position[5] = rpy.rz;
            }
            else
            {
                double joints[6];
                {
                    std::lock_guard<mutex> lock(m_tcp_mutex);
                    if (m_rt_joints_postions.size() != 6) return;
                    for (int i = 0; i < 6; i++)
                    {
                        joints[i] = m_rt_joints_postions[i];
                    }
                }
                aubo_robot_namespace::wayPoint_S currentWaypoint;
                m_robot_service.robotServiceRobotFk(joints, 6, currentWaypoint);
                aubo_robot_namespace::Rpy rpy;
                m_robot_service.quaternionToRPY(currentWaypoint.orientation, rpy);
                position.resize(6);
                position[0] = currentWaypoint.cartPos.position.x;
                position[1] = currentWaypoint.cartPos.position.y;
                position[2] = currentWaypoint.cartPos.position.z;
                position[3] = rpy.rx;
                position[4] = rpy.ry;
                position[5] = rpy.rz;
            }
        }
        break;
        case TOOL_FRAME: { // same a CAMERA_FRAME
            aubo_robot_namespace::wayPoint_S currentWaypoint;
            m_robot_service.robotServiceGetCurrentWaypointInfo(currentWaypoint);
            aubo_robot_namespace::Rpy rpy;
            m_robot_service.quaternionToRPY(currentWaypoint.orientation, rpy);
            vpColVector fPe_col;
            fPe_col.resize(6);
            fPe_col[0] = currentWaypoint.cartPos.position.x;
            fPe_col[1] = currentWaypoint.cartPos.position.y;
            fPe_col[2] = currentWaypoint.cartPos.position.z;
            fPe_col[3] = rpy.rx;
            fPe_col[4] = rpy.ry;
            fPe_col[5] = rpy.rz;
            fPe_col = exyz_tToAxisAngle_t(fPe_col);
            vpPoseVector fPe(fPe_col[0], fPe_col[1], fPe_col[2], fPe_col[3], fPe_col[4], fPe_col[5]);
            vpHomogeneousMatrix fMe(fPe);
            vpHomogeneousMatrix fMc = fMe * m_eMc;
            vpPoseVector fPc(fMc);
            position = AxisAngle_t2exyz_t(fPc);
        }
        break;
        default: {
            throw(vpException(vpException::fatalError, "Cannot get UR cartesian position: wrong method"));
        }
    }
}
std::vector<double> vpRobotAuboRobots::getPositionP(const vpRobot::vpControlFrameType frame)
{
    vpColVector position;
    position.resize(6);
    getPosition(frame, position);
    std::vector<double> position_vec;
    position_vec.push_back(position[0]);
    position_vec.push_back(position[1]);
    position_vec.push_back(position[2]);
    position_vec.push_back(position[3]);
    position_vec.push_back(position[4]);
    position_vec.push_back(position[5]);
    return position_vec;
}
/*!
 * Set the maximal velocity percentage to use for a position control.
 *
 * \param[in] velocity : Percentage of the maximal velocity. Values should be in ]0:100].
 */
void vpRobotAuboRobots::setPositioningVelocity(double velocity) { m_max_positioning_acceleration = velocity; }

/*!
 * 设置机器人笛卡尔位置,该函数是阻塞的;当到达所需位置时返回。
 * \param[in] pose :6维向量对应要到达的位置
 * meters for the translations and radians for the rotations.
 *
 * \param[in] frame : Frame in which the position is expressed.
 * - In the camera frame (or the tool frame which is the same), the 3 first vector values correspond to the
 * translation between the robot base and the camera (or tool frame), while the 3 last vector values to the ThetaU
 * rotations represented by a vpThetaUVector.
 * - In the end-effector frame (or TCP frame), the 3 first vector values correspond to the translation between the
 * robot base and the end-effector, while the 3 last vector values to the ThetaU rotations represented by a
 * vpThetaUVector.
 */
void vpRobotAuboRobots::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position)
{
    if (m_connected != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        throw(vpException(vpException::fatalError, "Cannot set Doosan robot position: robot is not connected"));
    }

    if (position.size() != 6)
    {
        throw(vpException(vpException::fatalError, "Cannot set Doosan robot position: position vector is not a 6-dim vector (%d)", position.size()));
    }

    if (frame == vpRobot::JOINT_STATE)
    {
        double wp1[6] = {position[0], position[1], position[2], position[3], position[4], position[5]};
        m_robot_service.robotServiceJointMove(wp1, true);
    }
    else if (frame == vpRobot::END_EFFECTOR_FRAME)
    {
        aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
        baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;
        aubo_robot_namespace::wayPoint_S currentWaypoint;
        m_robot_service.robotServiceGetCurrentWaypointInfo(currentWaypoint); // 获取当前关节角位置，用于计算逆解
        vpColVector postion_xyz(position);
        aubo_robot_namespace::Pos position{postion_xyz[0], postion_xyz[1], postion_xyz[2]};
        aubo_robot_namespace::Ori orientation;
        aubo_robot_namespace::Rpy rpy{postion_xyz[3], postion_xyz[4], postion_xyz[5]};
        m_robot_service.RPYToQuaternion(rpy, orientation);

        double start_joint[6] = {currentWaypoint.jointpos[0], currentWaypoint.jointpos[1], currentWaypoint.jointpos[2],
                                 currentWaypoint.jointpos[3], currentWaypoint.jointpos[4], currentWaypoint.jointpos[5]};

        aubo_robot_namespace::wayPoint_S wayPoint;
        int ret = m_robot_service.robotServiceRobotIk(start_joint, position, orientation, wayPoint);

        double wp1[6] = {wayPoint.jointpos[0], wayPoint.jointpos[1], wayPoint.jointpos[2], wayPoint.jointpos[3], wayPoint.jointpos[4], wayPoint.jointpos[5]};
        m_robot_service.robotServiceJointMove(wp1, true);
    }
    else if (frame == vpRobot::CAMERA_FRAME)
    {
        vpTranslationVector f_t_c(position.extract(0, 3));
        vpThetaUVector f_tu_c(position.extract(3, 3));
        vpHomogeneousMatrix fMc(f_t_c, f_tu_c);
        vpHomogeneousMatrix fMe = fMc * m_eMc.inverse();
        vpPoseVector fPe(fMe);

        vpColVector postion_xyz = AxisAngle_t2exyz_t(fPe);

        aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
        baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;
        aubo_robot_namespace::wayPoint_S currentWaypoint;
        m_robot_service.robotServiceGetCurrentWaypointInfo(currentWaypoint); // 获取当前关节角位置，用于计算逆解
        aubo_robot_namespace::Pos position{postion_xyz[0], postion_xyz[1], postion_xyz[2]};
        aubo_robot_namespace::Ori orientation;
        aubo_robot_namespace::Rpy rpy{postion_xyz[3], postion_xyz[4], postion_xyz[5]};
        m_robot_service.RPYToQuaternion(rpy, orientation);

        double start_joint[6] = {currentWaypoint.jointpos[0], currentWaypoint.jointpos[1], currentWaypoint.jointpos[2],
                                 currentWaypoint.jointpos[3], currentWaypoint.jointpos[4], currentWaypoint.jointpos[5]};

        aubo_robot_namespace::wayPoint_S wayPoint;
        int ret = m_robot_service.robotServiceRobotIk(start_joint, position, orientation, wayPoint);

        double wp1[6] = {wayPoint.jointpos[0], wayPoint.jointpos[1], wayPoint.jointpos[2], wayPoint.jointpos[3], wayPoint.jointpos[4], wayPoint.jointpos[5]};
        m_robot_service.robotServiceJointMove(wp1, true);
    }
    else
    {
        throw(vpException(vpRobotException::functionNotImplementedError, "Cannot move the robot to a cartesian position. Only joint positioning is implemented"));
    }
    return;
}

void vpRobotAuboRobots::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
    {
        std::unique_lock<std::mutex> lock(m_vel_mutex);
        for (int i = 0; i < 6; i++) m_cur_vel(i) = vel[i];
    }

    // vpColVector vel_sat(6);
    // m_vel_control_frame = frame;
    // vel_sat = vel;

    // // Set the maximal allowed velocities
    // vpColVector v_max(6);
    // for (int i = 0; i < 3; i++) v_max[i] = 0.3;             // in translation (m/s)
    // for (int i = 3; i < 6; i++) v_max[i] = vpMath::rad(10); // in rotation (rad/s)

    // // for (unsigned int i = 3; i < 6; i++)
    // // {
    // //     vel_sat[i]=0;
    // // }

    // // Velocity saturation
    // switch (frame)
    // {
    //     // saturation in cartesian space
    //     case vpRobot::CAMERA_FRAME:
    //     case vpRobot::REFERENCE_FRAME:
    //     case vpRobot::END_EFFECTOR_FRAME:
    //     case vpRobot::MIXT_FRAME: {
    //         vpColVector v_max(6);
    //         for (int i = 0; i < 3; i++) v_max[i] = 0.1;             // in translation (m/s)
    //         for (int i = 3; i < 6; i++) v_max[i] = vpMath::rad(20); // in rotation (rad/s)

    //         break;
    //     }
    //     // Saturation in joint space
    //     case vpRobot::JOINT_STATE: {
    //         vpColVector vel_max(6);
    //         vel_max = getMaxRotationVelocity();
    //         vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);
    //     }
    // }

    // if (frame == vpRobot::JOINT_STATE)
    // {
    //     double acceleration = 0.5 * 1000;

    //     float vel_sat_arrary[6] = {(float)vel_sat[0], (float)vel_sat[1], (float)vel_sat[2], (float)vel_sat[3], (float)vel_sat[4], (float)vel_sat[5]};
    //     float acceleration_arrary[6] = {(float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration};

    //     Drfl.speedj_rt(vel_sat_arrary, acceleration_arrary, m_dt);
    // }
    // else if (frame == vpRobot::REFERENCE_FRAME)
    // {
    //     double acceleration = 0.5 * 1000;

    //     vpColVector vel_zyz_t = xyz_t2zyz_t(vel_sat);
    //     float vel_sat_arrary[6] = {(float)(vel_zyz_t[0] * 1000),       (float)(vel_zyz_t[1] * 1000),       (float)(vel_zyz_t[2] * 1000),
    //                                (float)(vel_zyz_t[3] / M_PI * 180), (float)(vel_zyz_t[4] / M_PI * 180), (float)(vel_zyz_t[5] / M_PI * 180)};
    //     float acceleration_arrary[6] = {(float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration};

    //     Drfl.speedl_rt(vel_sat_arrary, acceleration_arrary, m_dt);
    // }
    // else if (frame == vpRobot::END_EFFECTOR_FRAME)
    // {
    //     double acceleration = 0.5 * 1000;
    //     vpVelocityTwistMatrix fVe(get_fMe(), false);
    //     vpColVector vel_ = xyz_t2zyz_t(fVe * vel_sat * m_dt);
    //     vel_ = vel_ / m_dt;

    //     // Compute the saturated velocity skew vector
    //     vpColVector vel_zyz_t = vpRobot::saturateVelocities(vel_, v_max, true);

    //     float vel_sat_arrary[6] = {(float)(vel_zyz_t[0] * 1000),       (float)(vel_zyz_t[1] * 1000),       (float)(vel_zyz_t[2] * 1000),
    //                                (float)(vel_zyz_t[3] / M_PI * 180), (float)(vel_zyz_t[4] / M_PI * 180), (float)(vel_zyz_t[5] / M_PI * 180)};
    //     float acceleration_arrary[6] = {(float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration};

    //     Drfl.speedl_rt(vel_sat_arrary, acceleration_arrary, m_dt);
    // }
    // else if (frame == vpRobot::CAMERA_FRAME)
    // {
    //     double acceleration = 0.25 * 1000;
    //     vpColVector w_v_e = vpVelocityTwistMatrix(m_eMc) * vpVelocityTwistMatrix(get_fMe(), false) * vel_sat;

    //     vpPoseVector current_pose_xyz;
    //     getPosition(vpRobot::END_EFFECTOR_FRAME, current_pose_xyz);

    //     vpRxyzVector current_pose_euler_xyz(current_pose_xyz[3], current_pose_xyz[4], current_pose_xyz[5]);
    //     vpRzyxVector current_pose_euler_zyz(current_pose_euler_xyz);

    //     vpRxyzVector w_v_e_euler_xyz(w_v_e[3], w_v_e[4], w_v_e[5]);

    //     vpRxyzVector target_pose_euler_xyz(current_pose_euler_xyz[0] + w_v_e_euler_xyz[0] * m_dt, current_pose_euler_xyz[1] + w_v_e_euler_xyz[1] * m_dt,
    //                                        current_pose_euler_xyz[2] + w_v_e_euler_xyz[2] * m_dt);
    //     vpRxyzVector target_pose_euler_zyz(target_pose_euler_xyz);
    //     vpColVector vel_ = {w_v_e[0],
    //                         w_v_e[1],
    //                         w_v_e[2],
    //                         (target_pose_euler_zyz[0] - current_pose_euler_zyz[0]) / m_dt,
    //                         (target_pose_euler_zyz[1] - current_pose_euler_zyz[1]) / m_dt,
    //                         (target_pose_euler_zyz[2] - current_pose_euler_zyz[2]) / m_dt};

    //     std::cout << "vel_zyz_t:" << vel_ << std::endl;
    //     // Compute the saturated velocity skew vector
    //     vpColVector vel_zyz_t = vpRobot::saturateVelocities(vel_, v_max, true);

    //     float vel_sat_arrary[6] = {(float)(vel_zyz_t[0] * 1000),       (float)(vel_zyz_t[1] * 1000),       (float)(vel_zyz_t[2] * 1000),
    //                                (float)(vel_zyz_t[3] / M_PI * 180), (float)(vel_zyz_t[4] / M_PI * 180), (float)(vel_zyz_t[5] / M_PI * 180)};
    //     float acceleration_arrary[6] = {(float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration, (float)acceleration};

    //     Drfl.speedl_rt(vel_sat_arrary, acceleration_arrary, m_dt);
    // }
    // else
    // {
    //     throw(vpException(vpRobotException::functionNotImplementedError, "Cannot move the robot in velocity in the specified frame: not implemented"));
    // }
}

/*!
 * Change the robot state.
 *
 * \param[in] newState : New requested robot state.
 */
vpRobot::vpRobotStateType vpRobotAuboRobots::setRobotState(vpRobot::vpRobotStateType newState)
{
    switch (newState)
    {
        case vpRobot::STATE_STOP: {
            // Start primitive STOP only if the current state is Velocity
            if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState())
            {
                if (m_connected != aubo_robot_namespace::InterfaceCallSuccCode)
                {
                    throw(vpException(vpException::fatalError, "Cannot set Doosan robot position: robot is not connected"));
                }
            }
            break;
        }
        case vpRobot::STATE_POSITION_CONTROL: {
            if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState())
            {
                if (m_connected != aubo_robot_namespace::InterfaceCallSuccCode)
                {
                    throw(vpException(vpException::fatalError, "Cannot set Doosan robot position: robot is not connected"));
                }
            }
            else
            {
                // std::cout << "Change the control mode from stop to position control" << std::endl;
            }
            break;
        }
        case vpRobot::STATE_VELOCITY_CONTROL: {
            if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState())
            {
                std::cout << "Change the control mode from stop to velocity control.\n";
            }
            break;
        }
        default: break;
    }

    return vpRobot::setRobotState(newState);
}

void vpRobotAuboRobots::stopMotion()
{
    if (!m_connected && !m_rt_connected)
    {
        throw(vpException(vpException::fatalError, "Cannot stop UR robot: robot is not connected"));
    }

    setRobotState(vpRobot::STATE_STOP);
}

void vpRobotAuboRobots::move(const std::string &filename, double velocity_percentage)
{
    vpColVector q;

    setRobotState(vpRobot::STATE_POSITION_CONTROL);
    setPositioningVelocity(velocity_percentage);
    setPosition(vpRobot::JOINT_STATE, q);
}

int vpRobotAuboRobots::connectRtServer(const std::string &ip, int port)
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

    if (::connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        // ROS_ERROR("Connection Failed");
        close(sock);
        return -1;
    }

    // ROS_INFO("Connected to server at %s:%d", ip.c_str(), port);
    return sock;
}

void vpRobotAuboRobots::parseJointData(const std::string &data, std::vector<std::string> &joint_names, std::vector<double> &joint_positions)
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

void vpRobotAuboRobots::realTimeCalJointStatus(const std::string &tcp_ip)
{
    int tcp_port = 8891;
    int sock = connectRtServer(tcp_ip, tcp_port);
    if (sock < 0)
    {
        std::cout << "read fail" << std::endl;
        return;
    }

    char buffer[1550] = {0};
    std::string accumulated_data;
    auto last = std::chrono::system_clock::now();
    auto now = std::chrono::system_clock::now();
    while (m_rt_connected == aubo_robot_namespace::InterfaceCallSuccCode)
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
                    std::lock_guard<mutex> lock(m_tcp_mutex);
                    m_rt_joints_postions = joint_positions;
                }
                m_tcp_cv.notify_one();
                // 输出计算结果
                now = std::chrono::system_clock::now();
                auto duration = std::chrono::duration_cast<chrono::microseconds>(now - last);
                last = now;
                // std::cout << "Duration:" << duration.count() << " Joint positions: ";
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
            sock = connectRtServer(tcp_ip, tcp_port);
            while (sock < 0)
            {
                sleep(1);
                sock = connectRtServer(tcp_ip, tcp_port);
            }
        }
        memset(buffer, 0, sizeof(buffer));
    }
    cout << "realTimeCalJointStatus end" << endl;
}

void vpRobotAuboRobots::realTimeSendTcp()
{
    while (!m_stop_tcp_flag)
    {
        {
            std::unique_lock<std::mutex> lock(m_tcp_mutex);
            m_tcp_cv.wait(lock); // 等待通知
        }

        // cout << cur_vel << endl;
        Eigen::MatrixXd joint_positions_matrix(6, 1);
        {
            // std::lock_guard<mutex> lock(mtx);
            for (int i = 0; i < 6; i++)
            {
                joint_positions_matrix(i) = m_rt_joints_postions[i];
            }
        }
        Eigen::MatrixXd jacobi_b = mr::JacobianBody(m_B_matrix, joint_positions_matrix);
        Eigen::MatrixXd inverse_jacobi_b = jacobi_b.completeOrthogonalDecomposition().pseudoInverse();
        // 矩阵乘以向量
        Eigen::VectorXd joint_velocities;
        {
            std::unique_lock<std::mutex> lock(m_vel_mutex);
            joint_velocities = inverse_jacobi_b * m_cur_vel;
        }

        // std::cout << "current joint: " << joint_positions_matrix.transpose() << std::endl;
        joint_positions_matrix += joint_velocities;
        double joint_angle[6];
        for (int i = 0; i < 6; i++) joint_angle[i] = joint_positions_matrix(i);
        int ret = m_robot_service.robotServiceSetRobotPosData2Canbus(joint_angle);
        // 输出计算结果
        // std::cout << "target joint: " << joint_positions_matrix.transpose() << std::endl;
    }
    cout << "realTimeSendTcp end" << endl;
    m_quit_cv.notify_one();
}

void vpRobotAuboRobots::readTxt(const string &filename, Eigen::VectorXi size, Eigen::MatrixXd &matrix)
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