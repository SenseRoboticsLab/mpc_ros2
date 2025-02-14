/*
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include <map>
#include <math.h>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "trackRefTraj.h"
#include <Eigen/Core>
#include <Eigen/QR>

#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNodeROS2 : public rclcpp::Node
{
public:
    MPCNodeROS2();
    ~MPCNodeROS2();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _sub_gen_path;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _sub_path;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_goal;
//    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _sub_amcl;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_totalcost;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_ctecost;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_ethetacost;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pub_odompath;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_twist;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pub_mpctraj;
    rclcpp::TimerBase::SharedPtr _timer1;
//    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::Point _goal_pos;
    nav_msgs::msg::Odometry _odom;
    nav_msgs::msg::Path _odom_path, _mpc_traj;
    geometry_msgs::msg::Twist _twist_msg;

//    std::string _globalPath_topic, _goal_topic;
    std::string _map_frame, _odom_frame, _car_frame;

    MPC _mpc;
    std::map<std::string, double> _mpc_params;
    double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel,
            _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

    double _dt, _w, _throttle, _speed, _max_speed;
    double _pathLength, _goalRadius, _waypointsDist;
    int _controller_freq;
    bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;
    bool _heading_ready;

    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

    void odomCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void pathCB(const nav_msgs::msg::Path::SharedPtr pathMsg);
    void desiredPathCB(const nav_msgs::msg::Path::SharedPtr pathMsg);
//    void goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg);
//    void amclCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amclMsg);
    void controlLoopCB();

    nav_msgs::msg::Path _gen_path;
    unsigned int min_idx;

    double _mpc_etheta;
    double _mpc_cte;
    std::ofstream file;
    unsigned int idx;
    double _min_distance;
    int _thread_numbers;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _dis_pub;
}; // end of class

MPCNodeROS2::MPCNodeROS2() : Node("mpc_node")
{
    //Parameters for control loop
    this->declare_parameter("thread_numbers", 2);
    this->declare_parameter("pub_twist_cmd", true);
    this->declare_parameter("debug_info", true);
    this->declare_parameter("delay_mode", true);
    this->declare_parameter("max_speed", 0.50);
    this->declare_parameter("waypoints_dist", -1.0);
    this->declare_parameter("path_length", 2.0);
    this->declare_parameter("goal_radius", 0.5);
    this->declare_parameter("controller_freq", 10);

    _thread_numbers = this->get_parameter("thread_numbers").as_int();
    _pub_twist_flag = this->get_parameter("pub_twist_cmd").as_bool();
    _debug_info = this->get_parameter("debug_info").as_bool();
    _delay_mode = this->get_parameter("delay_mode").as_bool();
    _max_speed = this->get_parameter("max_speed").as_double();
    _waypointsDist = this->get_parameter("waypoints_dist").as_double();
    _pathLength = this->get_parameter("path_length").as_double();
    _goalRadius = this->get_parameter("goal_radius").as_double();
    _controller_freq = this->get_parameter("controller_freq").as_int();

    _dt = 1.0 / _controller_freq;

    //Parameter for MPC solver
    this->declare_parameter("mpc_steps", 20.0);
    this->declare_parameter("mpc_ref_cte", 0.0);
    this->declare_parameter("mpc_ref_vel", 1.0);
    this->declare_parameter("mpc_ref_etheta", 0.0);
    this->declare_parameter("mpc_w_cte", 5000.0);
    this->declare_parameter("mpc_w_etheta", 5000.0);
    this->declare_parameter("mpc_w_vel", 1.0);
    this->declare_parameter("mpc_w_angvel", 100.0);
    this->declare_parameter("mpc_w_angvel_d", 10.0);
    this->declare_parameter("mpc_w_accel", 50.0);
    this->declare_parameter("mpc_w_accel_d", 10.0);
    this->declare_parameter("mpc_max_angvel", 3.0);
    this->declare_parameter("mpc_max_throttle", 1.0);
    this->declare_parameter("mpc_bound_value", 1.0e3);

    _mpc_steps = this->get_parameter("mpc_steps").as_double();
    _ref_cte = this->get_parameter("mpc_ref_cte").as_double();
    _ref_vel = this->get_parameter("mpc_ref_vel").as_double();
    _ref_etheta = this->get_parameter("mpc_ref_etheta").as_double();
    _w_cte = this->get_parameter("mpc_w_cte").as_double();
    _w_etheta = this->get_parameter("mpc_w_etheta").as_double();
    _w_vel = this->get_parameter("mpc_w_vel").as_double();
    _w_angvel = this->get_parameter("mpc_w_angvel").as_double();
    _w_angvel_d = this->get_parameter("mpc_w_angvel_d").as_double();
    _w_accel = this->get_parameter("mpc_w_accel").as_double();
    _w_accel_d = this->get_parameter("mpc_w_accel_d").as_double();
    _max_angvel = this->get_parameter("mpc_max_angvel").as_double();
    _max_throttle = this->get_parameter("mpc_max_throttle").as_double();
    _bound_value = this->get_parameter("mpc_bound_value").as_double();


    //Parameter for topics & Frame name
//    this->declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");
    this->declare_parameter<std::string>("map_frame", "odom"); //*****for mpc, "odom"
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("car_frame", "base_footprint");

//    _globalPath_topic = this->get_parameter("global_path_topic").as_string();
//    _goal_topic = this->get_parameter("goal_topic").as_string();
    _map_frame = this->get_parameter("map_frame").as_string();
    _odom_frame = this->get_parameter("odom_frame").as_string();
    _car_frame = this->get_parameter("car_frame").as_string();


    //Display the parameters
    RCLCPP_INFO(this->get_logger(), "\n===== Parameters =====");
    RCLCPP_INFO(this->get_logger(), "pub_twist_cmd: %d", _pub_twist_flag);
    RCLCPP_INFO(this->get_logger(), "debug_info: %d", _debug_info);
    RCLCPP_INFO(this->get_logger(), "delay_mode: %d", _delay_mode);
    RCLCPP_INFO(this->get_logger(), "frequency: %f", _dt);
    RCLCPP_INFO(this->get_logger(), "mpc_steps: %f", _mpc_steps);
    RCLCPP_INFO(this->get_logger(), "mpc_ref_vel: %f", _ref_vel);
    RCLCPP_INFO(this->get_logger(), "mpc_w_cte: %f", _w_cte);
    RCLCPP_INFO(this->get_logger(), "mpc_w_etheta: %f", _w_etheta);
    RCLCPP_INFO(this->get_logger(), "mpc_max_angvel: %f", _max_angvel);

    //Publishers and Subscribers
    _sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&MPCNodeROS2::odomCB, this, std::placeholders::_1));
//    _sub_path = this->create_subscription<nav_msgs::msg::Path>(
//            _globalPath_topic, 1, std::bind(&MPCNodeROS2::pathCB, this, std::placeholders::_1));
    _sub_gen_path = this->create_subscription<nav_msgs::msg::Path>(
            "desired_path", 1, std::bind(&MPCNodeROS2::desiredPathCB, this, std::placeholders::_1));
//    _sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//            _goal_topic, 1, std::bind(&MPCNodeROS2::goalCB, this, std::placeholders::_1));
//    _sub_amcl = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//            "/amcl_pose", 5, std::bind(&MPCNodeROS2::amclCB, this, std::placeholders::_1));

    _pub_odompath = this->create_publisher<nav_msgs::msg::Path>("/mpc_reference", 1);
    _pub_mpctraj = this->create_publisher<nav_msgs::msg::Path>("/mpc_trajectory", 1);
    if (_pub_twist_flag)
        _pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    _pub_totalcost = this->create_publisher<std_msgs::msg::Float32>("/total_cost", 1);
    _pub_ctecost = this->create_publisher<std_msgs::msg::Float32>("/cross_track_error", 1);
    _pub_ethetacost = this->create_publisher<std_msgs::msg::Float32>("/theta_error", 1);

    //TF
//    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_node_base_interface());
//    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //Timer
    _timer1 = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / _controller_freq), std::bind(&MPCNodeROS2::controlLoopCB, this));

    //Init variables
    _goal_received = false;
    _goal_reached = false;
    _path_computed = false;
    _heading_ready = false;
    _throttle = 0.0;
    _w = 0.0;
    _speed = 0.0;

    _twist_msg = geometry_msgs::msg::Twist();
    _mpc_traj = nav_msgs::msg::Path();

    //Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    _mpc_params["STEPS"] = _mpc_steps;
    _mpc_params["REF_CTE"] = _ref_cte;
    _mpc_params["REF_ETHETA"] = _ref_etheta;
    _mpc_params["REF_V"] = _ref_vel;
    _mpc_params["W_CTE"] = _w_cte;
    _mpc_params["W_EPSI"] = _w_etheta;
    _mpc_params["W_V"] = _w_vel;
    _mpc_params["W_ANGVEL"] = _w_angvel;
    _mpc_params["W_A"] = _w_accel;
    _mpc_params["W_DANGVEL"] = _w_angvel_d;
    _mpc_params["W_DA"] = _w_accel_d;
    _mpc_params["ANGVEL"] = _max_angvel;
    _mpc_params["MAXTHR"] = _max_throttle;
    _mpc_params["BOUND"] = _bound_value;
    _mpc.LoadParams(_mpc_params);

    min_idx = 0;
    idx = 0;
    _mpc_etheta = 0;
    _mpc_cte = 0;
    file.open("/home/geonhee/catkin_ws/src/mpc_ros/mpc.csv");
    _min_distance = 0.0;
    _dis_pub = this->create_publisher<std_msgs::msg::Float64>("/min_track_distance", 1);
}

MPCNodeROS2::~MPCNodeROS2()
{
    file.close();
}


double MPCNodeROS2::polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd MPCNodeROS2::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

void MPCNodeROS2::odomCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    _odom = *odomMsg;
}

void MPCNodeROS2::pathCB(const nav_msgs::msg::Path::SharedPtr pathMsg)
{
}

void MPCNodeROS2::desiredPathCB(const nav_msgs::msg::Path::SharedPtr totalPathMsg)
{
    if (totalPathMsg->poses.size() == 0)
    {
        return;
    }
    _gen_path = *totalPathMsg;

    for (auto &pose : _gen_path.poses)
    {
        pose.pose.position.z = 0;
    }

    _goal_received = true;
    _goal_reached = false;
    _heading_ready = false;
    Eigen::Vector3d pos_cur(_odom.pose.pose.position.x, _odom.pose.pose.position.y, 0);
    Eigen::Vector3d goal_cur(_gen_path.poses.rbegin()->pose.position.x,
                             _gen_path.poses.rbegin()->pose.position.y,
                             0);
    double distance = (pos_cur - goal_cur).norm();
    if (distance < 0.6)
    {
        _goal_reached = true;
        min_idx = 0;
        RCLCPP_INFO(this->get_logger(), "goal reached!");
        return;
    }
    nav_msgs::msg::Path mpc_path = nav_msgs::msg::Path();
    geometry_msgs::msg::PoseStamped tempPose;
    nav_msgs::msg::Odometry odom = _odom;

    try
    {
        double total_length = 0.0;
        if (_waypointsDist <= 0.0)
        {
            double gap_x = totalPathMsg->poses[1].pose.position.x - totalPathMsg->poses[0].pose.position.x;
            double gap_y = totalPathMsg->poses[1].pose.position.y - totalPathMsg->poses[0].pose.position.y;
            _waypointsDist = sqrt(gap_x * gap_x + gap_y * gap_y);
        }

        double min_val = 100.0;
        int N = totalPathMsg->poses.size();
        const double px = odom.pose.pose.position.x;
        const double py = odom.pose.pose.position.y;
        const double ptheta = odom.pose.pose.position.y;

        double dx, dy;
        double pre_yaw = 0;
        double roll, pitch, yaw = 0;

        for (int i = 0; i < N; i++)
        {
            dx = totalPathMsg->poses[i].pose.position.x - px;
            dy = totalPathMsg->poses[i].pose.position.y - py;

            tf2::Quaternion q(
                    totalPathMsg->poses[i].pose.orientation.x,
                    totalPathMsg->poses[i].pose.orientation.y,
                    totalPathMsg->poses[i].pose.orientation.z,
                    totalPathMsg->poses[i].pose.orientation.w);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            if (abs(pre_yaw - yaw) > 5)
            {
                RCLCPP_INFO(this->get_logger(), "abs(pre_yaw - yaw): %f", abs(pre_yaw - yaw));
                pre_yaw = yaw;
            }

            if (min_val > sqrt(dx * dx + dy * dy))
            {
                min_val = sqrt(dx * dx + dy * dy);
                min_idx = i;
            }
        }
        _min_distance = min_val;
        for (int i = min_idx; i < N; i++)
        {
            if (total_length > _pathLength)
                break;

            mpc_path.poses.push_back(totalPathMsg->poses[i]);
            if (i > 0)
            {
                Eigen::Vector3d p_pre(totalPathMsg->poses[i - 1].pose.position.x,
                                      totalPathMsg->poses[i - 1].pose.position.y,
                                      totalPathMsg->poses[i - 1].pose.position.z);
                Eigen::Vector3d p_cur(totalPathMsg->poses[i].pose.position.x,
                                      totalPathMsg->poses[i].pose.position.y,
                                      totalPathMsg->poses[i].pose.position.z);
                total_length = total_length + (p_cur - p_pre).norm();
            }
        }

        if (total_length < _pathLength)
        {
            for (int i = 0; i < N; i++)
            {
                if (total_length > _pathLength)
                    break;

                // Define a timeout duration, for example 100 ms:
                rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.1);
                geometry_msgs::msg::PoseStamped tempPose;
//                tempPose = tf_buffer_->transform(totalPathMsg->poses[i], _odom_frame, timeout);

                mpc_path.poses.push_back(tempPose);
                total_length = total_length + _waypointsDist;
            }
        }

        if (mpc_path.poses.size() >= _pathLength)
        {
            _odom_path = mpc_path;
            _path_computed = true;
            mpc_path.header.frame_id = _odom_frame;
            mpc_path.header.stamp = this->get_clock()->now();
            _pub_odompath->publish(mpc_path);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Failed to path generation");
            _waypointsDist = -1;
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    }
}


//void MPCNodeROS2::goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg)
//{
//    _goal_pos = goalMsg->pose.position;
//    _goal_received = true;
//    _goal_reached = false;
//    RCLCPP_INFO(this->get_logger(), "Goal Received :goalCB!");
//}

//void MPCNodeROS2::amclCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amclMsg)
//{
//    if (_goal_received)
//    {
//        double car2goal_x = _goal_pos.x - amclMsg->pose.pose.position.x;
//        double car2goal_y = _goal_pos.y - amclMsg->pose.pose.position.y;
//        double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
//        if (dist2goal < _goalRadius)
//        {
//            _goal_received = false;
//            _goal_reached = true;
//            _path_computed = false;
//            RCLCPP_INFO(this->get_logger(), "Goal Reached !");
//        }
//    }
//}

void MPCNodeROS2::controlLoopCB()
{
    if (_goal_received && !_goal_reached && _path_computed)
    {
        Eigen::Vector3d pos_cur(_odom.pose.pose.position.x, _odom.pose.pose.position.y, 0);
        Eigen::Vector3d goal_cur(_odom_path.poses.rbegin()->pose.position.x,
                                 _odom_path.poses.rbegin()->pose.position.y,
                                 0);
        double distance = (pos_cur - goal_cur).norm();
        if (distance < 0.5)
        {
            _goal_reached = true;
            min_idx = 0;
            RCLCPP_INFO(this->get_logger(), "goal reached!");
            return;
        }

        Eigen::Quaterniond q_cur(_odom.pose.pose.orientation.w, _odom.pose.pose.orientation.x,
                                 _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z);
        Eigen::Vector3d p_cur(_odom.pose.pose.position.x,
                              _odom.pose.pose.position.y,
                              _odom.pose.pose.position.z);
        Eigen::Vector3d p_end(_odom_path.poses.rbegin()->pose.position.x,
                              _odom_path.poses.rbegin()->pose.position.y,
                              _odom_path.poses.rbegin()->pose.position.z);
        double yaw_goal = atan2((p_end - p_cur).y(), (p_end - p_cur).x());

        Eigen::Quaterniond q_goal(_odom_path.poses.rbegin()->pose.orientation.w,
                                  _odom_path.poses.rbegin()->pose.orientation.x,
                                  _odom_path.poses.rbegin()->pose.orientation.y,
                                  _odom_path.poses.rbegin()->pose.orientation.z);
        Eigen::AngleAxisd r_axis(q_cur);
        double yaw_cur = (r_axis.angle() * r_axis.axis()).z();
        double yaw_dis = yaw_goal - yaw_cur;
        if (yaw_dis > M_PI)
        {
            yaw_dis -= 2 * M_PI;
        }
        else if (yaw_dis < -M_PI)
        {
            yaw_dis += 2 * M_PI;
        }
        if (abs(yaw_dis) < M_PI * 45 / 180.0)
        {
            _heading_ready = true;
        }
        else
        {
            _twist_msg.linear.x = 0;
            _twist_msg.linear.y = 0;
            _twist_msg.linear.z = 0;
            _twist_msg.angular.x = 0;
            _twist_msg.angular.y = 0;
            _twist_msg.angular.z = yaw_dis > 0 ? _max_angvel : -_max_angvel;
            _pub_twist->publish(_twist_msg);
            return;
        }

        nav_msgs::msg::Odometry odom = _odom;
        nav_msgs::msg::Path odom_path = _odom_path;

        const double px = odom.pose.pose.position.x;
        const double py = odom.pose.pose.position.y;
        tf2::Quaternion q(
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        const double theta = yaw;
        const double v = odom.twist.twist.linear.x;

        const double w = _w;
        const double throttle = _throttle;
        const double dt = _dt;

        const int N = odom_path.poses.size();
        const double costheta = cos(theta);
        const double sintheta = sin(theta);

        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for (int i = 0; i < N; i++)
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
        }

        auto coeffs = polyfit(x_veh, y_veh, 3);

        const double cte = polyeval(coeffs, 0.0);
        const double etheta = atan(coeffs[1]);

        _mpc_cte = cte;
        _mpc_etheta = etheta;

        VectorXd state(6);
        if (_delay_mode)
        {
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt;
            const double v_act = v + throttle * dt;

            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;

            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }

        vector<double> mpc_results = _mpc.Solve(state, coeffs);

        _w = mpc_results[0];
        _throttle = mpc_results[1];
        _speed = v + _throttle * dt;
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if (_speed <= 0.0)
            _speed = 0.0;

        if (_debug_info)
        {
            RCLCPP_INFO(this->get_logger(), "\n\nDEBUG");
            RCLCPP_INFO(this->get_logger(), "theta: %f", theta);
            RCLCPP_INFO(this->get_logger(), "V: %f", v);
            RCLCPP_INFO(this->get_logger(), "coeffs: \n%f %f %f %f", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
            RCLCPP_INFO(this->get_logger(), "_w: %f", _w);
            RCLCPP_INFO(this->get_logger(), "_throttle: %f", _throttle);
            RCLCPP_INFO(this->get_logger(), "_speed: %f", _speed);
        }

        _mpc_traj = nav_msgs::msg::Path();
        _mpc_traj.header.frame_id = _car_frame;
        _mpc_traj.header.stamp = this->get_clock()->now();
        for (int i = 0; i < _mpc.mpc_x.size(); i++)
        {
            geometry_msgs::msg::PoseStamped tempPose;
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_traj.poses.push_back(tempPose);
        }
        _pub_mpctraj->publish(_mpc_traj);
    }
    else
    {
        _throttle = 0.0;
        _speed = 0.0;
        _w = 0;
        if (_goal_reached && _goal_received)
            RCLCPP_INFO(this->get_logger(), "Goal Reached: control loop !");
    }

    if (_pub_twist_flag)
    {
        _twist_msg.linear.x = _speed;
        _twist_msg.angular.z = _w;
        _pub_twist->publish(_twist_msg);

        std_msgs::msg::Float32 mpc_total_cost;
        mpc_total_cost.data = static_cast<float>(_mpc._mpc_totalcost);
        _pub_totalcost->publish(mpc_total_cost);

        std_msgs::msg::Float32 mpc_cte_cost;
        mpc_cte_cost.data = static_cast<float>(_mpc._mpc_ctecost);
        _pub_ctecost->publish(mpc_cte_cost);

        std_msgs::msg::Float32 mpc_etheta_cost;
        mpc_etheta_cost.data = static_cast<float>(_mpc._mpc_ethetacost);
        _pub_ethetacost->publish(mpc_etheta_cost);

        idx++;
        file << idx << "," << _mpc_cte << "," << _mpc_etheta << "," << _twist_msg.linear.x << "," << _twist_msg.angular.z << ",";
    }
    else
    {
        _twist_msg.linear.x = 0;
        _twist_msg.angular.z = 0;
        _pub_twist->publish(_twist_msg);
    }

    std_msgs::msg::Float64 msg;
    msg.data = _min_distance;
    _dis_pub->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCNodeROS2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}