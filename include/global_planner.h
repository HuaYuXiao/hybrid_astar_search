#ifndef GLOBAL_PLANNER
#define GLOBAL_PLANNER

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <easondrone_msgs/ControlCommand.h>
#include "kinodynamic_astar.h"
#include "occupy_map.h"

using namespace std;

#define MIN_DIS 0.2

namespace hybrid_astar_search{
class Global_Planner{
private:
    ros::NodeHandle nh;

    // odometry state
    ros::Subscriber odom_sub_;
    bool have_odom_;
    // TODO: change to odom lost check
    ros::Time last_odom_stamp_;
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
    double odom_roll_, odom_pitch_, odom_yaw_;

    // 参数
    double safe_distance;
    double time_per_path;
    double replan_time;
    bool consider_neighbour;

    // 根据不同的输入（激光雷达输入、相机输入等）生成occupymap
    // 调用路径规划算法 生成路径
    // 调用轨迹优化算法 规划轨迹

    ros::Timer mainloop_timer, track_path_timer, safety_timer;

    // 订阅无人机状态、目标点、传感器数据（生成地图）
    ros::Subscriber goal_sub;
    // 支持直接输入全局已知点云
    ros::Subscriber Lpointcloud_sub;

    // A星规划器
    KinodynamicAstar::Ptr Astar_ptr;

    nav_msgs::Path path_cmd;
    ros::Publisher path_cmd_pub;
    double distance_walked;

    // 无人机当前执行命令
    easondrone_msgs::ControlCommand ctrl_cmd_out_;
    // 发布控制指令
    ros::Publisher easondrone_ctrl_pub;

    double distance_to_goal;

    // 规划器状态
    bool goal_ready; 
    bool is_safety;
    bool is_new_path;
    bool path_ok;
    int start_point_index;
    int Num_total_wp;
    int cur_id;

    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, start_vel, start_acc, goal_pos, goal_vel;
    float desired_yaw;

    ros::Time tra_start_time;
    
    // 打印的提示消息
    string message;

    // 五种状态机
    enum EXEC_STATE{
        WAIT_GOAL,
        PLANNING,
    };
    EXEC_STATE exec_state;

    // 回调函数
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);

    void safety_cb(const ros::TimerEvent& e);
    void mainloop_cb(const ros::TimerEvent& e);
    void track_path_cb(const ros::TimerEvent& e);
   
    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const ros::Time& begin_time);

    int get_start_point_id(void);
    
public:
    Global_Planner(void): nh("~"){}
    ~Global_Planner(){}

    void init(ros::NodeHandle& nh);
};
}

#endif
