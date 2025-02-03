#include "global_planner.h"

namespace Global_Planning{
// 初始化函数
void Global_Planner::init(ros::NodeHandle& nh){
    // 安全距离，若膨胀距离设置已考虑安全距离，建议此处设为0
    nh.param("global_planner/safe_distance", safe_distance, 0.05); 
    nh.param("global_planner/time_per_path", time_per_path, 1.0); 
    // 重规划频率 
    nh.param("global_planner/replan_time", replan_time, 2.0);
    nh.param("global_planner/map_groundtruth", map_groundtruth, false); 

    // 定时器 安全检测
    // safety_timer = nh.createTimer(ros::Duration(2.0), &Global_Planner::safety_cb, this); 
    // 定时器 规划器算法执行周期
    mainloop_timer = nh.createTimer(ros::Duration(0.1), &Global_Planner::mainloop_cb, this);
    // 路径追踪循环，快速移动场景应当适当提高执行频率
    // time_per_path
    track_path_timer = nh.createTimer(ros::Duration(0.2), &Global_Planner::track_path_cb, this);

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>
                ("/mavros/local_position/odom", 10, &Local_Planner::odometryCallback, this);
    // 订阅 目标点
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/prometheus/planning/goal", 1, &Global_Planner::goal_cb, this);
    // 地图更新
    Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>
            ("/prometheus/global_planning/global_pcl", 10, &Global_Planner::Gpointcloud_cb, this);

    // 发布 路径指令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    // 发布路径用于显示
    path_cmd_pub   = nh.advertise<nav_msgs::Path>("/prometheus/global_planning/path_cmd",  10);

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(4);

    cout << "[planner] Hybrid Astar Planner initialized!" << endl;

    Astar_ptr.reset(new KinodynamicAstar);
    Astar_ptr->init(nh);

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::WAIT_GOAL;
    goal_ready = false;
    is_safety = true;
    is_new_path = false;

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.source = NODE_NAME;
    desired_yaw = 0.0;
}

// 保存无人机当前里程计信息，包括位置、速度和姿态
void Global_Planner::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // TODO: add odom lost check
    have_odom_ = true;
    last_odom_stamp_ = ros::Time::now();

    odom_pos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;
    start_pos = odom_pos_;

    odom_vel_ << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;
    start_vel = odom_vel_;

    //odom_acc_ = estimateAcc( msg );
    start_acc.setZero();

    // 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    tf::Quaternion odom_q_(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
    );

    tf::Matrix3x3(odom_q_).getRPY(odom_roll_, odom_pitch_, odom_yaw_);

    if (local_alg_ptr) {
        local_alg_ptr->set_odom(*msg);
    } else {
        ROS_ERROR("local_alg_ptr is nullptr");
    }
}

void Global_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg){
    goal_pos << msg->pose.position.x, msg->pose.position.y, odom_pos_[2];
    
    goal_vel.setZero();

    goal_ready = true;
}

// 根据全局点云更新地图
// 情况：已知全局点云的场景、由SLAM实时获取的全局点云
void Global_Planner::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
        // 对Astar中的地图进行更新
        Astar_ptr->Occupy_map_ptr->map_update_gpcl(msg);
        // 并对地图进行膨胀
        Astar_ptr->Occupy_map_ptr->inflate_point_cloud();
}

void Global_Planner::track_path_cb(const ros::TimerEvent& e){
    if(!path_ok){
        return;
    }

    // if(!is_safety){
    //     // 若无人机与障碍物之间的距离小于安全距离，则停止执行路径
    //     // 但如何脱离该点呢？

    //     goal_ready = false;
    //     exec_state = EXEC_STATE::WAIT_GOAL;
        
    //     return;
    // }
    is_new_path = false;

    // 抵达终点
    if(cur_id >= Num_total_wp - 1){
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = goal_pos[0];
        Command_Now.Reference_State.position_ref[1]     = goal_pos[1];
        Command_Now.Reference_State.position_ref[2]     = goal_pos[2];
        Command_Now.Reference_State.yaw_ref             = desired_yaw;

        command_pub.publish(Command_Now);

        cout << "[planner] Reach the goal!" << endl;
        
        // 停止执行
        path_ok = false;
        // 转换状态为等待目标
        exec_state = EXEC_STATE::WAIT_GOAL;
        return;
    }

    int i = cur_id;

    cout << cur_id << "/"<< Num_total_wp<< " Moving to " <<
        path_cmd.poses[i].pose.position.x << ", " <<
        path_cmd.poses[i].pose.position.y << ", " <<
        path_cmd.poses[i].pose.position.z << endl;

    // 控制方式如果是走航点，则需要对无人机进行限速，保证无人机的平滑移动
    // 采用轨迹控制的方式进行追踪，期望速度 = （期望位置 - 当前位置）/预计时间；

    const float limit_velocity_x = 0.2;
    float velocity_x = (path_cmd.poses[i].pose.position.x - odom_pos_[0])/time_per_path;
    if(velocity_x < -limit_velocity_x){
        velocity_x = -limit_velocity_x;
    }else if(velocity_x > limit_velocity_x){
        velocity_x = limit_velocity_x;
    }

    const float limit_velocity_y = 0.2;
    float velocity_y = (path_cmd.poses[i].pose.position.y - odom_pos_[1])/time_per_path;
    if(velocity_y < -limit_velocity_y){
        velocity_y = -limit_velocity_y;
    }else if(velocity_y > limit_velocity_y){
        velocity_y = limit_velocity_y;
    }

    const float limit_velocity_z = 0.1;
    float velocity_z = (path_cmd.poses[i].pose.position.z - odom_pos_[2])/time_per_path;
    if(velocity_z < -limit_velocity_z){
        velocity_z = -limit_velocity_z;
    }else if(velocity_z > limit_velocity_z){
        velocity_z = limit_velocity_z;
    }

    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::TRAJECTORY;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = path_cmd.poses[i].pose.position.x;
    Command_Now.Reference_State.position_ref[1]     = path_cmd.poses[i].pose.position.y;
    Command_Now.Reference_State.position_ref[2]     = path_cmd.poses[i].pose.position.z;
    Command_Now.Reference_State.velocity_ref[0]     = velocity_x;
    Command_Now.Reference_State.velocity_ref[1]     = velocity_y;
    Command_Now.Reference_State.velocity_ref[2]     = velocity_z;
    Command_Now.Reference_State.yaw_ref             = desired_yaw;
    
    command_pub.publish(Command_Now);

    cout << velocity_x << " " << velocity_y << " " << velocity_z << endl;

    cur_id = cur_id + 1;
}
 
// 主循环 
void Global_Planner::mainloop_cb(const ros::TimerEvent& e)
{
    switch (exec_state){
        case WAIT_GOAL:{
            path_ok = false;
            if(goal_ready){
                // 获取到目标点后，生成新轨迹
                exec_state = EXEC_STATE::PLANNING;
                goal_ready = false;
            }
            break;
        }

        case PLANNING:{
            // 重置规划器
            Astar_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果

            bool init = false;
            bool dynamic = false;
            double time_start = 0;

            int astar_state = Astar_ptr->search(start_pos, start_vel, start_acc, goal_pos, goal_vel, init, dynamic, time_start);

            if(astar_state==KinodynamicAstar::NO_PATH){
                path_ok = false;
                exec_state = EXEC_STATE::WAIT_GOAL;

                     Command_Now.header.stamp = ros::Time::now();
                     Command_Now.Mode         = prometheus_msgs::ControlCommand::Hold;
                     Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                     Command_Now.source = NODE_NAME;
                     command_pub.publish(Command_Now);

                cout << "[planner] astar find no path, HOLD" << endl;
            }else{
                path_ok = true;
                is_new_path = true;
                path_cmd = Astar_ptr->get_ros_path();
                Num_total_wp = path_cmd.poses.size();
                start_point_index = get_start_point_id();
                cur_id = start_point_index;
                tra_start_time = ros::Time::now();
                path_cmd_pub.publish(path_cmd);
                cout << "astar find path success!" << endl;
            }

            break;
        }
    }
}

// 【获取当前时间函数】 单位：秒
float Global_Planner::get_time_in_sec(const ros::Time& begin_time){
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void Global_Planner::safety_cb(const ros::TimerEvent& e){
    Eigen::Vector3d cur_pos(odom_pos_[0], odom_pos_[1], odom_pos_[2]);
    
    is_safety = Astar_ptr->check_safety(cur_pos, safe_distance);
}

int Global_Planner::get_start_point_id(void){
    // 选择与当前无人机所在位置最近的点,并从该点开始追踪
    int id = 0;
    float distance_to_wp_min = abs(path_cmd.poses[0].pose.position.x - odom_pos_[0])
                                + abs(path_cmd.poses[0].pose.position.y - odom_pos_[1])
                                + abs(path_cmd.poses[0].pose.position.z - odom_pos_[2]);
    
    float distance_to_wp;

    for (int j=1; j<Num_total_wp;j++){
        distance_to_wp = abs(path_cmd.poses[j].pose.position.x - odom_pos_[0])
                                + abs(path_cmd.poses[j].pose.position.y - odom_pos_[1])
                                + abs(path_cmd.poses[j].pose.position.z - odom_pos_[2]);
        
        if(distance_to_wp < distance_to_wp_min){
            distance_to_wp_min = distance_to_wp;
            id = j;
        }
    }

    //　为防止出现回头的情况，此处对航点进行前馈处理
    if(id + 4 < Num_total_wp){
        id = id + 4;
    }
    return id;
}
}
