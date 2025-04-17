#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mine_detection/MineArray.h>
#include <mine_detection/UAVStatus.h>
#include <mine_detection/Mine.h>
#include <mine_detection/WaypointStatus.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <mutex>
#include <cmath>

// UAV2状态机枚举
enum class UAV2State { WAIT, MOVING, HOVERING, VERIFYING, COMPLETED, MOVING_TO_BOUNDARY };

class UAV2ScanPlanner {
public:
    UAV2ScanPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        current_state_(UAV2State::WAIT),
        current_mine_index_(-1),
        waypoint_reached_(false),
        hover_start_time_(0),
        gen_(rd_()),
        dis_(0.0, 1.0),
        last_verified_region_id(0), // UAV1的区域ID从1开始，所以0是安全的初始值
        //pending_region_id_(-1),
        //has_pending_region_(false),
        current_receiving_region_id_(0)
    {
        // ... (参数加载和订阅者/发布者初始化保持不变) ...
        // 加载参数
        nh_.param<double>("hover_time", hover_time_, 3.0);
        nh_.param<double>("arrival_threshold", arrival_threshold_, 0.1); // 到达阈值
        nh_.param<double>("verification_probability", verification_probability_, 0.8);

        // 在构造函数中添加
        nh_.param<double>("region_width", region_width_, 4.0);     // 默认值4.0
        nh_.param<double>("region_length", region_length_, 6.0);   // 默认值6.0
        nh_.param<double>("region_start_x", region_start_x_, -10.0); // 第一个区域起始X
        nh_.param<double>("region_start_y", region_start_y_, -9.0);  // Y轴中心坐标

        // 初始化订阅者 (确保话题名称正确)
        uav1_status_sub_ = nh_.subscribe("/uav1/scan_status", 10, &UAV2ScanPlanner::uav1StatusCallback, this);
        detected_mines_sub_ = nh_.subscribe("/uav1/detected_mines", 10, &UAV2ScanPlanner::detectedMinesCallback, this);
        odom_sub_ = nh_.subscribe("/uav2/odom", 10, &UAV2ScanPlanner::odomCallback, this); // 确认此话题名
        waypoint_status_sub_ = nh_.subscribe("/uav2/waypoint_status", 10, &UAV2ScanPlanner::egoStatusCallback, this); // 确认此话题名

        // 初始化发布者
        status_pub_ = nh_.advertise<mine_detection::UAVStatus>("/uav2/scan_status", 10);
        verified_mines_pub_ = nh_.advertise<mine_detection::MineArray>("/uav2/verified_mines", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav2/move_base_simple/goal", 10); // 确认此话题名

        // 等待订阅者连接 (特别是 EGO-Planner 的目标点订阅者)
        ros::Rate poll_rate(10); // 10 Hz
        while (ros::ok() && goal_pub_.getNumSubscribers() == 0) {
            ROS_INFO("UAV2: Waiting for goal subscribers on %s...", goal_pub_.getTopic().c_str());
            poll_rate.sleep();
        }
        ROS_INFO("UAV2: Goal subscribers connected.");
        /*    // 初始化位置，发送一个明确的悬停命令，覆盖EGO-Planner可能的初始行为
        ros::Duration(1.0).sleep(); // 等待系统稳定
        
        // 获取当前位置并发布为目标，使无人机保持在原地
        geometry_msgs::PoseStamped hover_cmd;
        hover_cmd.header.frame_id = "map";
        hover_cmd.header.stamp = ros::Time::now();
        hover_cmd.pose.position.x = -20.0; // 与init_x相同
        hover_cmd.pose.position.y = -12.0; // 与init_y相同
        hover_cmd.pose.position.z = 1.0;   // 与init_z相同
        hover_cmd.pose.orientation.w = 1.0;
        
        // 连续发送几次悬停命令，确保EGO-Planner接收
        for (int i = 0; i < 5; i++) {
            hover_cmd.header.stamp = ros::Time::now();
            goal_pub_.publish(hover_cmd);
            ros::Duration(0.1).sleep();
        } */

        // 启动状态机定时器
        state_machine_timer_ = nh_.createTimer(ros::Duration(0.5), &UAV2ScanPlanner::stateMachineCallback, this);

        // 发布初始状态
        //publishStatus();

        ROS_INFO("UAV2 Scan Planner Initialized. Current state: WAIT");
    }

private:
    // ... (成员变量声明保持不变) ...
    // ROS 相关成员变量
    ros::NodeHandle nh_;
    ros::Subscriber uav1_status_sub_, detected_mines_sub_, odom_sub_, waypoint_status_sub_;
    ros::Publisher status_pub_, verified_mines_pub_, goal_pub_;
    ros::Timer state_machine_timer_;

    UAV2State current_state_; //当前状态
    std::vector<geometry_msgs::PoseStamped> potential_mines_;//uav1发布的地雷点
    std::vector<geometry_msgs::PoseStamped> verified_mines_;//验证为真的地雷点
    int current_mine_index_;//当前正在验证的地雷索引
    int last_verified_region_id;//上一个验证的区域ID
    int current_receiving_region_id_; // 当前地雷点应归属的区域

    // 添加待处理的区域ID变量
    //int pending_region_id_;
    //bool has_pending_region_;

    // 按区域ID存储地雷信息
    std::map<int, std::vector<geometry_msgs::PoseStamped>> mines_by_region_;
    

    geometry_msgs::Pose current_pose_;//当前位置
    bool waypoint_reached_;//是否到达当前目标点
    //状态控制
    ros::Time hover_start_time_;//开始悬停的时间点
    std::mutex ego_mutex_;//保护共享数据的互斥锁

    //状态机相关参数
    double hover_time_;//悬停时间
    double arrival_threshold_;//判断到达的距离阈值(米)
    double verification_probability_;//地雷验证为真的概率值

    //随机模拟地雷验证的概率
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_;//0到1之间的均匀分布，用于地雷验证决策
    

    // 区域尺寸相关参数 
    double region_width_;    // 区域宽度
    double region_length_;   // 区域长度
    double region_start_x_;  // 第一个区域起始X坐标
    double region_start_y_;  // 区域Y轴中心坐标

    // 下一个区域边界位置
    geometry_msgs::Point next_region_boundary_;

    // --- 状态机核心逻辑 (保持不变) ---
    void stateMachineCallback(const ros::TimerEvent&) {
        // ... (内容不变) ...
        switch(current_state_) {
            case UAV2State::WAIT:
                break;
            case UAV2State::MOVING:
                handleMovingState();
                break;
            case UAV2State::HOVERING:
                handleHoveringState();
                break;
            case UAV2State::VERIFYING:
                handleVerifyingState();
                break;
            case UAV2State::COMPLETED:
                break;
            case UAV2State::MOVING_TO_BOUNDARY://如果上一个区域没有地雷点，就飞到下一个区域的边界
                handleMovingToBoundaryState();
                break;
        }
        //publishStatus();
    }

    // --- 移动到地雷点，状态处理函数 (保持不变) ---
    void handleMovingState() {
        // ... (内容不变) ...
        std::lock_guard<std::mutex> lock(ego_mutex_);
        if (waypoint_reached_) {
            ROS_INFO("UAV2: EGO reported arrival at potential mine %d. Starting hover.", current_mine_index_ + 1);
            waypoint_reached_ = false;
            hover_start_time_ = ros::Time::now();
            current_state_ = UAV2State::HOVERING;
        }
        else if (checkArrivalByPosition()) {
             ROS_INFO("UAV2: Arrival detected by position check for mine %d. Starting hover.", current_mine_index_ + 1);
             hover_start_time_ = ros::Time::now();
             current_state_ = UAV2State::HOVERING;
        }
    }
    // --- 悬停状态处理函数 (保持不变) ---
    void handleHoveringState() {
        // ... (内容不变) ...
        if ((ros::Time::now() - hover_start_time_).toSec() >= hover_time_) {
            ROS_INFO("UAV2: Hover complete at mine %d. Starting verification.", current_mine_index_ + 1);
            current_state_ = UAV2State::VERIFYING;
        }
    }
    // --- 验证地雷状态处理函数 (保持不变) ---
    void handleVerifyingState() {
        // ... (内容不变) ...
        ROS_INFO("UAV2: Verifying potential mine %d.", current_mine_index_ + 1);
        if (dis_(gen_) < verification_probability_) {
            if (current_mine_index_ >= 0 && current_mine_index_ < potential_mines_.size()) {
                verified_mines_.push_back(potential_mines_[current_mine_index_]);
                ROS_INFO("UAV2: Mine %d VERIFIED as real.", current_mine_index_ + 1);
            }
        } else {
            ROS_INFO("UAV2: Mine %d identified as FALSE POSITIVE.", current_mine_index_ + 1);
        }
        proceedToNextMine();
    }
    
    // 新增：处理飞向边界状态的函数
    void handleMovingToBoundaryState() {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        
        // 检查是否到达边界位置
        double dx = current_pose_.position.x - next_region_boundary_.x;
        double dy = current_pose_.position.y - next_region_boundary_.y;
        double dz = current_pose_.position.z - next_region_boundary_.z;
        double dist_sq = dx*dx + dy*dy + dz*dz;
        
        if (dist_sq < (arrival_threshold_ * arrival_threshold_) || waypoint_reached_) {
            ROS_INFO("UAV2: Arrived at boundary position for next region. Waiting for UAV1...");
            waypoint_reached_ = false;
            current_state_ = UAV2State::COMPLETED;  // 回到COMPLETED状态等待UAV1的下一个区域
            //publishStatus();
        }
    }






    // --- 回调函数 (修改点) ---
    void uav1StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
        ROS_INFO("UAV2: Received status from UAV1 - region_id: %d. Current State: %d, Last Verified Region: %d",
                 msg->region_id, static_cast<int>(current_state_), last_verified_region_id);
        // 更新区域ID
        current_receiving_region_id_ = msg->region_id;
        // **关键检查**: 必须是新的区域ID，且UAV2当前处于等待或完成状态
        if (msg->region_id == last_verified_region_id+1 &&
            (current_state_ == UAV2State::WAIT || current_state_ == UAV2State::COMPLETED))
        {
            ROS_INFO("UAV2: UAV1 completed NEW region %d. Checking for mines to verify.", last_verified_region_id);
            int previous_last_verified = last_verified_region_id;
            //last_verified_region_id = msg->region_id; // 更新记录 
            auto it = mines_by_region_.find(previous_last_verified);
            // **检查是否有地雷**: 必须有地雷才能开始验证流程
            /* if (!potential_mines_.empty()) {
                ROS_INFO("UAV2: Starting verification process for %zu mines in region %d.",
                         potential_mines_.size(), previous_last_verified);
                optimizeMinePath(); // 优化地雷点访问顺序
                current_mine_index_ = 0; // 从第一个地雷开始
                verified_mines_.clear(); // 清空上个区域确认的地雷列表
                moveToCurrentMine(); // **触发移动**
                // moveToCurrentMine 会将状态设置为 MOVING */
            if (it != mines_by_region_.end() && !it->second.empty()) {
            // 有地雷，开始验证
            ROS_INFO("UAV2: Starting verification process for %zu mines in region %d.",
                     it->second.size(), previous_last_verified);
            
            // 设置当前要处理的地雷列表
            potential_mines_ = it->second;
            verified_mines_.clear();
            
            // 优化路径并开始验证
            optimizeMinePath();
            current_mine_index_ = 0;
            moveToCurrentMine();

            } else {
                // 修改：无地雷时，先通知UAV1当前区域已完成验证
                ROS_INFO("UAV2: No potential mines received for region %d. Moving to next boundary.", previous_last_verified);
                notifyVerificationCompleted();
                
                // 然后计算并飞向下一个区域边界
                calculateRegionBoundary(previous_last_verified+1);
                moveToBoundary();
            }
        } else if (current_state_ == UAV2State::MOVING_TO_BOUNDARY && 
                    msg->region_id == last_verified_region_id + 1) {
            // 如果已经在飞向边界，且收到了下一个区域的完成消息
            ROS_INFO("UAV2: Received next region completion while moving to boundary. Adapting...");
            int previous_last_verified =last_verified_region_id;
            // 更新区域ID
            //last_verified_region_id = msg->region_id; 
            if (!potential_mines_.empty()) {
                // 如果有地雷，中断飞向边界的过程，开始验证地雷
                ROS_INFO("UAV2: Found %zu mines for new region %d. Starting verification.",
                         potential_mines_.size(), previous_last_verified);
                optimizeMinePath();
                current_mine_index_ = 0;
                verified_mines_.clear();
                moveToCurrentMine();
            }
            else {
                // 仍然没有地雷，继续飞向下一个边界
                notifyVerificationCompleted();
                calculateRegionBoundary(previous_last_verified + 1);
                moveToBoundary();
            }
        }    
        else {
            // 如果UAV2正忙，收到新区域信号
            ROS_WARN("UAV2: Received UAV1 status for new region %d while busy (Current State: %d). Will process after current task.",
                     msg->region_id, static_cast<int>(current_state_));
            // 注意：这里假设UAV1的协作逻辑会等待UAV2完成。如果UAV1不等待，
            // 可能需要在这里设置一个标志，表示有新区域待处理。
           // 保存待处理区域ID
        //pending_region_id_ = msg->region_id;
        //has_pending_region_ = true;
    
   // ROS_INFO("UAV2: Queued region %d for processing after current task completes", pending_region_id_);
        }
    }

    void detectedMinesCallback(const mine_detection::MineArray::ConstPtr& msg) {
        // 估计当前地雷属于哪个区域 
        //int target_region_id = last_verified_region_id;
        int target_region_id = current_receiving_region_id_;
        ROS_INFO("UAV2: Received %zu potential mines from UAV1, estimating for region %d.", 
                 msg->mines.size(), target_region_id);
        
        // 获取或创建目标区域的地雷数组
        auto& region_mines = mines_by_region_[target_region_id];
        size_t previous_mine_count = region_mines.size();
        size_t duplicates_count = 0;
        
        for (size_t i = 0; i < msg->mines.size(); ++i) {
            const auto& mine = msg->mines[i];
            
            // 检查是否是重复的地雷点
            bool is_duplicate = false;
            for (const auto& existing_mine : region_mines) {
                double dx = existing_mine.pose.position.x - mine.position.x;
                double dy = existing_mine.pose.position.y - mine.position.y;
                double dist_sq = dx*dx + dy*dy;
                
                // 如果距离非常近（小于阈值），认为是重复点
                if (dist_sq < 0.01) { // 10厘米阈值
                    is_duplicate = true;
                    duplicates_count++;
                    ROS_WARN("UAV2: Skipping duplicate mine at (%.2f, %.2f, %.2f) for region %d",
                            mine.position.x, mine.position.y, mine.position.z, target_region_id);
                    break;
                }
            }
            
            // 只添加非重复的地雷点
            if (!is_duplicate) {
                ROS_INFO("UAV2: Adding Potential Mine %zu at (%.2f, %.2f, %.2f) to region %d", 
                        region_mines.size() + 1, mine.position.x, mine.position.y, mine.position.z,
                        target_region_id);
                
                geometry_msgs::PoseStamped mine_pose;
                mine_pose.header = msg->header;
                mine_pose.header.stamp = ros::Time(0);
                mine_pose.pose.position = mine.position;
                mine_pose.pose.position.z = 0.5; // UAV2验证高度
                mine_pose.pose.orientation.w = 1.0;
                region_mines.push_back(mine_pose);
            }
        }
        
        // 添加新的日志
        if (duplicates_count > 0) {
            ROS_WARN("UAV2: Filtered out %zu duplicate mines for region %d", 
                     duplicates_count, target_region_id);
        }
        
        ROS_INFO("UAV2: Region %d now has %zu potential mines", 
                 target_region_id, region_mines.size());
    }

    // --- odomCallback 和 egoStatusCallback (保持不变) ---
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // ... (内容不变) ...
        std::lock_guard<std::mutex> lock(ego_mutex_);
        current_pose_ = msg->pose.pose;
    }

    void egoStatusCallback(const mine_detection::WaypointStatus::ConstPtr& msg) {
        // ... (内容不变) ...
        std::lock_guard<std::mutex> lock(ego_mutex_);
        if (current_state_ == UAV2State::MOVING && msg->isReached) {
            if (current_mine_index_ >= 0 && current_mine_index_ < potential_mines_.size()) {
                double dx = msg->position.x - potential_mines_[current_mine_index_].pose.position.x;
                double dy = msg->position.y - potential_mines_[current_mine_index_].pose.position.y;
                double dist_xy_sq = dx*dx + dy*dy;
                if (dist_xy_sq < (arrival_threshold_ * 1.5) * (arrival_threshold_ * 1.5) ) {
                    waypoint_reached_ = true;
                    ROS_INFO("UAV2: WaypointStatus confirms arrival near mine %d.", current_mine_index_ + 1);
                } else {
                    ROS_WARN("UAV2: WaypointStatus position mismatch. Ignoring status.");
                }
            } else {
                 ROS_WARN("UAV2: Received WaypointStatus while not expecting.");
            }
        }
    }


    // 新增：优化地雷点访问顺序的函数
    void optimizeMinePath() {
        if (potential_mines_.empty()) {
            ROS_WARN("UAV2: No mines to optimize path for");
            return;
        }
        
        ROS_INFO("UAV2: Optimizing verification path for %zu mines", potential_mines_.size());
        
        // 重排地雷点顺序 - 基于贪心算法（最近邻）
        std::vector<geometry_msgs::PoseStamped> ordered_mines;
        std::vector<bool> visited(potential_mines_.size(), false);
        
        // 当前位置作为起点
        geometry_msgs::Point current = current_pose_.position;
        
        // 找出所有地雷的最佳访问顺序
        while (ordered_mines.size() < potential_mines_.size()) {
            double min_dist = std::numeric_limits<double>::max();
            int next_idx = -1;
            
            for (size_t i = 0; i < potential_mines_.size(); ++i) {
                if (!visited[i]) {
                    double dx = potential_mines_[i].pose.position.x - current.x;
                    double dy = potential_mines_[i].pose.position.y - current.y;
                    double dz = potential_mines_[i].pose.position.z - current.z;
                    double dist = dx*dx + dy*dy + dz*dz; // 距离平方
                    
                    if (dist < min_dist) {
                        min_dist = dist;
                        next_idx = i;
                    }
                }
            }
            
            if (next_idx >= 0) {
                ROS_INFO("UAV2: Adding mine at (%.2f, %.2f, %.2f) to optimized path at position %zu",
                         potential_mines_[next_idx].pose.position.x,
                         potential_mines_[next_idx].pose.position.y,
                         potential_mines_[next_idx].pose.position.z,
                         ordered_mines.size()+1);
                
                ordered_mines.push_back(potential_mines_[next_idx]);
                visited[next_idx] = true;
                current = potential_mines_[next_idx].pose.position;
            }
        }
        
        // 打印优化前后的总路径长度，用于对比
        double original_path_length = calculatePathLength(potential_mines_);
        double optimized_path_length = calculatePathLength(ordered_mines);
        ROS_INFO("UAV2: Path optimization: original length %.2f m, optimized length %.2f m, improvement %.2f%%",
                original_path_length, optimized_path_length, 
                100.0 * (original_path_length - optimized_path_length) / original_path_length);
        
        // 用优化后的顺序替换原列表
        potential_mines_ = ordered_mines;
    }

     // 辅助函数：计算路径总长度
    double calculatePathLength(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (path.size() < 2) return 0.0;
        
        double total_length = 0.0;
        // 从当前位置到第一个点
        double dx = path[0].pose.position.x - current_pose_.position.x;
        double dy = path[0].pose.position.y - current_pose_.position.y;
        double dz = path[0].pose.position.z - current_pose_.position.z;
        total_length = sqrt(dx*dx + dy*dy + dz*dz);
        
        // 各点之间的距离
        for (size_t i = 0; i < path.size()-1; ++i) {
            dx = path[i+1].pose.position.x - path[i].pose.position.x;
            dy = path[i+1].pose.position.y - path[i].pose.position.y;
            dz = path[i+1].pose.position.z - path[i].pose.position.z;
            total_length += sqrt(dx*dx + dy*dy + dz*dz);
        }
        
        return total_length;
    }


    // 计算指定区域ID的边界位置
    void calculateRegionBoundary(int region_id) {
        // 计算指定区域的右边界位置（即下一个区域的起始位置）
        next_region_boundary_.x = region_start_x_ + region_id * region_width_;
        next_region_boundary_.y = region_start_y_; // Y保持在中心线
        next_region_boundary_.z = 1.5; // 保持安全飞行高度
        
        ROS_INFO("UAV2: Calculated boundary position for region %d at (%.2f, %.2f, %.2f)",
                region_id, next_region_boundary_.x, next_region_boundary_.y, next_region_boundary_.z);
    }

    // 移动到下一个区域边界
    void moveToBoundary() {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position = next_region_boundary_;
        goal.pose.orientation.w = 1.0;
        
        ROS_INFO("UAV2: Moving to boundary position for next region at (%.2f, %.2f, %.2f)",
                next_region_boundary_.x, next_region_boundary_.y, next_region_boundary_.z);
        
        // 发布目标点
        goal_pub_.publish(goal);
        
        // 更新状态
        waypoint_reached_ = false;
        current_state_ = UAV2State::MOVING_TO_BOUNDARY;
        //publishStatus();
    }

    // --- 飞往地雷点 (修改点) ---
    void moveToCurrentMine() {
        // ... (内容不变，除了状态设置) ...
        if (current_mine_index_ < 0 || current_mine_index_ >= potential_mines_.size()) {
            ROS_ERROR("UAV2: Invalid mine index %d requested for movement.", current_mine_index_);
            finishVerificationProcess(); // 调用完成处理
            return;
        }
        geometry_msgs::PoseStamped goal = potential_mines_[current_mine_index_];
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        ROS_INFO("UAV2: Publishing goal for potential mine %d at (%.2f, %.2f, %.2f)",
                 current_mine_index_ + 1,
                 goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        goal_pub_.publish(goal);
        waypoint_reached_ = false;
        current_state_ = UAV2State::MOVING; // **确保在这里设置状态**
        //publishStatus();
    }
    // --- 处理下一个地雷 (修改点) ---
    void proceedToNextMine() {
        // ... (内容不变) ...
        current_mine_index_++;
        if (current_mine_index_ < potential_mines_.size()) {
            ROS_INFO("UAV2: Proceeding to verify next potential mine %d / %zu.",
                     current_mine_index_ + 1, potential_mines_.size());
            moveToCurrentMine();
        } else {
            ROS_INFO("UAV2: Finished verifying all %zu potential mines for region %d.",
                     potential_mines_.size(), last_verified_region_id);
            finishVerificationProcess();
        }
    }

    // **修改**: 确保清理 potential_mines_
    void finishVerificationProcess() {
        publishVerifiedMines();
        notifyVerificationCompleted();
        // 清理已处理区域的数据
        int completed_region_id = last_verified_region_id-1; // 处理完的区域ID
        mines_by_region_.erase(completed_region_id); // 从存储中移除已处理的区域
        ROS_INFO("UAV2: Removed processed mines for region %d from storage", completed_region_id);

        // 清理已处理区域的数据
        potential_mines_.clear(); // **在这里清空潜在地雷列表**
        verified_mines_.clear();
        current_mine_index_ = -1;



        /*  // 检查是否有待处理的区域
        if (has_pending_region_) {
            ROS_INFO("UAV2: Found pending region %d. Starting its processing immediately.", pending_region_id_);
            
            // 创建临时消息，模拟收到UAV1状态消息
            mine_detection::UAVStatus pending_msg;
            pending_msg.region_id = pending_region_id_;
            
            // 重置待处理标志
            has_pending_region_ = false;
            pending_region_id_ = -1;
            
            // 设置为COMPLETED状态，以便能接收新区域处理
            current_state_ = UAV2State::COMPLETED;
            //publishStatus();
            
            // 延迟一小段时间后处理待定区域，确保状态已更新
            ros::Duration(0.2).sleep();
            
            // 处理等待中的区域
            uav1StatusCallback(boost::make_shared<mine_detection::UAVStatus>(pending_msg));
        } else {
            // 无待处理区域，正常进入完成状态
            current_state_ = UAV2State::COMPLETED;
            //publishStatus();
            ROS_INFO("UAV2: Verification process finished for region %d. State set to COMPLETED.", last_verified_region_id-1);
        } */
        current_state_ = UAV2State::COMPLETED;
        ROS_INFO("UAV2: Verification process finished for region %d. State set to COMPLETED.", last_verified_region_id-1);
    }

    // --- publishVerifiedMines, notifyVerificationCompleted, publishStatus, checkArrivalByPosition (保持不变) ---
    void publishVerifiedMines() {
        // ... (内容不变) ...
        if (verified_mines_.empty()) {
            ROS_INFO("UAV2: No mines verified as real in region %d.", last_verified_region_id);
            return;
        }
        mine_detection::MineArray mines_msg;
        mines_msg.header.stamp = ros::Time::now();
        mines_msg.header.frame_id = "map";
        for (const auto& pose_stamped : verified_mines_) {
            mine_detection::Mine mine;
            mine.position = pose_stamped.pose.position;
            mine.position.z = 0.0;
            mines_msg.mines.push_back(mine);
        }
        verified_mines_pub_.publish(mines_msg);
        ROS_INFO("UAV2: Published %zu verified mines for region %d.", mines_msg.mines.size(), last_verified_region_id);
    }

    void notifyVerificationCompleted() {
        // ... (内容不变) ...
        mine_detection::UAVStatus status_msg;
        last_verified_region_id++;
        status_msg.region_id = last_verified_region_id;
        status_pub_.publish(status_msg);
        ROS_INFO("UAV2: Notified completion for region %d.", last_verified_region_id-1);
    }

    /* void publishStatus() {
        // ... (内容不变) ...
        mine_detection::UAVStatus status_msg;
        status_msg.region_id = last_verified_region_id;
        status_pub_.publish(status_msg);
    } */

    bool checkArrivalByPosition() {
        if (current_mine_index_ < 0 || current_mine_index_ >= potential_mines_.size()) {
            return false;
        }
        
        const auto& target_pos = potential_mines_[current_mine_index_].pose.position;
        double dx = current_pose_.position.x - target_pos.x;
        double dy = current_pose_.position.y - target_pos.y;
        double dz = current_pose_.position.z - target_pos.z;
        
        // 计算三维欧几里得距离平方
        double dist_sq = dx*dx + dy*dy + dz*dz;
        
        // 添加详细日志，帮助调试
        ROS_DEBUG("UAV2: Distance to mine %d: %.2f m (threshold: %.2f m)",
                  current_mine_index_ + 1, sqrt(dist_sq), arrival_threshold_);
        ROS_DEBUG("UAV2: Current pos: (%.2f, %.2f, %.2f), Target: (%.2f, %.2f, %.2f)",
                  current_pose_.position.x, current_pose_.position.y, current_pose_.position.z,
                  target_pos.x, target_pos.y, target_pos.z);
        
        bool arrived = dist_sq < arrival_threshold_ ;
        return arrived;
    }
};

// --- main 函数 (保持不变) ---
int main(int argc, char** argv) {
    // ... (内容不变) ...
    ros::init(argc, argv, "uav2_scan_planner_node");
    ros::NodeHandle nh("~");
    UAV2ScanPlanner planner(nh);
    ros::spin();
    return 0;
}