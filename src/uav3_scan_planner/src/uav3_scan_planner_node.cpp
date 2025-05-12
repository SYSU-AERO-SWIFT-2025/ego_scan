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

// UAV3状态机枚举
enum class UAV3State { WAIT, MOVING,VERIFYING,MOVING_TO_BOUNDARY };

class UAV3ScanPlanner {
public:
    UAV3ScanPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        current_state_(UAV3State::WAIT),
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
        nh_.param<double>("hover_time", hover_time_, 5.0);
        nh_.param<double>("arrival_threshold", arrival_threshold_, 0.1); // 到达阈值
        nh_.param<double>("verification_probability", verification_probability_, 0.8);

        // 在构造函数中添加
        nh_.param<double>("region_width", region_width_, 4.0);     // 默认值4.0
        nh_.param<double>("region_length", region_length_, 4.0);   // 默认值4.0，和uav1统一
        nh_.param<double>("region_start_x", region_start_x_, -10.0); // 第一个区域起始X
        nh_.param<double>("region_start_y", region_start_y_, -9.0);  // Y轴中心坐标

        // 初始化订阅者 (确保话题名称正确)
        uav1_status_sub_ = nh_.subscribe("/uav1/scan_status", 10, &UAV3ScanPlanner::uav1StatusCallback, this);
        detected_mines_sub_ = nh_.subscribe("/uav1/detected_mines", 10, &UAV3ScanPlanner::detectedMinesCallback, this);
        odom_sub_ = nh_.subscribe("/uav3/odom", 10, &UAV3ScanPlanner::odomCallback, this); // 确认此话题名
        waypoint_status_sub_ = nh_.subscribe("/uav3/waypoint_status", 10, &UAV3ScanPlanner::egoStatusCallback, this); // 确认此话题名

        // 初始化发布者
        status_pub_ = nh_.advertise<mine_detection::UAVStatus>("/uav3/scan_status", 10);
        verified_mines_pub_ = nh_.advertise<mine_detection::MineArray>("/uav3/verified_mines", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav3/move_base_simple/goal", 10); // 确认此话题名

        // 等待订阅者连接 (特别是 EGO-Planner 的目标点订阅者)
        ros::Rate poll_rate(10); // 10 Hz
        while (ros::ok() && goal_pub_.getNumSubscribers() == 0) {
            ROS_INFO("UAV3: Waiting for goal subscribers on %s...", goal_pub_.getTopic().c_str());
            poll_rate.sleep();
        }
        ROS_INFO("UAV3: Goal subscribers connected.");
        // 启动状态机定时器
        state_machine_timer_ = nh_.createTimer(ros::Duration(0.5), &UAV3ScanPlanner::stateMachineCallback, this);

        // 发布初始状态
        //publishStatus();

        ROS_INFO("UAV3 Scan Planner Initialized. Current state: WAIT");
    }

private:
    // ... (成员变量声明保持不变) ...
    // ROS 相关成员变量
    ros::NodeHandle nh_;
    ros::Subscriber uav1_status_sub_, detected_mines_sub_, odom_sub_, waypoint_status_sub_;
    ros::Publisher status_pub_, verified_mines_pub_, goal_pub_;
    ros::Timer state_machine_timer_;

    UAV3State current_state_; //当前状态
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
    

    geometry_msgs::Point current_position;//当前位置
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

    int last_goal_index_ = -1; // 上一个目标点索引

    // --- 状态机核心逻辑 (保持不变) ---
    void stateMachineCallback(const ros::TimerEvent&) {
        // ... (内容不变) ...
        state_machine_timer_.stop(); // 停止定时器，避免重复调用
        switch(current_state_) {
            case UAV3State::WAIT:
                handleWaitState();
                break;
            case UAV3State::MOVING:
                handleMovingState();
                break;
            case UAV3State::VERIFYING:
                handleVerifyingState();
                break;
            case UAV3State::MOVING_TO_BOUNDARY://如果上一个区域没有地雷点，就飞到下一个区域的边界
                handleMovingToBoundaryState();
                break;
        }
        state_machine_timer_.start();//重新启动定时器
        //publishStatus();
    }
    // --- 等待状态处理函数 (保持不变) ---
    void handleWaitState(){
        if(current_receiving_region_id_ ==last_verified_region_id+1){
            auto it = mines_by_region_.find(last_verified_region_id);
            if(it == mines_by_region_.end()&& it->second.empty()){
                // 如果没有地雷点，飞到下一个区域的边界
                ROS_INFO("UAV3: No mines detected in region %d. Moving to boundary.", last_verified_region_id);
                current_state_ = UAV3State::MOVING_TO_BOUNDARY;
                return;
            }
            else if (it != mines_by_region_.end() && !it->second.empty()) {
                // 如果有地雷点，开始验证
                potential_mines_ = it->second;
                verified_mines_.clear(); // 清空已验证的地雷列表
                optimizeMinePath(); // 优化地雷点顺序
                current_mine_index_ = 0; // 重置地雷索引
                last_goal_index_ = -1; // 重置上一个目标点索引
                current_state_ = UAV3State::MOVING;
            }
            else{
                ROS_WARN("UAV3: No new region detected or no mines to verify.");
            }
        }
            // 这里可以添加其他逻辑，比如等待UAV1的状态更新
        else{
            ROS_INFO("UAV3: Waiting for UAV1 to complete region %d.", last_verified_region_id);
        }

        }
    
    // --- 移动到地雷点，状态处理函数 (保持不变) ---
    void handleMovingState() {
        // ... (内容不变) ...
        std::lock_guard<std::mutex> lock(ego_mutex_);
        /* if(last_goal_index_!=current_mine_index_){
            moveToCurrentMine();
            last_goal_index_ = current_mine_index_;
        } */
         /// 判断是否到达目标地雷点
        if (waypoint_reached_ || checkArrivalByPosition()) {
            waypoint_reached_ = false;
            ROS_INFO("UAV3: Arrived at mine %d, start verifying...", current_mine_index_ + 1);
            current_state_ = UAV3State::VERIFYING;
        } else {
            // 未到达则持续等待
            ROS_INFO_THROTTLE(2, "UAV3: Moving to mine %d...", current_mine_index_ + 1);
           moveToCurrentMine();
        }
    }

  
    // --- 验证地雷状态处理函数 (保持不变) ---
    void handleVerifyingState() {
        if (simulateVisualMineDetection()) {
            current_mine_index_++;
            if (current_mine_index_ < potential_mines_.size()) {
                current_state_ = UAV3State::MOVING;
                ROS_INFO("UAV3: Moving to next potential mine %d.", current_mine_index_ + 1);
            } else {
                ROS_INFO("UAV3: Finished verifying all %zu potential mines for region %d.",
                         potential_mines_.size(), last_verified_region_id);
                finishVerificationProcess();
            }
        }
    }
    
    // 新增：处理飞向边界状态的函数
    void handleMovingToBoundaryState() {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        notifyVerificationCompleted();
        // 计算下一个区域的边界位置
        calculateRegionBoundary(last_verified_region_id+1);
        moveToBoundary();

        // 检查是否到达边界位置
        double dx = current_position.x - next_region_boundary_.x;
        double dy = current_position.y - next_region_boundary_.y;
        double dz = current_position.z - next_region_boundary_.z;
        double dist_sq = dx*dx + dy*dy + dz*dz;
        
        if (dist_sq < (arrival_threshold_ * arrival_threshold_) || waypoint_reached_) {
            ROS_INFO("UAV3: Arrived at boundary position for next region. Waiting for UAV1...");
            waypoint_reached_ = false;
            current_state_ = UAV3State::WAIT;  // 回到WAIT状态等待UAV1的下一个区域
            //publishStatus();
        }
    }

    // --- 回调函数 (修改点) ---
    void uav1StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
        ROS_INFO("UAV3: Received status from UAV1 - region_id: %d. Current State: %d, Last Verified Region: %d",
                 msg->region_id, static_cast<int>(current_state_), last_verified_region_id);
        // 更新区域ID
        current_receiving_region_id_ = msg->region_id;
    }

    void detectedMinesCallback(const mine_detection::MineArray::ConstPtr& msg) {
        // 估计当前地雷属于哪个区域 
        int target_region_id = last_verified_region_id;
        //int target_region_id = current_receiving_region_id_;
        ROS_INFO("UAV3: Received %zu potential mines from UAV1, estimating for region %d.", 
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
                    ROS_WARN("UAV3: Skipping duplicate mine at (%.2f, %.2f, %.2f) for region %d",
                            mine.position.x, mine.position.y, mine.position.z, target_region_id);
                    break;
                }
            }
            
            // 只添加非重复的地雷点
            if (!is_duplicate) {
                ROS_INFO("UAV3: Adding Potential Mine %zu at (%.2f, %.2f, %.2f) to region %d", 
                        region_mines.size() + 1, mine.position.x, mine.position.y, mine.position.z,
                        target_region_id);
                
                geometry_msgs::PoseStamped mine_pose;
                mine_pose.header = msg->header;
                mine_pose.header.stamp = ros::Time(0);
                mine_pose.pose.position = mine.position;
                mine_pose.pose.position.z = 2.0; // UAV3验证高度
                mine_pose.pose.orientation.w = 1.0;
                region_mines.push_back(mine_pose);
            }
        }
        
        // 添加新的日志
        if (duplicates_count > 0) {
            ROS_WARN("UAV3: Filtered out %zu duplicate mines for region %d", 
                     duplicates_count, target_region_id);
        }
        
        ROS_INFO("UAV3: Region %d now has %zu potential mines", 
                 target_region_id, region_mines.size());
    }

    // --- odomCallback 和 egoStatusCallback (保持不变) ---
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // ... (内容不变) ...
        std::lock_guard<std::mutex> lock(ego_mutex_);
        current_position = msg->pose.pose.position;
    }

    void egoStatusCallback(const mine_detection::WaypointStatus::ConstPtr& msg) {
        // ... (内容不变) ...
        std::lock_guard<std::mutex> lock(ego_mutex_);
        if (msg->isReached) {
            waypoint_reached_ = true;
            //current_position = msg->position;
            ROS_INFO("UAV3: EGO reported arrival at waypoint.");
        }
    }


    // 新增：优化地雷点访问顺序的函数
    void optimizeMinePath() {
        if (potential_mines_.empty()) {
            ROS_WARN("UAV3: No mines to optimize path for");
            return;
        }
        
        ROS_INFO("UAV3: Optimizing verification path for %zu mines", potential_mines_.size());
        
        // 重排地雷点顺序 - 基于贪心算法（最近邻）
        std::vector<geometry_msgs::PoseStamped> ordered_mines;
        std::vector<bool> visited(potential_mines_.size(), false);
        
        // 当前位置作为起点
        geometry_msgs::Point current = current_position;
        
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
                ROS_INFO("UAV3: Adding mine at (%.2f, %.2f, %.2f) to optimized path at position %zu",
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
        ROS_INFO("UAV3: Path optimization: original length %.2f m, optimized length %.2f m, improvement %.2f%%",
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
        double dx = path[0].pose.position.x - current_position.x;
        double dy = path[0].pose.position.y - current_position.y;
        double dz = path[0].pose.position.z - current_position.z;
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
        
        ROS_INFO("UAV3: Calculated boundary position for region %d at (%.2f, %.2f, %.2f)",
                region_id, next_region_boundary_.x, next_region_boundary_.y, next_region_boundary_.z);
    }

    // 移动到下一个区域边界
    void moveToBoundary() {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position = next_region_boundary_;
        goal.pose.orientation.w = 1.0;
        
        ROS_INFO("UAV3: Moving to boundary position for next region at (%.2f, %.2f, %.2f)",
                next_region_boundary_.x, next_region_boundary_.y, next_region_boundary_.z);
        
        // 发布目标点
        goal_pub_.publish(goal);
        //publishStatus();
    }

    // --- 飞往地雷点 (修改点) ---
    void moveToCurrentMine() {
        // ... (内容不变，除了状态设置) ...
        if (current_mine_index_ < 0 || current_mine_index_ >= potential_mines_.size()) {
            ROS_ERROR("UAV3: Invalid mine index %d requested for movement.", current_mine_index_);
            finishVerificationProcess(); // 调用完成处理
            return;
        }
        geometry_msgs::PoseStamped goal = potential_mines_[current_mine_index_];
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        ROS_INFO("UAV3: Publishing goal for potential mine %d at (%.2f, %.2f, %.2f)",
                 current_mine_index_ + 1,
                 goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        goal_pub_.publish(goal);
        //publishStatus();
    }

    // **修改**: 确保清理 potential_mines_
    void finishVerificationProcess() {
        publishVerifiedMines();
        notifyVerificationCompleted();
        // 清理已处理区域的数据
        int completed_region_id = last_verified_region_id-1; // 处理完的区域ID
        mines_by_region_.erase(completed_region_id); // 从存储中移除已处理的区域
        ROS_INFO("UAV3: Removed processed mines for region %d from storage", completed_region_id);

        // 清理已处理区域的数据
        potential_mines_.clear(); // **在这里清空潜在地雷列表**
        verified_mines_.clear();
        current_mine_index_ = -1;
        current_state_ = UAV3State::WAIT; // 设置为等待状态
        ROS_INFO("UAV3: Verification process finished for region %d. State set to COMPLETED.", last_verified_region_id-1);
    }

    // --- publishVerifiedMines, notifyVerificationCompleted, publishStatus, checkArrivalByPosition (保持不变) ---
    void publishVerifiedMines() {
        // ... (内容不变) ...
        if (verified_mines_.empty()) {
            ROS_INFO("UAV3: No mines verified as real in region %d.", last_verified_region_id);
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
        ROS_INFO("UAV3: Published %zu verified mines for region %d.", mines_msg.mines.size(), last_verified_region_id);
    }

    void notifyVerificationCompleted() {
        // ... (内容不变) ...
        mine_detection::UAVStatus status_msg;
        last_verified_region_id++;
        status_msg.region_id = last_verified_region_id;
        status_pub_.publish(status_msg);
        ROS_INFO("UAV3: Notified completion for region %d.", last_verified_region_id-1);
    }

   

    // 模拟视觉地雷检测：悬停5秒钟即可判定完成
    bool simulateVisualMineDetection() {
        static bool started = false;

        if (!started) {
            hover_start_time_ = ros::Time::now();
            started = true;
            ROS_INFO("UAV3: Start visual mine detection, hovering...");
            return false;
        }

        if ((ros::Time::now() - hover_start_time_).toSec() >=hover_time_) {
            started = false; // 重置以便下次调用
            ROS_INFO("UAV3: Visual mine detection finished (hovered 5s).");
            if(dis_(gen_)<verification_probability_){
                if (current_mine_index_ >= 0 && current_mine_index_ < potential_mines_.size()) {
                    verified_mines_.push_back(potential_mines_[current_mine_index_]);
                }
                ROS_INFO("UAV3: Mine %d VERIFIED as real.", current_mine_index_ + 1);
            }
            else{
                ROS_INFO("UAV3: Mine %d identified as FALSE POSITIVE.", current_mine_index_ + 1);
            }
            return true;
        }
        return false;
    }

    bool checkArrivalByPosition() {
        if (current_mine_index_ < 0 || current_mine_index_ >= potential_mines_.size()) {
            ROS_ERROR("UAV3: Invalid mine index %d for arrival check.", current_mine_index_);
            return false;
        }
        
        const auto& target_pos = potential_mines_[current_mine_index_].pose.position;
        double dx = current_position.x - target_pos.x;
        double dy = current_position.y - target_pos.y;
        double dz = current_position.z - target_pos.z;
        
        // 计算三维欧几里得距离平方
        double dist_sq = dx*dx + dy*dy + dz*dz;
        ROS_INFO("UAV3: Distance to mine %d: %.2f m (threshold: %.2f m)",
                 current_mine_index_ + 1, sqrt(dist_sq), arrival_threshold_);
        ROS_INFO("UAV3: Current pos: (%.2f, %.2f, %.2f), Target: (%.2f, %.2f, %.2f)",
                 current_position.x, current_position.y, current_position.z,
                 target_pos.x, target_pos.y, target_pos.z);
        if(dist_sq < (arrival_threshold_*arrival_threshold_)){
            waypoint_reached_ = false;
            return true;
            ROS_INFO("UAV3: Arrived at mine %d by position check.", current_mine_index_ +1);
        }
        ROS_INFO("UAV3: Waiting for waypoint status to confirm arrival at mine %d.", current_mine_index_ + 1);
        return false;
       
    }
};

// --- main 函数 (保持不变) ---
int main(int argc, char** argv) {
    // ... (内容不变) ...
    ros::init(argc, argv, "uav3_scan_planner_node");
    ros::NodeHandle nh("~");
    UAV3ScanPlanner planner(nh);
    ros::spin();
    return 0;
}