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
enum class UAV2State { WAIT, MOVING,VERIFYING,MOVING_TO_BOUNDARY };
// 方向枚举（1:前 2:左 3:右）
enum class Direction { FRONT = 1, LEFT = 2, RIGHT = 3 };

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
        current_region_id_(0),
        current_direction_(Direction::FRONT) // 默认朝向
    {
        // 加载参数
        nh_.param<double>("hover_time", hover_time_, 5.0);
        nh_.param<double>("arrival_threshold", arrival_threshold_, 0.1); // 检测到达阈值
        nh_.param<double>("verification_probability", verification_probability_, 0.8);// 地雷验证概率

        // 区域尺寸参数初始化
        nh_.param<double>("region_width", region_width_, 4.0);     // 默认值4.0
        nh_.param<double>("region_length", region_length_, 4.0);   // 默认值4.0 和uav1统一
        region_center_.x = -8.0; 
        region_center_.y = -9.0;
        region_center_.z = 1.0; 

        // 初始化订阅者 (确保话题名称正确)
        uav1_status_sub_ = nh_.subscribe("/uav1/scan_status", 10, &UAV2ScanPlanner::uav1StatusCallback, this);
        uav4_status_sub_ = nh_.subscribe("/uav4/scan_status", 10, &UAV2ScanPlanner::uav4StatusCallback, this);
        detected_mines_sub_ = nh_.subscribe("/uav1/detected_mines", 10, &UAV2ScanPlanner::detectedMinesCallback, this);
        odom_sub_ = nh_.subscribe("/uav2/odom", 10, &UAV2ScanPlanner::odomCallback, this); 
        waypoint_status_sub_ = nh_.subscribe("/uav2/waypoint_status", 10, &UAV2ScanPlanner::egoStatusCallback, this); 

        // 初始化发布者
        status_pub_ = nh_.advertise<mine_detection::UAVStatus>("/uav2/scan_status", 10);
        verified_mines_pub_ = nh_.advertise<mine_detection::MineArray>("/uav2/verified_mines", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav2/move_base_simple/goal", 10); 

        // 等待订阅者连接 (特别是 EGO-Planner 的目标点订阅者)
        ros::Rate poll_rate(10); // 10 Hz
        while (ros::ok() && goal_pub_.getNumSubscribers() == 0) {
            ROS_INFO("UAV2: Waiting for goal subscribers on %s...", goal_pub_.getTopic().c_str());
            poll_rate.sleep();
        }
        ROS_INFO("UAV2: Goal subscribers connected.");
        // 启动状态机定时器
        state_machine_timer_ = nh_.createTimer(ros::Duration(0.5), &UAV2ScanPlanner::stateMachineCallback, this);

        ROS_INFO("UAV2 Scan Planner Initialized. Current state: WAIT");
    }

private:
    // ROS 相关成员变量
    ros::NodeHandle nh_;
    ros::Subscriber uav1_status_sub_, detected_mines_sub_, odom_sub_, waypoint_status_sub_, uav4_status_sub_;
    ros::Publisher status_pub_, verified_mines_pub_, goal_pub_;
    ros::Timer state_machine_timer_;
    
   
    UAV2State current_state_; //当前状态机状态

    // 地雷点相关成员变量
    std::vector<geometry_msgs::PoseStamped> potential_mines_;//uav1发布的地雷点
    std::vector<geometry_msgs::PoseStamped> verified_mines_;//验证为真的地雷点
    std::map<int, std::vector<geometry_msgs::PoseStamped>> mines_by_region_;   // 按区域ID存储地雷信息，以便保存地雷点不会丢失
    int current_mine_index_;//当前正在验证的地雷索引
    int current_region_id_; // 当前地雷点应归属的区域

    //无人机位置信息
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
    
    //朝向信息
    Direction current_direction_; // 当前朝向（前、左、右）

    // 区域尺寸相关参数 
    double region_width_;    // 区域宽度
    double region_length_;   // 区域长度
    geometry_msgs::Point region_center_; // 区域中心点

    // 下一个区域边界位置
    geometry_msgs::Point next_region_boundary_;

    // 协同参数
    mine_detection::UAVStatus uav1_status_;
    mine_detection::UAVStatus uav4_status_;

    bool boundary_goal_published_ = false;
    // --- 状态机核心逻辑 ---
    void stateMachineCallback(const ros::TimerEvent&) {
        state_machine_timer_.stop(); // 停止定时器，避免重复调用
        switch(current_state_) {
            case UAV2State::WAIT://等待扫描
                handleWaitState();
                break;
            case UAV2State::MOVING://移动到地雷点
                handleMovingState();
                break;
            case UAV2State::VERIFYING://验证地雷点
                handleVerifyingState();
                break;
            case UAV2State::MOVING_TO_BOUNDARY://如果上一个区域没有地雷点，就飞到下一个区域的边界
                handleMovingToBoundaryState();
                break;
        }
        state_machine_timer_.start();//重新启动定时器
    }

    /*状态处理函数*/
    // --- 等待状态处理函数 ---
    void handleWaitState(){
        //是否需要协同等待
        if(requireCooperation()){
            ROS_INFO("UAV2: Waiting for UAV4 to finish scanning region %d.", current_region_id_-1);
            return;
        } 
        if(uav1_status_.region_id ==current_region_id_+1){
            auto it = mines_by_region_.find(current_region_id_);
            if(it == mines_by_region_.end()){
                // 如果没有地雷点，飞到下一个区域的边界
                ROS_INFO("UAV2: No mines detected in region %d. Moving to boundary.", current_region_id_+1);
                current_state_ = UAV2State::MOVING_TO_BOUNDARY;
                return;
            }
            else if (it != mines_by_region_.end() && !it->second.empty()) {
                // 如果有地雷点，开始验证
                //将当前区域的地雷点赋值给potential_mines_
                //后续操作都用potential_mines_来进行
                potential_mines_ = it->second;
                verified_mines_.clear(); // 清空已验证的地雷列表
                optimizeMinePath(); // 优化地雷点顺序
                current_mine_index_ = 0; // 重置地雷索引
                current_state_ = UAV2State::MOVING;
            }
            else{
                ROS_WARN("UAV2: No new region detected or no mines to verify.");
            }
        }

        }
    
    // --- 移动到地雷点，状态处理函数 ---
    void handleMovingState() {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        ROS_INFO("UAV2: handleMovingState, mines size=%zu, current_mine_index=%d", potential_mines_.size(), current_mine_index_);
        //当前地雷点
        auto& current_mine = potential_mines_[current_mine_index_].pose.position;
        // 判断是否到达目标地雷点
        if (checkArrivalByPosition(current_mine)) {
            ROS_INFO("UAV2: Arrived at mine %d, start verifying...", current_mine_index_ + 1);
            current_state_ = UAV2State::VERIFYING;
        } else {
            //初始化发送地雷点
            ROS_INFO("UAV2: Moving to mine %d...", current_mine_index_ + 1);
            moveToCurrentMine();
        }
      
}
    // --- 验证地雷状态处理函数 ---
    void handleVerifyingState() {
        //视觉验证地雷点是否为真
        if (simulateVisualMineDetection()) {
            current_mine_index_++;
            // 检查是否还有下一个地雷点,还有的话就将状态改为MOVING，然后重复移动验证地雷的过程
            if (current_mine_index_ < potential_mines_.size()) {
                current_state_ = UAV2State::MOVING;
                ROS_INFO("UAV2: Moving to next potential mine %d.", current_mine_index_ + 1);
            } 
            // 如果验证完当前区域地雷点之后，发布验证结果        
            else {
                ROS_INFO("UAV2: Finished verifying all %zu potential mines for region %d.",
                         potential_mines_.size(), current_region_id_+1);
                finishVerificationProcess();
            }
        }
    }
    
    // 处理飞向边界状态的函数
    void handleMovingToBoundaryState() {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        // 移动到下一个区域的边界位置
        moveToBoundary();

        // 检查是否到达边界位置
        if (checkArrivalByPosition(next_region_boundary_)) {
            ROS_INFO("UAV2: Arrived at boundary position for region %d.", current_region_id_ + 2);
            // 更新当前区域ID
            current_region_id_++;
            //发布当前状态
            notifyVerificationCompleted();
            //更新区域中心点
            updateRegionCenter();
            current_state_ = UAV2State::WAIT; // 返回等待状态
        } else {
            // 未到达则持续等待
            ROS_INFO_THROTTLE(2, "UAV2: Moving to boundary position for region %d...", current_region_id_ + 2);
        }
    }

    // --- uav1回调函数  ---
    void uav1StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
        ROS_INFO("UAV2: Received status from UAV1 - region_id: %d. Current State: %d, Current region id: %d",
                 msg->region_id, static_cast<int>(current_state_), current_region_id_+1);
        // 更新uav1的状态
        uav1_status_ = *msg;
        
    }

    // --- uav4回调函数 ---
    void uav4StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
        ROS_INFO("UAV2: Received status from UAV4 - region_id: %d. Current State: %d, Current region id: %d",
                 msg->region_id, static_cast<int>(current_state_), current_region_id_+1);
        // 更新收到的uav4状态
        uav4_status_ = *msg;
    }

     // --- odom回调函数 ---
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        //current_position = msg->pose.pose.position;
    }
    // --- ego回调函数 ---
    void egoStatusCallback(const mine_detection::WaypointStatus::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        if (msg->isReached) {
            waypoint_reached_ = true;
            current_position=msg->position;
            ROS_INFO("UAV2: EGO reported arrival at waypoint.");
        }
    }
    // --- 地雷回调函数 ---
    void detectedMinesCallback(const mine_detection::MineArray::ConstPtr& msg) {
        // 接收uav1当前发布的地雷
        //将其放在对应的区域ID中 
        int target_region_id = current_region_id_;
        ROS_INFO("UAV2: Received %zu potential mines from UAV1, estimating for region %d.", 
                 msg->mines.size(), target_region_id+1);
    
        // 获取或创建目标区域的地雷数组
        auto& region_mines = mines_by_region_[target_region_id];
        size_t previous_mine_count = region_mines.size();// 地雷点数量
        size_t duplicates_count = 0;// 重复地雷点数量
        
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
                            mine.position.x, mine.position.y, mine.position.z, target_region_id+1);
                    break;
                }
            }
            
            // 只添加非重复的地雷点
            if (!is_duplicate) {
                ROS_INFO("UAV2: Adding Potential Mine %zu at (%.2f, %.2f, %.2f) to region %d", 
                        region_mines.size() + 1, mine.position.x, mine.position.y, mine.position.z,
                        target_region_id+1);
                
                geometry_msgs::PoseStamped mine_pose;
                mine_pose.header = msg->header;
                mine_pose.header.stamp = ros::Time(0);
                mine_pose.pose.position = mine.position;
                mine_pose.pose.position.z = 1.0; // UAV2验证高度
                mine_pose.pose.orientation.w = 1.0;
                region_mines.push_back(mine_pose);
            }
        }
        
        // 添加新的日志
        if (duplicates_count > 0) {
            ROS_WARN("UAV2: Filtered out %zu duplicate mines for region %d", 
                     duplicates_count, target_region_id+1);
        }
        
        ROS_INFO("UAV2: Region %d now has %zu potential mines", 
                 target_region_id+1, region_mines.size());
    }


    /*地雷点访问路径优化*/
    //优化地雷点访问顺序的函数
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

    // 移动到下一个区域边界
    void moveToBoundary() {
        geometry_msgs::PoseStamped goal;
        //根据方向设置下一个区域边界
        if (current_direction_ == Direction::FRONT) {
            next_region_boundary_.x = region_center_.x + region_length_ / 2;
            next_region_boundary_.y = region_center_.y;
        } else if (current_direction_ == Direction::LEFT) {
            next_region_boundary_.x = region_center_.x;
            next_region_boundary_.y = region_center_.y - region_width_ / 2;
        } else if (current_direction_ == Direction::RIGHT) {
            next_region_boundary_.x = region_center_.x;
            next_region_boundary_.y = region_center_.y + region_width_ / 2;
        }
        next_region_boundary_.z = region_center_.z; // 保持高度不变
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position = next_region_boundary_;
        goal.pose.orientation.w = 1.0;// 默认朝向
        
        ROS_INFO("UAV2: Moving to boundary position for next region at (%.2f, %.2f, %.2f)",
                next_region_boundary_.x, next_region_boundary_.y, next_region_boundary_.z);
        
        // 发布目标点
        goal_pub_.publish(goal);
    }

    // --- 飞往地雷点 ---
    void moveToCurrentMine() {
        if (current_mine_index_ < 0 || current_mine_index_ >= potential_mines_.size()) {
            ROS_ERROR("UAV2: Invalid mine index %d requested for movement.", current_mine_index_);
            return;
        }
        geometry_msgs::PoseStamped goal = potential_mines_[current_mine_index_];
        if(current_position==goal.pose.position){
            ROS_INFO("UAV2: Already at mine %d, no need to move.", current_mine_index_ + 1);
            return;
        }
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        ROS_INFO("UAV2: Publishing goal for potential mine %d at (%.2f, %.2f, %.2f)",
                 current_mine_index_ + 1,
                 goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        goal_pub_.publish(goal);
    }

    /*结束验证过程*/
    void finishVerificationProcess() {
        //记录当前区域ID
        current_region_id_++;
        //重置地雷索引
        current_mine_index_ = 0;
        // 清理已处理区域的数据
        int completed_region_id = current_region_id_; // 处理完的区域ID
        mines_by_region_.erase(completed_region_id); // 从地雷列表存储中移除已处理的区域
        ROS_INFO("UAV2: Removed processed mines for region %d from storage", completed_region_id);
        
        notifyVerificationCompleted(); // 发布当前状态，通知uav4已经验证的区域的地雷
        publishVerifiedMines();// 再发布验证后的地雷点
        

        // 清理已处理区域的数据
        potential_mines_.clear(); // **在这里清空潜在地雷列表**
        verified_mines_.clear();
        ROS_INFO("UAV2: Verification process finished for region %d. State set to WAIT.", current_region_id_);
         //更新当前区域的中心点
         updateRegionCenter();
        current_state_ = UAV2State::WAIT; // 设置为等待状态
    }

    // 发布验证后的地雷点
    void publishVerifiedMines() {
        if (verified_mines_.empty()) {
            ROS_INFO("UAV2: No mines verified as real in region %d.", current_region_id_);
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
        ROS_INFO("UAV2: Published %zu verified mines for region %d.", mines_msg.mines.size(), current_region_id_);
    }

    // 发布当前状态，通知uav4已经验证的区域的地雷
    void notifyVerificationCompleted() {
        mine_detection::UAVStatus status_msg;
        status_msg.region_id = current_region_id_;
        status_pub_.publish(status_msg);
        ROS_INFO("UAV2: Notified completion for region %d.", current_region_id_);
    }

   

    // 模拟视觉地雷检测：悬停5秒钟即可判定完成
    bool simulateVisualMineDetection() {
        static bool started = false;

        if (!started) {
            hover_start_time_ = ros::Time::now();
            started = true;
            ROS_INFO("UAV2: Start visual mine detection, hovering...");
            return false;
        }

        if ((ros::Time::now() - hover_start_time_).toSec() >=hover_time_) {
            started = false; // 重置以便下次调用
            ROS_INFO("UAV2: Visual mine detection finished (hovered 5s).");
            if(dis_(gen_)<verification_probability_){
                if (current_mine_index_ >= 0 && current_mine_index_ < potential_mines_.size()) {
                    verified_mines_.push_back(potential_mines_[current_mine_index_]);
                }
                ROS_INFO("UAV2: Mine %d VERIFIED as real.", current_mine_index_ + 1);
            }
            else{
                ROS_INFO("UAV2: Mine %d identified as FALSE POSITIVE.", current_mine_index_ + 1);
            }
            return true;
        }
        return false;
    }    


    // 检查到达目标地雷点
    bool checkArrivalByPosition(geometry_msgs::Point goal){
        double dx = current_position.x - goal.x;
        double dy = current_position.y - goal.y;
        double dz = current_position.z - goal.z;
        double dist_sq = dx*dx + dy*dy + dz*dz;
        
        ROS_INFO("UAV2: Current position (%.2f, %.2f, %.2f), Goal position (%.2f, %.2f, %.2f), Distance squared: %.2f",
                 current_position.x, current_position.y, current_position.z,
                 goal.x, goal.y, goal.z, dist_sq);
        // 检查是否到达目标点
        if (dist_sq < (arrival_threshold_ * arrival_threshold_) && waypoint_reached_) {
            ROS_INFO("UAV2: Arrived at target position (%.2f, %.2f, %.2f)", goal.x, goal.y, goal.z);
            waypoint_reached_ = false; // 重置到达标志
            return true;
        }
        return false;
    }

    //更新区域中心点
    void updateRegionCenter() {
        if (current_direction_ == Direction::FRONT) {
            region_center_.x += region_length_;
        } else if (current_direction_ == Direction::LEFT) {
            region_center_.y -= region_width_;
        } else if (current_direction_ == Direction::RIGHT) {
            region_center_.y += region_width_;
        }
        ROS_INFO("UAV2: Updated region center to (%.2f, %.2f, %.2f)", 
                 region_center_.x, region_center_.y, region_center_.z);
    }

    //协同判断函数
    bool requireCooperation(){
        //检查当前uav4区域ID
        ROS_INFO("uav4_status_.region_id=%d",uav4_status_.region_id);
        //检查是否需要协同，1需要，0不需要
        return (current_region_id_ >uav4_status_.region_id+1);
    }


};


// --- main 函数 ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "uav2_scan_planner_node");
    ros::NodeHandle nh("~");
    UAV2ScanPlanner planner(nh);
    ros::spin();    
    return 0;
}