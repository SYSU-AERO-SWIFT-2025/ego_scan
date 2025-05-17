#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mine_detection/MineArray.h>
#include <mine_detection/UAVStatus.h>
#include <nav_msgs/Odometry.h>
#include <mine_detection/WaypointStatus.h>
#include <vector>
#include <map>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <random>
#include <nav_msgs/Path.h> 
#include <AStarPlanner.h>

// UAV4 state machine
enum class UAV4State { WAIT, COLLECTING, PLANNING, MOVING, COMPLETED };
// 方向枚举（1:前 2:左 3:右）
enum class Direction { FRONT = 1, LEFT = 2, RIGHT = 3 };

class UAV4PathPlanner {
public:
    UAV4PathPlanner(ros::NodeHandle& nh) : 
        nh_(nh),
        current_state_(UAV4State::WAIT),
        current_region_id_(0),
        waypoint_reached_(false),
        gen_(rd_()),
        dis_(0.0, 1.0),
        current_direction_(Direction::FRONT) // 默认朝向
    {
        // Load parameters
        nh_.param<double>("arrival_threshold", arrival_threshold_, 0.1);
        nh_.param<double>("region_width", region_width_, 4.0);  //区域参数
        nh_.param<double>("region_length", region_length_, 4.0);
        nh_.param<double>("region_center_x", region_center_x_, -10.0); //第一个区域的中心
        nh_.param<double>("region_center_y", region_center_y_, -9.0);
        nh_.param<double>("safe_distance", safe_distance_, 0.5); // 和地雷的安全距离
        nh_.param<double>("flight_height", flight_height_, 2.0); // Flight height for UAV4
        nh_.param<double>("astar_resolution", astar_resolution_, 0.2);  // 网格分辨率

        // 初始化A*规划器
        astar_planner_ = std::make_unique<AStarPlanner>(astar_resolution_, safe_distance_);

        // Initialize subscribers
        uav2_status_sub_ = nh_.subscribe("/uav2/scan_status", 10, &UAV4PathPlanner::uav2StatusCallback, this);
        uav3_status_sub_ = nh_.subscribe("/uav3/scan_status", 10, &UAV4PathPlanner::uav3StatusCallback, this);
        uav2_mines_sub_ = nh_.subscribe("/uav2/verified_mines", 10, &UAV4PathPlanner::uav2MinesCallback, this);
        uav3_mines_sub_ = nh_.subscribe("/uav3/verified_mines", 10, &UAV4PathPlanner::uav3MinesCallback, this);
        odom_sub_ = nh_.subscribe("/uav4/odom", 10, &UAV4PathPlanner::odomCallback, this);
        waypoint_status_sub_ = nh_.subscribe("/uav4/waypoint_status", 10, &UAV4PathPlanner::egoStatusCallback, this);

        // Initialize publishers
        status_pub_ = nh_.advertise<mine_detection::UAVStatus>("/uav4/scan_status", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav4/move_base_simple/goal", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/uav4/safe_path", 10);

        //初始化边界
        next_region_boundary_.x = region_center_x_-region_length_/2;
        next_region_boundary_.y = region_center_y_;
        next_region_boundary_.z = flight_height_;
        // Wait for subscribers to connect
        // ros::Rate poll_rate(10); //等待订阅，这里后面要改成人的接口
        // while (ros::ok() && goal_pub_.getNumSubscribers() == 0) {
        //     ROS_INFO("UAV4: Waiting for goal subscribers...");
        //     poll_rate.sleep();
        // }
        ROS_INFO("UAV4: Goal subscribers connected.");

        // Start state machine timer
        state_machine_timer_ = nh_.createTimer(ros::Duration(0.5), &UAV4PathPlanner::stateMachineCallback, this);

        ROS_INFO("UAV4 Path Planner Initialized. Current state: WAIT");
    }

private:
    //A* 相关成员
    std::unique_ptr<AStarPlanner> astar_planner_;
    double astar_resolution_;  // 网格分辨率
    double safe_distance_;
    // ROS members
    ros::NodeHandle nh_;
    ros::Subscriber uav2_status_sub_, uav3_status_sub_, uav2_mines_sub_, uav3_mines_sub_;
    ros::Subscriber odom_sub_, waypoint_status_sub_;
    ros::Publisher status_pub_, goal_pub_, path_pub_;
    ros::Timer state_machine_timer_;

    // State machine
    UAV4State current_state_;
    int current_region_id_; //初始化为0
    geometry_msgs::Point current_position_;
    bool waypoint_reached_;
    std::mutex ego_mutex_;

    // Mine data storage
    std::map<int, std::vector<geometry_msgs::Point>> verified_mines_by_region_;
    std::vector<geometry_msgs::PoseStamped> current_path_;
    size_t current_path_index_ = 1;

    // Parameters
    double arrival_threshold_;
    double region_width_;
    double region_length_;
    double region_center_x_;
    double region_center_y_;
    double flight_height_;

    // Random number generation for simulation 随机数生成器
    // 生成路径时添加微小随机偏移，避免完全机械化的路径
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_;

     // Member variables for UAV status tracking
    mine_detection::UAVStatus uav2_status_, uav3_status_;
    geometry_msgs::Point next_region_boundary_;

    //朝向信息
    Direction current_direction_;
    // 
    void stateMachineCallback(const ros::TimerEvent&) {
        state_machine_timer_.stop();
        switch(current_state_) {
            case UAV4State::WAIT:
                handleWaitState();
                break;
            case UAV4State::COLLECTING:
                handleCollectingState();
                break;
            case UAV4State::PLANNING:
                handlePlanningState();
                break;
            case UAV4State::MOVING:
                handleMovingState();
                break;
            case UAV4State::COMPLETED:
                ROS_INFO("UAV4: All regions completed.");
                break;
        }
        state_machine_timer_.start();
    }

    // State handlers
    void handleWaitState() {
        // 、等待uav2/3检查结束
        if (
            (uav2_status_.region_id > current_region_id_ && 
             uav3_status_.region_id > current_region_id_)) {
            
            ROS_INFO("UAV4: Starting to collect mines for region %d", current_region_id_+1);
            current_state_ = UAV4State::COLLECTING;
        } else {
            ROS_INFO("UAV4: Waiting for UAV2 (%d) and UAV3 (%d) to complete region %d", 
                    uav2_status_.region_id, uav3_status_.region_id, current_region_id_+1);
        }
    }

    //收集当前区域内的地雷点
    void handleCollectingState() {
        static bool goal_sent = false;  // 静态变量，保持状态
        // 检查当前是否接收到地雷数据
        auto it = verified_mines_by_region_.find(current_region_id_);
        if (it != verified_mines_by_region_.end() && !it->second.empty()) {
            ROS_INFO("UAV4: Collected %zu mines for region %d. Starting path planning.", 
                    it->second.size(), current_region_id_+1);
            if(current_region_id_ == 0){ //初始化要飞到探索区域的边界
                if(!goal_sent) {
                    moveToBoundary();
                    goal_sent = true;
                }
                //只有到达了边界才开始后续的规划
                if(checkWaypointReached(next_region_boundary_)){
                    goal_sent = false;
                    current_state_ = UAV4State::PLANNING;
                }
            }
            else{
                current_state_ = UAV4State::PLANNING;
            }
        } else {
            // map中存在region索引，但是对应区域内没有地雷
            ROS_INFO("UAV4: No mines detected in region %d. Moving to boundary.", current_region_id_ + 1);
            current_state_ = UAV4State::MOVING;
            next_region_boundary_ = calculateRegionBoundary(current_region_id_);
        }
    }

    void handlePlanningState() {
        // Generate safe path around mines
        if (generateSafePath(current_region_id_)) {
            ROS_INFO("UAV4: Generated safe path with %zu waypoints for region %d", 
                    current_path_.size(), current_region_id_+1);
            current_path_index_ = 1;
            current_state_ = UAV4State::MOVING;
        } else {
            ROS_ERROR("UAV4: Failed to generate safe path for region %d", current_region_id_+1);
            current_state_ = UAV4State::WAIT;
        }
    }

    void handleMovingState() { //移动到对面边界
        static bool goal_sent = false;  // 静态变量，保持状态
        static ros::Time last_waypoint_time = ros::Time(0);// 上次发送航点的时间
        static const double waypoint_timeout = 5.0; // 航点超时时间
        if (current_path_.empty()) {
            // 当前地区没有路径生成（没有地雷，直接飞到区域边界）
            if (checkWaypointReached(next_region_boundary_)) {
                ROS_INFO("UAV4: Arrived at boundary of region %d", current_region_id_+1);
                finishRegion();
            } else if (!goal_sent) {  // 只在未发送时发送
            moveToBoundary();
            goal_sent = true;
            last_waypoint_time = ros::Time::now(); // 更新上次发送航点的时间
         }
         else if((ros::Time::now() - last_waypoint_time).toSec() > waypoint_timeout) {
            // 超过超时时间，重新发送航点
            moveToBoundary();
            ROS_INFO("UAV4: Resending waypoint to boundary of region %d", current_region_id_+1);
            last_waypoint_time = ros::Time::now(); // 更新上次发送航点的时间
        } 
    }
        else {
            // 跟随生成的路径
            // 修改为：
            if (checkWaypointReached(current_path_[current_path_index_].pose.position)) {
                // 已到达当前航点，移动到下一个航点
                current_path_index_++;
                if (current_path_index_ < current_path_.size()) {
                    publishCurrentWaypoint();
                }
                else{
                    // 到达路径末尾，完成当前区域
                    ROS_INFO("UAV4: Reached the end of the path for region %d", current_region_id_+1);
                    finishRegion();
                }
            } else{
                // 未到达当前航点，继续发送当前航点
                publishCurrentWaypoint();

            }
        }
    }

    // Callbacks
    //更新收到的uav2状态
    void uav2StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
        uav2_status_ = *msg;
        ROS_INFO("UAV4: Received status from UAV2 - region_id: %d. Current State: %d, Current region id: %d",
                 msg->region_id, static_cast<int>(current_state_), current_region_id_+1);
    }

    //更新收到的uav3状态
    void uav3StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
        uav3_status_ = *msg;
        ROS_INFO("UAV4: Received status from UAV3 - region_id: %d. Current State: %d, Current region id: %d",
                 msg->region_id, static_cast<int>(current_state_), current_region_id_+1);
    }

    void uav2MinesCallback(const mine_detection::MineArray::ConstPtr& msg) {
        processMines(msg, 2);
    }

    void uav3MinesCallback(const mine_detection::MineArray::ConstPtr& msg) {
        processMines(msg, 3);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        //current_position_ = msg->pose.pose.position;
    }

    void egoStatusCallback(const mine_detection::WaypointStatus::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        if (msg->isReached) {
            waypoint_reached_ = true;
            current_position_ = msg->position;
            ROS_INFO("UAV4: Waypoint reached confirmed by EGO planner");
        }
    }

    // Helper functions
    // 处理接收到的地雷点
    void processMines(const mine_detection::MineArray::ConstPtr& msg, int uav_id) {
        //确定正确的区域ID
        int region_id = (uav_id == 2) ? uav2_status_.region_id-1 : uav3_status_.region_id-1;
        if (region_id < 0) region_id = 0;
        
        ROS_INFO("UAV4: Received %zu mines from UAV%d for estimated region %d", 
                msg->mines.size(), uav_id, region_id);
        
        auto& region_mines = verified_mines_by_region_[region_id];
        
        // 将地雷坐标检查后加入总地雷中，用map存储
        for (const auto& mine : msg->mines) { //检查重复点
            bool duplicate = false;
            for (const auto& existing_mine : region_mines) {
                double dx = existing_mine.x - mine.position.x;
                double dy = existing_mine.y - mine.position.y;
                if (dx*dx + dy*dy < 0.01) { // 10cm threshold
                    duplicate = true;
                    break;
                }
            }
            
            if (!duplicate) {
                region_mines.push_back(mine.position);
                ROS_INFO("UAV4: Added mine at (%.2f, %.2f) to region %d", 
                        mine.position.x, mine.position.y, region_id);
            }
        }
    }


    // 简单的区域避障算法
    bool generateSafePath_Simple(int region_id) {
        auto it = verified_mines_by_region_.find(region_id);
        if (it == verified_mines_by_region_.end() || it->second.empty()) {
            return false;
        }

        // Get region boundaries
        geometry_msgs::Point start_point, end_point;
        start_point = current_position_;
        end_point = start_point; 
        end_point.x +=  region_width_; //start和end水平垂直对应
        
        current_path_.clear();
        
        // 
        geometry_msgs::PoseStamped wp;
        wp.header.stamp = ros::Time::now();
        wp.header.frame_id = "map";
        wp.pose.position = start_point;
        wp.pose.position.z = flight_height_;
        wp.pose.orientation.w = 1.0;
        current_path_.push_back(wp);
        
        // 生成路径中间点 
        double step = 0.2; // 0.2 steps
        double dx = end_point.x - start_point.x;
        double dy = end_point.y - start_point.y;
        double distance = sqrt(dx*dx + dy*dy); //计算起始点之间的距离
        int steps = static_cast<int>(distance / step);
        
        for (int i = 1; i <= steps; i++) {
            double ratio = static_cast<double>(i) / steps; //按比例线性插值 
            geometry_msgs::PoseStamped intermediate;
            intermediate.header = wp.header;
            intermediate.pose.position.x = start_point.x + dx * ratio;
            intermediate.pose.position.y = start_point.y + dy * ratio;
            intermediate.pose.position.z = flight_height_;
            intermediate.pose.orientation.w = 1.0;
            
            //遍历当前区域地雷看是否碰撞
            bool safe = true;
            for (const auto& mine : it->second) {
                double mdx = intermediate.pose.position.x - mine.x;
                double mdy = intermediate.pose.position.y - mine.y;
                if (sqrt(mdx*mdx + mdy*mdy) < safe_distance_) {
                    safe = false;
                    break;
                }
            }
            
            if (safe) {
                current_path_.push_back(intermediate);
            } else { // 不安全就尝试绕开
                generateDetour_Simple(intermediate.pose.position, it->second);
            }
        }
        
        // 添加终点
        wp.pose.position = end_point;
        current_path_.push_back(wp);
        
        // 发布当前路径
        publishPath();
        
        return !current_path_.empty();
    }

    // 简单绕障路径生成
    void generateDetour_Simple(const geometry_msgs::Point& unsafe_point, 
                       const std::vector<geometry_msgs::Point>& mines) {
        // 寻找最近的地雷点
        geometry_msgs::Point closest_mine;
        double min_dist = std::numeric_limits<double>::max();

        for (const auto& mine : mines) {
            double dx = unsafe_point.x - mine.x;
            double dy = unsafe_point.y - mine.y;
            double dist = dx*dx + dy*dy;
            if (dist < min_dist) {
                min_dist = dist;
                closest_mine = mine;
            }
        }
        
        // 生成半圆绕开障碍物路径
        const int num_points = 5;
        for (int i = 1; i <= num_points; i++) {
            double angle = M_PI * i / (num_points + 1);
            double radius = safe_distance_ * 1.2; // Slightly larger than safe distance
            
            geometry_msgs::PoseStamped detour_wp;
            detour_wp.header.stamp = ros::Time::now();
            detour_wp.header.frame_id = "map";
            detour_wp.pose.position.x = closest_mine.x + radius * cos(angle);
            detour_wp.pose.position.y = closest_mine.y + radius * sin(angle);
            detour_wp.pose.position.z = flight_height_;
            detour_wp.pose.orientation.w = 1.0;
            
            current_path_.push_back(detour_wp);
        }
    }

    // A* 版generateSafePath函数
    bool generateSafePath(int region_id) {
        auto it = verified_mines_by_region_.find(region_id);
        if (it == verified_mines_by_region_.end()) {
            return false;
        }

        // 获取起点和终点
        geometry_msgs::Point start_point = current_position_;
        geometry_msgs::Point end_point = start_point;
        end_point.x += region_width_;  // 水平移动
        ROS_INFO("UAV4: Start point: (%.2f, %.2f, %.2f), End point: (%.2f, %.2f, %.2f)", 
                start_point.x, start_point.y, start_point.z,
                end_point.x, end_point.y, end_point.z);
        // 使用A*规划路径
        astar_planner_->initRegionParm(region_width_,region_length_,region_center_x_,region_center_y_,region_id);
        nav_msgs::Path path = astar_planner_->planPathToBoundary(start_point, end_point, 0,it->second ,flight_height_);
    
        if (path.poses.empty()) {
            ROS_ERROR("UAV4: A* failed to find path for region %d", region_id);
            return false;
        }
    
        // 转换为当前路径格式
        current_path_.clear();
        for (const auto& pose : path.poses) {
            current_path_.push_back(pose);
        }
    
         // 发布路径
        publishPath();
    
        ROS_INFO("UAV4: Generated A* path with %zu waypoints for region %d", 
            current_path_.size(), region_id);
        return true;
    }
    
    
    // 计算区域边界，默认在x方向上运动
    geometry_msgs::Point calculateRegionBoundary(int region_id) {
        geometry_msgs::Point next_region_boundary;
        // 根据方向设置下一个区域边界
        if(current_direction_ == Direction::FRONT) {
            next_region_boundary.x = region_center_x_ + region_length_ / 2;
            next_region_boundary.y = region_center_y_;
        } else if (current_direction_ == Direction::LEFT) {
            next_region_boundary.x = region_center_x_;
            next_region_boundary.y = region_center_y_ - region_width_ / 2;
        } else if (current_direction_ == Direction::RIGHT) {
            next_region_boundary.x = region_center_x_;
            next_region_boundary.y = region_center_y_ + region_width_ / 2;
        }
        next_region_boundary.z = flight_height_;
        
        ROS_INFO("UAV4: Calculated boundary for region %d at (%.2f, %.2f, %.2f)", 
                region_id+1, next_region_boundary.x, next_region_boundary.y, next_region_boundary.z);
        return next_region_boundary;
    }

    //
    void moveToBoundary() {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position = next_region_boundary_;
        goal.pose.orientation.w = 1.0;
        
        ROS_INFO("UAV4: Moving to boundary at (%.2f, %.2f, %.2f)", 
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        
        goal_pub_.publish(goal);
    }

    void publishCurrentWaypoint() {
        if (current_path_index_ < current_path_.size()) {
            goal_pub_.publish(current_path_[current_path_index_]);
            ROS_INFO("UAV4: Publishing waypoint %zu at (%.2f, %.2f, %.2f)", 
                    current_path_index_ ,
                    current_path_[current_path_index_].pose.position.x,
                    current_path_[current_path_index_].pose.position.y,
                    current_path_[current_path_index_].pose.position.z);
        }
    }

    void publishPath() {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        path_msg.poses = current_path_;
        
        path_pub_.publish(path_msg);
        ROS_INFO("UAV4: Published path with %zu waypoints", current_path_.size());
    }

    bool checkWaypointReached(const geometry_msgs::Point& goal) {
        std::lock_guard<std::mutex> lock(ego_mutex_);
        double dx = current_position_.x - goal.x;
        double dy = current_position_.y - goal.y;
        double dz = current_position_.z - goal.z;
        double dist_sq = dx*dx + dy*dy + dz*dz;
        ROS_INFO("UAV4: Current position: (%.2f, %.2f, %.2f)", 
                current_position_.x, current_position_.y, current_position_.z);
        ROS_INFO("UAV4: Checking waypoint reached. Distance squared: %.2f", dist_sq);

        if (dist_sq < (arrival_threshold_ * arrival_threshold_)&& waypoint_reached_) {
            waypoint_reached_ = false; // Reset for next waypoint
            ROS_INFO("UAV4: Reached waypoint at (%.2f, %.2f, %.2f)", 
                    goal.x, goal.y, goal.z);
            return true;
        }
        return false;
    }

    void finishRegion() {
        // 重置相关数据，发布状态
        current_region_id_++;
        current_path_.clear();
        current_path_index_ = 1;
        
        // Notify completion
        mine_detection::UAVStatus status_msg;
        status_msg.region_id = current_region_id_;
        status_pub_.publish(status_msg);
    
        
        ROS_INFO("UAV4: Completed region %d. Notifying and waiting for next region.", 
                current_region_id_);
        // 更新区域中心点
        updateRegionCenter();
        current_state_ = UAV4State::WAIT;
    }

    //更新当前区域中心点
    void updateRegionCenter() {
        if (current_direction_ == Direction::FRONT) {
            region_center_x_ += region_length_;
        } else if (current_direction_ == Direction::LEFT) {
            region_center_y_ -= region_width_;
        } else if (current_direction_ == Direction::RIGHT) {
            region_center_y_ += region_width_;
        }
        ROS_INFO("UAV4: Updated region center to (%.2f, %.2f, %.2f)", 
                 region_center_x_, region_center_y_, flight_height_);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav4_path_planner");
    ros::NodeHandle nh("~");
    UAV4PathPlanner planner(nh);
    ros::spin();
    return 0;
}