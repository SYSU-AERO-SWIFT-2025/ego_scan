
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mine_detection/MineArray.h>
#include <mine_detection/UAVStatus.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <mine_detection/HumanPose.h>
#include <mine_detection/WaypointStatus.h>
// 状态机枚举
enum class UAVState { INIT,SCANNING, WAITING, TRANSITION, COMPLETED };
// 方向枚举（1:前 2:左 3:右）
enum class Direction { FRONT = 1, LEFT = 2, RIGHT = 3 };

class ScanPlanner{
    public:
        ScanPlanner():nh_("~"),current_state_(UAVState::INIT), current_direction_(Direction::FRONT) {
            //参数初始化 //需要改回正常的参数初始化
            nh_.param("scan_width", scan_width_, 4.0);
            nh_.param("scan_length", scan_length_, 4.0);
            nh_.param("lane_spacing", lane_spacing_, 2.0);
            nh_.param("forward_offset", foward_offset_, 2.5);
            region_center_.x = -10.0;
            region_center_.y = -9.0;
            region_center_.z = 2.5; 
            // can_width_ = 4;
            // scan_length_ = 4;
            // lane_spacing_ = 1;
                // 调试输出
        // 话题订阅与发布
        //human_pose_sub_ = nh_.subscribe("/human/pose", 1, &ScanPlanner::humanPoseCallback, this);
        uav2_status_sub_ = nh_.subscribe("/uav2/scan_status", 10, &ScanPlanner::uav2StatusCallback, this);
        uav3_status_sub_ = nh_.subscribe("/uav3/scan_status", 10, &ScanPlanner::uav3StatusCallback, this);
        waypoint_status_sub_ = nh_.subscribe("/uav1/waypoint_status", 10,  &ScanPlanner::egoStatusCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav1/move_base_simple/goal", 1);//
        //等待订阅方初始化
        if (goal_pub_.getNumSubscribers() == 0) { 
            ROS_INFO("Waiting for subscribers...");
            ros::Time start = ros::Time::now();
            while ((ros::Time::now() - start).toSec() < 3.0 && goal_pub_.getNumSubscribers() == 0) {
                ros::Duration(0.1).sleep();
            }
        }
        mines_pub_ = nh_.advertise<mine_detection::MineArray>("/uav1/detected_mines", 10);
        notify_pub_ = nh_.advertise<mine_detection::UAVStatus>("/uav1/scan_status", 10);
        transitionToNextRegion();
            // 启动状态机
         scan_timer_ = nh_.createTimer(ros::Duration(0.5), &ScanPlanner::stateMachineCallback, this);

        }



        //模拟地雷扫描
        void simulateMineDetection(const geometry_msgs::PoseStamped& wp) {
            
            if (rand() % 10 < 8) {  // 30%概率模拟检测到地雷
                mine_detection::Mine mine;
                mine.position.x = wp.pose.position.x + (rand() % 3 - 1.5);
                mine.position.y = wp.pose.position.y + (rand() % 3 - 1.5);
                mine.position.z = 0;
                
                current_region_mines_.push_back(mine);
            }
        }

    
        private:
        ros::NodeHandle nh_;
        //人位置，uav2/3的状态订阅topic（//？考虑把人位置的topic改造成人命令topic）
        ros::Subscriber human_pose_sub_,uav2_status_sub_, uav3_status_sub_, waypoint_status_sub_;
        ros::Publisher mines_pub_, goal_pub_, notify_pub_;
        ros::Timer scan_timer_;
        

        bool current_waypoint_reached_ = false; //当前预计到达点的状态
        geometry_msgs::Point current_position; //当前位置
        std::mutex ego_mutex_; //保护current_waypoint_reached_的锁
        std::vector<geometry_msgs::PoseStamped> waypoints_;
        size_t current_waypoint_index_ = 0;
        int scan_direction_;
        std::vector<mine_detection::Mine> current_region_mines_; //地雷数组

        //状态信息
        UAVState current_state_;
        //朝向信息
        Direction current_direction_;
        //协同参数
        mine_detection::UAVStatus uav2_status_;
        mine_detection::UAVStatus uav3_status_;
        //扫描区域参数
        int current_region_id = 0; //？待考虑初始值为-1还是0
        geometry_msgs::Point region_center_; //扫描中心
        double foward_offset_; //暂时没用
        double scan_width_, scan_length_, lane_spacing_;

        //后期可以修改成人的命令回调函数
        void humanPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
            // 更新人员位置，重置扫描区域中心
            region_center_ = msg->pose.position;
            initializeZigzagWaypoints();  // 重新生成路径点
            current_waypoint_index_ = 0;  // 从第一个点开始
        }

        //uav2完成状态回调函数 
        void uav2StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
            uav2_status_ = *msg; // 拷贝完整状态
            ROS_INFO("UAV2 Status Update - Region: %d", 
                msg->region_id);
        }

        //uav3完成状态回调函数
        void uav3StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
            uav3_status_ = *msg; // 拷贝完整状态
            ROS_DEBUG("UAV2 Status Update - Region: %d", 
                msg->region_id);
        }

        // 状态机
        void stateMachineCallback(const ros::TimerEvent&) {
            ROS_WARN("Timer callback triggered");
            scan_timer_.stop(); //回调开始停止计时器
            switch(current_state_) {
              case UAVState::SCANNING: //扫描状态
                handleScanning();
                break;
              case UAVState::WAITING: //等待uav2/3扫描完成
                handleWaiting();
                break;
              case UAVState::TRANSITION: //区域切换
                handleTransition();
                break;
              case UAVState::COMPLETED:
                ROS_INFO("All tasks completed.");
                break;
            }
            scan_timer_.start();//激活定时器
          }
        
        /*状态处理函数*/
        // 扫描状态处理函数：生成扫描点发布
        void handleTransition() {
            ROS_WARN("Status Transition");
            if (checkWaypointReached(region_center_)) {
              initializeZigzagWaypoints();
              current_state_ = UAVState::SCANNING;
              ROS_WARN("Started scanning region %d", current_region_id+1);
            }
        }

        void handleScanning() {
            ROS_WARN("Status Scanning");
            if (current_waypoint_index_ < waypoints_.size()) {
              publishCurrentWaypoint();
            } else {
                //判断是否需要协同，不需要就状态wait，否则transition to next poiont
              if(checkWaypointReached(waypoints_.back().pose.position)){ //到达该区域最后一个点
                ROS_WARN("Finish scanning region %d", current_region_id+1);
                finishCurrentRegion();
              }else{
                //什么都不干，等待直到到达该区域最后一个点
              }
            }
          }        

          void handleWaiting() {
            static ros::Time wait_start = ros::Time::now();
            
            if (!requireCooperation()) {
              ROS_INFO("Cooperation ready!");
              notifyRegionCompletion();
              transitionToNextRegion();
            } 
            else if ((ros::Time::now() - wait_start).toSec() > 300.0) { // 5分钟超时
              std_msgs::String alert;
              alert.data = "UAV1 waiting timeout at region " + std::to_string(current_region_id);
              wait_start = ros::Time::now(); // 重置计时器
            }
            else {
              //publishHover();
              ROS_INFO("UAV1: Still waiting for UAV2(%d) and UAV3(%d) to reach region %d",
                uav2_status_.region_id, uav3_status_.region_id, current_region_id-1);
            }
          }

       /*工具函数*/
        //创造点
        geometry_msgs::PoseStamped createWaypoint(double x, double y, double z_offset) {
            geometry_msgs::PoseStamped wp;
            wp.pose.position.x = region_center_.x + x;
            wp.pose.position.y = region_center_.y + y;
            wp.pose.position.z = 2.5;  // 固定高度
            wp.pose.orientation.w = 1.0;
            const double MAX_VAL = 1e6;
            if (wp.pose.position.x > MAX_VAL || wp.pose.position.y > MAX_VAL || wp.pose.position.z > MAX_VAL) {
                ROS_ERROR("Invalid waypoint detected: [%.2f, %.2f, %.2f]", 
                    wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
                throw std::invalid_argument("Invalid waypoint");
            }
            return wp;
        }

        //生成区域扫描点
        //Z形扫描
        void initializeZigzagWaypoints() {
            waypoints_.clear();
            const int num_lanes = static_cast<int>(scan_width_ / lane_spacing_);
            ROS_WARN("Region center: (%.1f, %.1f, %.1f)", 
             region_center_.x, region_center_.y, region_center_.z);
             ROS_WARN("Parameters: scan_width=%.1f, scan_length=%.1f, lane_spacing=%.1f, forward_offset=%.1f",
                scan_width_, scan_length_, lane_spacing_, foward_offset_);
            for (int i = 0; i <= num_lanes; ++i) {
                // 计算当前车道的 Y 偏移（相对于中心点 region_center_）
                // 例如，i=0 → y_offset = -5.0（最下方车道）
                //      i=1 → y_offset = -3.0
                //      ...
                //      i=5 → y_offset = +5.0（最上方车道）
                double y_offset = (i * lane_spacing_) - (scan_width_ / 2.0);
        
                // 交替方向生成航点（Z 形路径的关键）
                if (i % 2 == 0) {
                    // **偶数车道（i=0,2,4,...）：从左到右（X 最小 → X 最大）**
                    waypoints_.push_back(createWaypoint(
                        - (scan_length_ / 2.0),  // X 起点（最小）
                         y_offset,              // Y 当前车道偏移
                        0                          // Z 固定高度
                    ));
                    waypoints_.push_back(createWaypoint(
                        (scan_length_ / 2.0),  // X 终点（最大）
                         y_offset,              // Y 当前车道偏移
                        0                          // Z 固定高度
                    ));
                } else {
                    // **奇数车道（i=1,3,5,...）：从右到左（X 最大 → X 最小）**
                    waypoints_.push_back(createWaypoint(
                        (scan_length_ / 2.0),  // X 起点（最大）
                         y_offset,              // Y 当前车道偏移
                        0                          // Z 固定高度
                    ));
                    waypoints_.push_back(createWaypoint(
                        - (scan_length_ / 2.0),  // X 终点（最小）
                        y_offset,              // Y 当前车道偏移
                        0                          // Z 固定高度
                    ));
                }
            }
            
        }

        //飞到下一个区域中心点
        void transitionToNextRegion() {
            
            // 发布转移状态
            // mine_detection::ScanStatus status_msg;
            // status_msg.region_id = current_region_id;
            // status_msg.status = static_cast<int>(UAVState::TRANSITION);
            // status_pub_.publish(status_msg);
        
            
            // 根据方向设置下一个区域中心
            geometry_msgs::PoseStamped goal;
            if(current_direction_ == Direction::LEFT) { //？ 考虑了解清除坐标系xy轴正方向
                region_center_.y = region_center_.y + scan_width_;
            } 
            else if(current_direction_ == Direction::RIGHT) {
                region_center_.y = region_center_.y - scan_width_;
            }else if(current_direction_ == Direction::FRONT) {
                region_center_.x = region_center_.x + scan_length_;
            }
            
            // 发布目标位置
            goal.header.stamp = ros::Time::now();
            goal.pose.position = region_center_;
            goal.pose.orientation.w = 1.0; // 默认朝向

            goal_pub_.publish(goal);
            current_state_ = UAVState::TRANSITION;
            ROS_INFO("Transitioning to region %d at (x,y,z):(%.1f, %.1f,%.1f)", 
                    current_region_id+1, region_center_.x, region_center_.y, region_center_.z);
        }
        
        //ego回调函数
        void egoStatusCallback(const mine_detection::WaypointStatus::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(ego_mutex_);
            if(msg->isReached) {
                current_waypoint_reached_ = true;
                current_position = msg->position;
                ROS_INFO("Waypoint confirmed reached by EGO planner");
            }
        }

        // 检查是否到达
        bool checkWaypointReached(geometry_msgs::Point goal) {
            std::lock_guard<std::mutex> lock(ego_mutex_);
            double dx = current_position.x - goal.x;
            double dy = current_position.y - goal.y;
            double dz = current_position.z - goal.z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            ROS_INFO("Now at:(%.1f, %.1f,%.1f),goal at :(%.1f, %.1f,%.1f),distance:(%.1f)", 
                current_position.x, current_position.y, current_position.z,goal.x, goal.y, goal.z ,distance);
            const double distance_threshold = 0.08;
            if(current_waypoint_reached_ && (distance < distance_threshold)) {
                current_waypoint_reached_ = false;
                return true;
                ROS_WARN("Reach!");
            }
            return false;
        }

        // 发布z型扫描点
        void publishCurrentWaypoint() {
            if (waypoints_.empty() || current_waypoint_index_ >= waypoints_.size()) {
                ROS_WARN("No waypoints or scan completed!");
                return;
            }
    
            // 发布当前路径点给EGO-Planner
            goal_pub_.publish(waypoints_[current_waypoint_index_]);
            geometry_msgs::PoseStamped wp;
            wp = waypoints_[current_waypoint_index_];
            ROS_INFO("waypoint publish: [%.2f, %.2f, %.2f]", 
                wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
            
            // 模拟检测地雷（实际由摄像头节点处理）
            simulateMineDetection(waypoints_[current_waypoint_index_]);
    
            // 更新下一个路径点
            current_waypoint_index_++;
        }
        
        //完成当前区域扫描，状态转换
        void finishCurrentRegion() {
            // 1. 记录当前区域已完成
            current_region_id++;
            current_waypoint_index_ = 0; //归0
            // 2. 决策是否需要协同等待
            if (requireCooperation()) {
                current_state_ = UAVState::WAITING;
                notifyRegionCompletion();
                ROS_INFO("UAV1: Waiting for UAV2 and UAV3 to complete region %d", current_region_id);
            } else {
                notifyRegionCompletion();
                transitionToNextRegion();
            }

            
        }

        //检查协同函数
        bool requireCooperation() const {
            // 检查是否需要等待其他无人机 1需要，0不需要
            return (uav2_status_.region_id + 1  < current_region_id);
        }

        //通知uav2/3 uav1已经完成的区域
        void notifyRegionCompletion() {
            mine_detection::UAVStatus msg;
            msg.region_id = current_region_id;
            notify_pub_.publish(msg);

            // 2. 然后发布该区域的所有地雷
            if (!current_region_mines_.empty()) {
                mine_detection::MineArray mines_msg;
                mines_msg.header.stamp = ros::Time::now();
                mines_msg.header.frame_id = "map";
                
                // 添加所有地雷到消息中
                for (const auto& mine : current_region_mines_) {
                    mines_msg.mines.push_back(mine);
                }
                
                // 发布地雷数据
                mines_pub_.publish(mines_msg);
                ROS_INFO("UAV1: Published %zu mines for region %d", 
                         current_region_mines_.size(), current_region_id);
                
                // 短暂等待，确保消息已发送
                ros::Duration(0.1).sleep();
            } else {
                ROS_INFO("UAV1: No mines detected in region %d", current_region_id);
            }
            
            // 3. 清空当前区域的地雷列表，准备下一个区域
            current_region_mines_.clear();
        }
        

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_planner");
    ScanPlanner planner;
    ros::spin();
    return 0;
}