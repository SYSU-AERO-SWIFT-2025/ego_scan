
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mine_detection/MineArray.h>
#include <mine_detection/UAVStatus.h>
#include <nav_msgs/Odometry.h>
#include <mine_detection/HumanPose.h>

// 状态机枚举
enum class UAVState { SCANNING, WAITING, TRANSITION, COMPLETED };
// 方向枚举（1:前 2:左 3:右）
enum class Direction { FRONT = 1, LEFT = 2, RIGHT = 3 };

class ScanPlanner{
    public:
        ScanPlanner():nh_("~"),current_state_(UAVState::TRANSITION), current_direction_(Direction::FRONT) {
            //参数初始化 //需要改回正常的参数初始化
            nh_.param("scan_width", scan_width_, 4.0);
            nh_.param("scan_length", scan_length_, 4.0);
            nh_.param("lane_spacing", lane_spacing_, 2.0);
            nh_.param("forward_offset", foward_offset_, 2.5);
            region_center_.x = 0.0;
            region_center_.y = 0.0;
            region_center_.z = 10.0; 
            // can_width_ = 4;
            // scan_length_ = 4;
            // lane_spacing_ = 1;
        // 话题订阅与发布
        //human_pose_sub_ = nh_.subscribe("/human/pose", 1, &ScanPlanner::humanPoseCallback, this);
        uav2_status_sub_ = nh_.subscribe("/uav2/scan_status", 1, &ScanPlanner::uav2StatusCallback, this);
        uav3_status_sub_ = nh_.subscribe("/uav3/scan_status", 1, &ScanPlanner::uav3StatusCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);//
        mines_pub_ = nh_.advertise<mine_detection::MineArray>("/uav1/detected_mines", 10);

            // 启动状态机
         transitionToNextRegion();
         scan_timer_ = nh_.createTimer(ros::Duration(0.5), &DynamicScanPlanner::stateMachineCallback, this);

        }



        //模拟地雷扫描
        void simulateMineDetection(const geometry_msgs::PoseStamped& wp) {
            mine_detection::MineArray mines;
            if (rand() % 10 < 3) {  // 30%概率模拟检测到地雷
                mine_detection::Mine mine;
                mine.position.x = wp.pose.position.x + (rand() % 3 - 1.5);
                mine.position.y = wp.pose.position.y + (rand() % 3 - 1.5);
                mine.position.z = 0;
                mines.mines.push_back(mine);
                mines_pub_.publish(mines);
            }
        }


        void scanCallback(const ros::TimerEvent&) {
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
    

        private:
        ros::NodeHandle nh_;
        ros::Subscriber human_pose_sub_,uav2_status_sub_, uav3_status_sub_;;
        ros::Publisher mines_pub_, goal_pub_;
        ros::Timer scan_timer_;
        
        std::vector<geometry_msgs::PoseStamped> waypoints_;
        size_t current_waypoint_index_;
        int scan_direction_;
        
        //状态信息
        UAVState current_state_;
        //朝向信息
        Direction current_direction_;
        //
        mine_detection::UAVStatus uav2_status_;
        mine_detection::UAVStatus uav3_status_;
        geometry_msgs::Point region_center_;
        int current_scan_orientation_; //新增朝向控制
        double foward_offset_; //扫描区域中心距离人的偏移量
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
            ROS_DEBUG("UAV2 Status Update - Region: %d, Status: %s", 
                msg->region_id,
                (msg->status == 0) ? "IDLE" : 
                (msg->status == 1) ? "SCANNING" : "COMPLETED");
        }

        //uav3完成状态回调函数
        void uav3StatusCallback(const mine_detection::UAVStatus::ConstPtr& msg) {
            uav3_status_ = *msg; // 拷贝完整状态
            ROS_DEBUG("UAV2 Status Update - Region: %d, Status: %s", 
                msg->region_id,
                (msg->status == 0) ? "IDLE" : 
                (msg->status == 1) ? "SCANNING" : "COMPLETED");
        }

        // 状态机
        void stateMachineCallback(const ros::TimerEvent&) {
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
          }
        
        /*状态处理函数*/
        // 扫描状态处理函数：生成扫描点发布
        void handleScanning() {
            if (current_waypoint_index_ < waypoints_.size()) {
              publishCurrentWaypoint();
              checkWaypointReached();
            } else {
              finishCurrentRegion();
            }
          }        



       /*工具函数*/
        //创造点
        geometry_msgs::PoseStamped createWaypoint(double x, double y, double z_offset) {
            geometry_msgs::PoseStamped wp;
            wp.pose.position.x = region_center_.x + x;
            wp.pose.position.y = region_center_.y + y;
            wp.pose.position.z = 10.0;  // 固定高度
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
                        region_center_.x - (scan_length_ / 2.0),  // X 起点（最小）
                        region_center_.y + y_offset,              // Y 当前车道偏移
                        region_center_.z                          // Z 固定高度
                    ));
                    waypoints_.push_back(createWaypoint(
                        region_center_.x + (scan_length_ / 2.0),  // X 终点（最大）
                        region_center_.y + y_offset,              // Y 当前车道偏移
                        region_center_.z                          // Z 固定高度
                    ));
                } else {
                    // **奇数车道（i=1,3,5,...）：从右到左（X 最大 → X 最小）**
                    waypoints_.push_back(createWaypoint(
                        region_center_.x + (scan_length_ / 2.0),  // X 起点（最大）
                        region_center_.y + y_offset,              // Y 当前车道偏移
                        region_center_.z                          // Z 固定高度
                    ));
                    waypoints_.push_back(createWaypoint(
                        region_center_.x - (scan_length_ / 2.0),  // X 终点（最小）
                        region_center_.y + y_offset,              // Y 当前车道偏移
                        region_center_.z                          // Z 固定高度
                    ));
                }
            }
            
        }
        
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_planner");
    ScanPlanner planner;
    ros::spin();
    return 0;
}