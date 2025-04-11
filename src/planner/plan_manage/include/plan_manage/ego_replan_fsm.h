#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT, //初始化状态
      WAIT_TARGET, //等待目标状态。等待用户输入目标
      GEN_NEW_TRAJ, //生成新轨迹状态
      REPLAN_TRAJ, //重新规划轨迹状态
      EXEC_TRAJ, //执行轨迹状态
      EMERGENCY_STOP, //紧急停止状态
      SEQUENTIAL_START //顺序启动状态
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1, //手动目标
      PRESET_TARGET = 2, //预设目标
      REFENCE_PATH = 3  //目标是路径
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_; //轨迹的生成
    PlanningVisualization::Ptr visualization_; //结果可视化
    traj_utils::DataDisp data_disp_; //显示和记录轨迹数据
    traj_utils::MultiBsplines multi_bspline_msgs_buf_; //管理多条B样条轨迹

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_; //目标点与无人机当前位置小于no_replan_thresh_就不需要再规划到这个点的轨迹
    double waypoints_[50][3];  //路标点三轴位置
    int waypoint_num_, wp_id_;  //路标点个数，每架飞机一个终点 
    double planning_horizen_, planning_horizen_time_; //局部规划的范围 
    double emergency_time_; //如果距离碰撞的时间小于该值，立刻切换到停止模式
    bool flag_realworld_experiment_; //是否为真实环境实验
    bool enable_fail_safe_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    bool is_waypoint_queue_running_=false;//标记是否正在执行队列
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, swarm_trajs_sub_, broadcast_bspline_sub_, trigger_sub_;
    ros::Publisher waypoint_status_pub_; // 新增发布器
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, swarm_trajs_pub_, broadcast_bspline_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg);
    void BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg);

    bool checkCollision();
    void publishSwarmTrajs(bool startup_pub);

    void publishWaypointReached(int wp_id, const Eigen::Vector3d& position);
  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif