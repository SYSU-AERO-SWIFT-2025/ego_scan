#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mine_detection/MineArray.h>
#include <mine_detection/WaypointStatus.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <vector>
#include <random>
#include <string>
#include <cmath>
#include <mutex> // Include mutex

// Define the state enumeration for UAV2 (similar structure to UAV1's UAVState)
enum class UAV2State {
    WAIT,       // Waiting for UAV1 scan completion and mine list
    MOVING,     // Moving to the next potential mine location
    HOVERING,   // Hovering over a potential mine for verification
    VERIFYING,  // Simulating the verification process
    COMPLETED   // Verification process for the current batch is complete, ready to notify
};

class UAV2ScanPlanner {
public:
    UAV2ScanPlanner(ros::NodeHandle& nh) :
        nh_(nh), // Use initializer list
        current_state_(UAV2State::WAIT),
        current_mine_index_(-1),
        waypoint_reached_(false),
        hover_start_time_(0),
        gen_(rd_()),
        dis_(0.0, 1.0)
    {
        // Load parameters (using private node handle nh_)
        nh_.param<double>("hover_time", hover_time_, 3.0);
        nh_.param<double>("arrival_threshold", arrival_threshold_, 0.5); // Threshold for fallback check

        // Initialize Subscribers (using private node handle nh_)
        uav1_status_sub_ = nh_.subscribe("/uav1/scan_status", 10, &UAV2ScanPlanner::uav1StatusCallback, this);
        detected_mines_sub_ = nh_.subscribe("/uav1/detected_mines", 10, &UAV2ScanPlanner::detectedMinesCallback, this);
        odom_sub_ = nh_.subscribe("/uav2/odometry", 10, &UAV2ScanPlanner::odomCallback, this);
        // Use egoStatusCallback name for consistency with UAV1's self-status callback
        waypoint_status_sub_ = nh_.subscribe("/uav2/waypoint_status", 10, &UAV2ScanPlanner::egoStatusCallback, this);

        // Initialize Publishers (using private node handle nh_)
        status_pub_ = nh_.advertise<std_msgs::String>("/uav2/scan_status", 10);
        verified_mines_pub_ = nh_.advertise<mine_detection::MineArray>("/uav2/verified_mines", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav2/goal", 10); // Goal for UAV2's navigation

        // Wait for goal subscriber (optional, similar to UAV1)
        if (goal_pub_.getNumSubscribers() == 0) {
            ROS_INFO("UAV2: Waiting for goal subscribers...");
            ros::Time start = ros::Time::now();
            while (ros::ok() && (ros::Time::now() - start).toSec() < 3.0 && goal_pub_.getNumSubscribers() == 0) {
                ros::Duration(0.1).sleep();
            }
             if (goal_pub_.getNumSubscribers() == 0) {
                ROS_WARN("UAV2: No subscriber found for goal topic after waiting.");
            } else {
                ROS_INFO("UAV2: Goal subscriber found.");
            }
        }


        // Initialize State Machine Timer (similar to UAV1)
        state_machine_timer_ = nh_.createTimer(ros::Duration(0.1), &UAV2ScanPlanner::stateMachineCallback, this); // 10 Hz

        ROS_INFO("UAV2 Scan Planner initialized. State: WAIT");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber uav1_status_sub_, detected_mines_sub_, odom_sub_, waypoint_status_sub_;
    ros::Publisher status_pub_, verified_mines_pub_, goal_pub_;
    ros::Timer state_machine_timer_;

    UAV2State current_state_;
    std::vector<geometry_msgs::PoseStamped> potential_mines_;
    std::vector<geometry_msgs::PoseStamped> verified_mines_;
    int current_mine_index_;

    geometry_msgs::Pose current_pose_;
    bool waypoint_reached_;
    ros::Time hover_start_time_;
    std::mutex ego_mutex_; // Mutex to protect shared variables like waypoint_reached_

    // Parameters
    double hover_time_;
    double arrival_threshold_;

    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_;

    // --- Callbacks ---

    // Callback for UAV1's status (e.g., "area_completed")
    void uav1StatusCallback(const std_msgs::String::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(ego_mutex_); // Lock if modifying shared state potentially read by timer
        if (current_state_ == UAV2State::WAIT) {
            ROS_INFO_STREAM("UAV2 received status from UAV1: " << msg->data);
            if (msg->data.find("area_completed") != std::string::npos) {
                // Check if we also have mines before starting
                if (!potential_mines_.empty()) {
                    ROS_INFO("UAV1 completed area. UAV2 starting verification process.");
                    current_mine_index_ = 0;
                    verified_mines_.clear();
                    // Transition happens in the state machine based on conditions met
                } else {
                    ROS_WARN("UAV1 completed area, but UAV2 has no potential mines received yet.");
                }
                 // Set a flag or condition checked by the state machine in WAIT state
                 // For simplicity, we can directly attempt transition if mines are present
                 if (!potential_mines_.empty()) {
                     moveToNextMine(); // Initiate the first move
                 }
            }
        }
    }

    // Callback for UAV1's detected mines
    void detectedMinesCallback(const mine_detection::MineArray::ConstPtr& msg) {
         std::lock_guard<std::mutex> lock(ego_mutex_); // Lock if modifying shared state potentially read by timer
        if (current_state_ == UAV2State::WAIT) {
            // Adapt this based on the actual structure of Mine message vs PoseStamped
            potential_mines_.clear(); // Clear previous mines
            for (const auto& mine : msg->mines){
                geometry_msgs::PoseStamped mine_pose;
                mine_pose.header = msg->header; // Use the same header as the detected mines
                mine_pose.pose.position = mine.position; // Assuming mine has a pose field
                potential_mines_.push_back(mine_pose);
            }
            ROS_INFO("UAV2 received %zu potential mines from UAV1.", potential_mines_.size());
            // If UAV1 status was already received, we might need to trigger the start here too
            // This logic depends on the exact order messages are expected.
            // The current uav1StatusCallback handles starting if mines are present when status arrives.
        } else {
            ROS_WARN("UAV2 received potential mines while not in WAIT state. Ignoring.");
        }
    }

    // Callback for UAV2's own odometry
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // No lock needed if only writing to current_pose_ and it's only read by checkWaypointReached (called within timer)
        current_pose_ = msg->pose.pose;
    }

    // Callback for UAV2's own waypoint status (renamed for consistency)
    void egoStatusCallback(const mine_detection::WaypointStatus::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(ego_mutex_); // Lock as this modifies waypoint_reached_ read by timer
        if (current_state_ == UAV2State::MOVING && msg->isReached) {
             // Check if the reported position matches the current goal reasonably well
             // This prevents reacting to stale 'isReached' messages from previous goals
             if (current_mine_index_ >= 0 && current_mine_index_ < potential_mines_.size()) {
                 double dx = msg->position.x - potential_mines_[current_mine_index_].pose.position.x;
                 double dy = msg->position.y - potential_mines_[current_mine_index_].pose.position.y;
                 // Ignore Z for arrival check? Or use arrival_threshold_
                 double dist_sq = dx*dx + dy*dy;
                 if (dist_sq < arrival_threshold_ * arrival_threshold_ * 4) { // Allow slightly larger tolerance for confirmation
                    if (!waypoint_reached_) { // Avoid spamming logs
                        waypoint_reached_ = true;
                        ROS_INFO("UAV2 Waypoint Status: Reached potential mine location %d.", current_mine_index_ + 1);
                    }
                 } else {
                      ROS_WARN("UAV2 Waypoint Status: 'isReached' is true, but reported position (%.2f, %.2f) doesn't match goal %d (%.2f, %.2f). Ignoring.",
                        msg->position.x, msg->position.y, current_mine_index_+1,
                        potential_mines_[current_mine_index_].pose.position.x, potential_mines_[current_mine_index_].pose.position.y);
                 }
             }
        }
    }

    // --- State Machine ---

    // Main state machine callback driven by timer (like UAV1)
    void stateMachineCallback(const ros::TimerEvent&) {
        // state_machine_timer_.stop(); // Optional: Stop timer during processing

        // Lock the mutex when accessing shared state variables like current_state_ and waypoint_reached_
        std::lock_guard<std::mutex> lock(ego_mutex_);

        switch (current_state_) {
            case UAV2State::WAIT:
                handleWait();
                break;
            case UAV2State::MOVING:
                handleMoving();
                break;
            case UAV2State::HOVERING:
                handleHovering();
                break;
            case UAV2State::VERIFYING:
                handleVerifying();
                break;
            case UAV2State::COMPLETED:
                handleCompleted();
                break;
        }

        // state_machine_timer_.start(); // Optional: Restart timer if stopped
    }

    // --- State Handling Functions (called by stateMachineCallback) ---

    void handleWait() {
        // In WAIT state, the transition is triggered by callbacks
        // (uav1StatusCallback when mines are already present, or potentially
        // detectedMinesCallback if status was already received - needs careful sync logic
        // or rely on uav1StatusCallback as the main trigger after mines arrive).
        // No active processing needed in the timer loop itself for WAIT state.
        ROS_DEBUG_THROTTLE(5.0, "UAV2 State: WAIT"); // Log state periodically
    }

    void handleMoving() {
        ROS_DEBUG_THROTTLE(1.0, "UAV2 State: MOVING to mine %d", current_mine_index_ + 1);
        // Check if the waypoint_reached_ flag was set by the egoStatusCallback
        if (waypoint_reached_) {
            current_state_ = UAV2State::HOVERING;
            hover_start_time_ = ros::Time::now(); // Record hover start time
            ROS_INFO("UAV2 arrived at mine %d. State: HOVERING", current_mine_index_ + 1);
            // Command UAV to hold position if necessary (depends on low-level controller)
            // goal_pub_.publish(potential_mines_[current_mine_index_]); // Re-publish current pose as hold goal?
        }
        // Optional: Add fallback check using odometry if egoStatusCallback is unreliable
        // else if (checkWaypointReached(potential_mines_[current_mine_index_].pose.position)) { ... }
    }

    void handleHovering() {
        ROS_DEBUG_THROTTLE(1.0, "UAV2 State: HOVERING at mine %d", current_mine_index_ + 1);
        // Check if hover duration has elapsed
        if ((ros::Time::now() - hover_start_time_).toSec() >= hover_time_) {
            current_state_ = UAV2State::VERIFYING;
            ROS_INFO("UAV2 hover complete at mine %d. State: VERIFYING", current_mine_index_ + 1);
        }
    }

    void handleVerifying() {
        ROS_INFO("UAV2 State: VERIFYING mine %d", current_mine_index_ + 1);
        // Simulate the verification process
        bool is_real_mine = verifyMineSimulation();

        if (is_real_mine) {
            verified_mines_.push_back(potential_mines_[current_mine_index_]);
            ROS_INFO("Mine %d verified as REAL.", current_mine_index_ + 1);
        } else {
            ROS_INFO("Mine %d determined to be FALSE POSITIVE.", current_mine_index_ + 1);
        }

        // Move to the next mine or complete the process
        current_mine_index_++;
        if (current_mine_index_ < potential_mines_.size()) {
            moveToNextMine(); // This sets state back to MOVING
        } else {
            current_state_ = UAV2State::COMPLETED;
            ROS_INFO("UAV2 finished verifying all %zu potential mines. State: COMPLETED", potential_mines_.size());
        }
    }

    void handleCompleted() {
        ROS_INFO("UAV2 State: COMPLETED. Publishing results and notifying UAV1.");
        publishVerifiedMines();
        notifyCompletion(); // Notify UAV1
        // Reset for next cycle
        current_state_ = UAV2State::WAIT;
        potential_mines_.clear();
        verified_mines_.clear();
        current_mine_index_ = -1;
        waypoint_reached_ = false; // Ensure reset
        ROS_INFO("UAV2 returning to State: WAIT");
    }

    // --- Helper Functions (like UAV1) ---

    // Moves to the next mine in the list
    void moveToNextMine() {
        // Assumes lock is already held by the calling state handler (handleVerifying or uav1StatusCallback)
        if (current_mine_index_ < 0 || current_mine_index_ >= potential_mines_.size()) {
             ROS_ERROR("Invalid mine index %d.", current_mine_index_);
             current_state_ = UAV2State::WAIT; // Go back to wait on error
             return;
        }

        geometry_msgs::PoseStamped goal_pose = potential_mines_[current_mine_index_];
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.header.frame_id = "map"; // Set appropriate frame_id if known, e.g., "map" or "odom"

        goal_pub_.publish(goal_pose);
        waypoint_reached_ = false; // Reset flag for the new goal
        current_state_ = UAV2State::MOVING; // Set state explicitly
        ROS_INFO("UAV2 moving to potential mine %d at (%.2f, %.2f, %.2f). State: MOVING",
                 current_mine_index_ + 1,
                 goal_pose.pose.position.x,
                 goal_pose.pose.position.y,
                 goal_pose.pose.position.z);
    }

    // Simulates the verification process
    bool verifyMineSimulation() {
        // Assumes lock is held by calling state handler (handleVerifying)
        return dis_(gen_) < 0.8; // 80% chance of being real
    }

    // Publishes the list of verified mines
    void publishVerifiedMines() {
        // Assumes lock is held by calling state handler (handleCompleted)
        mine_detection::MineArray verified_msg;
        verified_msg.header.stamp = ros::Time::now();
        verified_msg.header.frame_id = "map"; // Use consistent frame_id

        // Ensure verified_mines_ elements can be assigned to verified_msg.mines
        verified_msg.mines.reserve(verified_mines_.size());
        for (const auto& mine : verified_mines_){
            mine_detection::Mine mine_msg;
            mine_msg.position = mine.pose.position; // Assuming Mine has a pose field
            verified_msg.mines.push_back(mine_msg);
        }

        if (!verified_msg.mines.empty()) {
             verified_mines_pub_.publish(verified_msg);
             ROS_INFO("UAV2 published %zu verified mines.", verified_msg.mines.size());
        } else {
             ROS_INFO("UAV2 found no real mines in this batch.");
             // Publish empty message so downstream nodes know the process finished
             verified_mines_pub_.publish(verified_msg);
        }
    }

    // Notifies UAV1 that verification for the current batch is complete
    void notifyCompletion() {
        // Assumes lock is held by calling state handler (handleCompleted)
        std_msgs::String status_msg;
        status_msg.data = "verification_complete"; // Simple status message
        status_pub_.publish(status_msg);
        ROS_INFO("UAV2 verification complete. Notified UAV1.");
    }

    // Optional: Fallback check using odometry (like UAV1's checkWaypointReached)
    bool checkWaypointReached(const geometry_msgs::Point& goal) {
        // Assumes lock is held by calling state handler (e.g., handleMoving if used there)
        // This function reads current_pose_ which is updated in odomCallback (no lock needed there if only writer)
        double dx = current_pose_.position.x - goal.x;
        double dy = current_pose_.position.y - goal.y;
        double dz = current_pose_.position.z - goal.z; // Consider if Z matters for arrival
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        ROS_DEBUG("Dist to target mine %d: %.2f m", current_mine_index_ + 1, distance);
        return distance < arrival_threshold_;
    }

}; // End of class UAV2ScanPlanner

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav2_scan_planner_node");
    // Use private node handle for parameters, consistent with UAV1 example structure
    ros::NodeHandle nh("~");

    UAV2ScanPlanner planner(nh);

    ros::spin(); // Process callbacks

    return 0;
}