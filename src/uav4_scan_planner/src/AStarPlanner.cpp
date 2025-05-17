#include "AStarPlanner.h"
#include <algorithm>
#include <ros/console.h> // 用于ROS调试输出

AStarPlanner::AStarPlanner(double resolution, double safe_distance) 
    : resolution_(resolution), safe_distance_(safe_distance) {} //初始化

// 主要规划函数
nav_msgs::Path AStarPlanner::planPathToBoundary(
    const geometry_msgs::Point& start, 
    const geometry_msgs::Point& boundary_point,
    int boundary_x_or_y,
    const std::vector<geometry_msgs::Point>& obstacles,
    double flight_height) {
    // 打印调试信息
    ROS_INFO("\n-----UAV4 Planning started -----");
    ROS_INFO("Start point: (%.2f, %.2f)", start.x, start.y);
    ROS_INFO("Boundary point: (%.2f, %.2f), direction: %s", 
             boundary_point.x, boundary_point.y, 
             boundary_x_or_y == 0 ? "X" : "Y");
    ROS_INFO("Obstacles count: %zu", obstacles.size());    
    ROS_INFO("A* min_x=%.2f max_x=%.2f min_y=%.2f max_y=%.2f", min_x, max_x, min_y, max_y);
    
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    
    // 转换为网格坐标
    Node* start_node = new Node(static_cast<int>(start.x / resolution_), 
                               static_cast<int>(start.y / resolution_));
    ROS_DEBUG("Start node (grid): (%d, %d)", start_node->x, start_node->y);
    // 优先队列，比较函数为价值f大小，f越小优先级越高，在队列的越前面
    auto cmp = [](Node* left, Node* right) { return left->f > right->f; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_set(cmp);
    
    // 已经访问的节点
    std::unordered_set<Node*, NodeHash, NodeEqual> closed_set;
    // 初始化开始节点，计算启发函数
    start_node->h = heuristicToBoundary(start_node, boundary_point, boundary_x_or_y);
    start_node->f = start_node->h;
    open_set.push(start_node);
    
    Node* goal_node = nullptr;
    
    while (!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();
        
        // 检查是否到达边界区域 
        if (isInBoundary(current, boundary_point, boundary_x_or_y)) {
            goal_node = current;
            break;
        }
        
        // 已经访问放入closed_set
        closed_set.insert(current);
        
    for (Node* neighbor : getNeighbors(current, closed_set, obstacles)) {
        // 计算从current到neighbor的实际移动代价
        const double move_cost = calculateMoveCost(current, neighbor);  // 实际移动代价函数
        const double tentative_g = current->g + move_cost;
    
        // 检查是否需要跳过该邻居，被访问过，且当前g>=上次的g
        const bool is_visited = (closed_set.find(neighbor) != closed_set.end());
        if (is_visited && tentative_g >= neighbor->g) {
            delete neighbor;
            continue;
        }
    
        // 如果找到更优路径或首次访问该节点
        if (!is_visited || tentative_g < neighbor->g) {
        // 更新节点信息
        neighbor->parent = current;
        neighbor->g = tentative_g;
        neighbor->h = heuristicToBoundary(neighbor, boundary_point, boundary_x_or_y);  //启发式
        neighbor->f = neighbor->g + neighbor->h;
        
        // 如果是新发现的节点，加入开放集
        if (!is_visited) {
            open_set.push(neighbor);
        } else {
            // 如果是已访问节点但找到了更优路径
            // 需要重新调整该节点在open_set中的位置
            // 注意：标准priority_queue无法直接更新，这里需要特殊处理
            // 简单实现方式是重新插入（会有重复节点但不影响正确性）
            open_set.push(neighbor);  
        }
      } else {
            // 不需要更新的情况，释放内存
            delete neighbor;
        }
    }

    }
    // 
    if (goal_node != nullptr) { //到达边界后重构路径
        path = reconstructPath(goal_node, flight_height);
    }
    
    // 清除closed_set和open_set
    for (Node* node : closed_set) {
        delete node;
    }
    while (!open_set.empty()) {
        delete open_set.top();
        open_set.pop();
    }
    
    return path;
}

// 启发函数，boundary_x_or_y为边界方向，boundary_pointr看成目标边上某点
double AStarPlanner::heuristicToBoundary(const Node* node, 
                                       const geometry_msgs::Point& boundary_point,
                                       int boundary_x_or_y) {
    // 转化回原来的物理左边
    double node_x = node->x * resolution_;
    double node_y = node->y * resolution_;
    double h;
    if(boundary_x_or_y == 0){ //x方向
        h = std::abs(node_x - boundary_point.x);
    }else{ // y方向
        h = std::abs(node_y - boundary_point.y);
    }

    return h;
}

// 0.05m容忍误差，可修改
bool AStarPlanner::isInBoundary(const Node* node,
                              const geometry_msgs::Point& boundary_point,
                              int boundary_x_or_y) {
    double node_x = node->x * resolution_;
    double node_y = node->y * resolution_;
    
    double dx = std::abs(node_x - boundary_point.x);
    double dy = std::abs(node_y - boundary_point.y);
    
    if(boundary_x_or_y == 0){ //x方向
        return dx<0.05;
    }else{ // y方向
        return dy<0.05;
    }
}

std::vector<AStarPlanner::Node*> AStarPlanner::getNeighbors(
    Node* node, 
    const std::unordered_set<Node*, NodeHash, NodeEqual>& closed_set,
    const std::vector<geometry_msgs::Point>& obstacles) {
    
    std::vector<Node*> neighbors;
    const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    for (int i = 0; i < 8; ++i) {
        // 计算邻居节点的网格坐标
        int nx = node->x + dx[i];
        int ny = node->y + dy[i];
        
        // 将网格坐标转换为物理坐标（单位：米）
        double physical_x = nx * resolution_;
        double physical_y = ny * resolution_;
        
        // 检查是否超出地雷区边界
        if (physical_x < min_x || physical_x > max_x || 
            physical_y < min_y || physical_y > max_y) {
            continue; // 跳过超出边界的邻居
        }
        
        // 创建邻居节点并检查碰撞和闭集
        Node* neighbor = new Node(nx, ny);
        if (isCollisionFree(neighbor, obstacles)) {
            ROS_DEBUG("Valid neighbor: (%d,%d)", nx, ny);
            neighbors.push_back(neighbor);
        } else {
            delete neighbor;
        }
    }
    
    return neighbors;
}

// 检测邻居点是否和地雷有不安全距离
bool AStarPlanner::isCollisionFree(const Node* node, const std::vector<geometry_msgs::Point>& obstacles) {
    double node_x = node->x * resolution_;
    double node_y = node->y * resolution_;
    
    for (const auto& obs : obstacles) {
        double dist_sq = std::pow(node_x - obs.x, 2) + std::pow(node_y - obs.y, 2);
        if (dist_sq < std::pow(safe_distance_, 2)) {
            return false;
        }
    }
    return true;
}

// 构建路径
nav_msgs::Path AStarPlanner::reconstructPath(Node* node, double flight_height) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    
    std::vector<Node*> nodes;
    while (node != nullptr) {
        nodes.push_back(node);
        node = node->parent;
    }
    std::reverse(nodes.begin(), nodes.end());
    
    for (Node* n : nodes) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = n->x * resolution_;
        pose.pose.position.y = n->y * resolution_;
        pose.pose.position.z = flight_height;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    
    return path;
}

double AStarPlanner::calculateMoveCost(const Node* from, const Node* to) const {
    const double dx = (to->x - from->x) * resolution_;
    const double dy = (to->y - from->y) * resolution_;
    return std::hypot(dx, dy);  // 欧氏距离
}