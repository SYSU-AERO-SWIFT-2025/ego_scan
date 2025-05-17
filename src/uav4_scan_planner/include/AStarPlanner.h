// astar_planner.h
#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <vector>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <unordered_set>
#include <cmath>

class AStarPlanner {
public:
    struct Node {
        int x, y;
        // g已经走过的代价，h是启发式估计，f=g+h
        double g, h, f;
        // 父节点
        Node* parent;
        
        Node(int x_, int y_) : x(x_), y(y_), g(0), h(0), f(0), parent(nullptr) {}
        
        //判断两点相同，坐标相同则是相同
        bool operator==(const Node& other) const { 
            return x == other.x && y == other.y;
        }
    };

    //用于closet快速发现节点是否已经访问（closet的结构是unorderset,自定义hash函数），这里最后再检查一次还是有疑问
    struct NodeHash {
        size_t operator()(const Node* node) const {
            // 结合 x 和 y 的简单哈希
            return std::hash<int>()(node->x) ^ (std::hash<int>()(node->y) << 1);
        }
    };

    //比较相等
    struct NodeEqual {
        bool operator()(const Node* lhs, const Node* rhs) const {
            return lhs->x == rhs->x && lhs->y == rhs->y;
        }
    };

    AStarPlanner(double resolution, double safe_distance); //resolution 地图网格化的精确度，safe_distance地雷的安全距离
    
    // 修改为接受目标边界而非单个目标点，最后目标是到达边界的某点即可
    nav_msgs::Path planPathToBoundary(const geometry_msgs::Point& start, 
                                    const geometry_msgs::Point& boundary_point,//边界中心
                                    int boundary_x_or_y,//边界方向
                                    const std::vector<geometry_msgs::Point>& obstacles,//障碍物，实则地雷
                                    double flight_height);//飞行高度
    // 初始化区域参数，每次调用planPathToBoundar前应该初始化
    void initRegionParm(double region_width, double region_length,double region_center_x, double region_center_y,int current_region_id){
        //region_center_x += current_region_id * region_width;
        //region_center_y += current_region_id * region_length;
        min_x = region_center_x - region_width/2;
        max_x = region_center_x + region_width/2;
        min_y = region_center_y - region_length/2;
        max_y = region_center_y + region_length/2;
    }
    
private:
    double resolution_; //地图网格化的精确度
    double safe_distance_;
    
    double heuristicToBoundary(const Node* node, 
                              const geometry_msgs::Point& boundary_point,
                              int boundary_x_or_y); // 启发函数到达边界的距离
    
    bool isInBoundary(const Node* node,
                     const geometry_msgs::Point& boundary_point,
                     int boundary_x_or_y); //判断是否已经到达边界
    
    std::vector<Node*> getNeighbors(Node* node, 
                                  const std::unordered_set<Node*, NodeHash, NodeEqual>& closed_set,
                                  const std::vector<geometry_msgs::Point>& obstacles);//得到邻居节点
    
    bool isCollisionFree(const Node* node, const std::vector<geometry_msgs::Point>& obstacles);
    
    nav_msgs::Path reconstructPath(Node* node, double flight_height); //构建路径
    
    double calculateMoveCost(const Node* from, const Node* to) const;

    //区域边界参数
    double min_x,min_y,max_x,max_y;
};

#endif // ASTAR_PLANNER_H