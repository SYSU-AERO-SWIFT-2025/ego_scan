#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap; //用于快速查询机器人周围的障碍物
vector<int> pointIdxRadiusSearch; //存储搜索结果
vector<float> pointRadiusSquaredDistance; //存储搜索结果

random_device rd;
// default_random_engine eng(4);
default_random_engine eng(rd()); 
//随机数生成器，用于生成障碍物的位置（x, y）、宽度（w）、高度（h）和膨胀系数（inf）
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;

ros::Publisher _local_map_pub; //发布局部地图
ros::Publisher _all_map_pub; //发布全局地图
ros::Publisher click_map_pub_; //点击生成的地图
ros::Subscriber _odom_sub; //接收机器人的里程计信息

vector<double> _state; //存储机器人状态
//地图参数
int _obs_num; 
double _x_size, _y_size, _z_size;
//障碍物的生成范围
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
//地图的分辨率、传感器的感知范围、感知频率、机器人的初始位置
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
//障碍物之间的最小距离
double _min_dist;
//地图是否生成完成和是否接收到里程计信息
bool _map_ok = false;
bool _has_odom = false;

//圆形障碍物的数量、半径范围、高度范围和旋转角度范围
int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

//存储全局地图和局部地图的点云数据
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

//存储全局地图的点云数据和点击生成的点云数据
sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

void RandomMapGenerate() {
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution); //坐标化障碍物位置

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)//生成点云数据
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // generate circle obs 生成圆形障碍物
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z); //障碍物中心点

    double theta = rand_theta_(eng); //随机生成旋转角度
    Eigen::Matrix3d rotate;//旋转矩阵
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng); //圆环的两个半径
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      //// 对圆形进行膨胀（增加分辨率）
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size(); //点云宽度
  cloudMap.height = 1; //点云高度
  cloudMap.is_dense = true; //点云是否密集

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true; //地图生成完成
}

//跟上面那个差不多，但是生成的是圆柱体障碍物
void RandomMapGenerateCylinder() {
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num && ros::ok(); i++) {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);
    
    bool flag_continue = false;
    for ( auto p : obs_position )
      if ( (Eigen::Vector2d(x,y) - p).norm() < _min_dist /*metres*/ )
      {
        i--;
        flag_continue = true;
        break;
      }
    if ( flag_continue ) continue;

    obs_position.push_back( Eigen::Vector2d(x,y) );
    

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((w*inf) / _resolution);
    double radius = (w*inf) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++) {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= radius )
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

//diy地图
void DiyMapGenerate() {
  pcl::PointXYZ pt_random;

  // 设置三个长方体的中心位置 (x, y, z)
  double center_x = 5.0;  // 中心位置的x坐标
  double center_y = -5.0;  // 中心位置的y坐标
  double center_z = 0.0;  // 中心位置的z坐标

  // 设置长方体的宽度、高度和深度
  double width = 2.0;   // 宽度
  double height = 2.0;  // 高度
  double depth = 2.0;   // 深度

  // 设置三个长方体之间的间隔
  double spacing = 6.0;  // 间隔

  // 生成三个长方体障碍物
  for (int i = 0; i < 3; i++) {
    double x = center_x ;  // 每个长方体的x坐标
    double y = center_y+ i * spacing;                // y坐标保持不变
    double z = center_z;                // z坐标保持不变

    // 将坐标对齐到地图的分辨率网格
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    // 计算长方体占据的网格数量
    int widNum = ceil(width / _resolution);   // 宽度方向的网格数量
    int heiNum = ceil(height / _resolution);  // 高度方向的网格数量
    int depNum = ceil(depth / _resolution);   // 深度方向的网格数量

    // 在长方体范围内生成点云
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -heiNum / 2.0; s < heiNum / 2.0; s++)
        for (int t = -depNum / 2.0; t < depNum / 2.0; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = z + (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);  // 将点添加到全局地图
        }
  }

  // 设置点云属性
  cloudMap.width = cloudMap.points.size();  // 点云宽度（点数）
  cloudMap.height = 1;                     // 点云高度（单行点云）
  cloudMap.is_dense = true;                // 点云是否密集（无无效点）

  ROS_WARN("Finished generate random map ");

  // 将全局地图的点云数据输入到KD树中
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;  // 设置标志位，表示地图生成完成
}

//接收里程计信息
void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

//
int i = 0;
void pubSensedPoints() {
  // if (i < 10) {
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd); //发布全局地图
  // }

  return;

  /* ---------- only publish points around current position ---------- */
  if (!_map_ok || !_has_odom) return;

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x; //获取点击位置
  double y = msg.pose.position.y; 
  double w = rand_w(eng); //随机生成障碍物的宽度
  double h; //障碍物高度
  pcl::PointXYZ pt_random; 

  //对齐到地图的分辨率网络
  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / _resolution);//计算宽度占据的网格数量

  //在宽度和高度方向上生成点云
  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      h = rand_h(eng);
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++) {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }

  //设置点云属性
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;
    //发布点云信息
  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);

  n.param("min_distance", _min_dist, 1.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  unsigned int seed = rd();
  // unsigned int seed = 2433201515;
  cout << "seed=" << seed << endl;
  eng.seed(seed);

  // RandomMapGenerate();
  //RandomMapGenerateCylinder();
  DiyMapGenerate();
  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}