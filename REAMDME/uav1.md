无人机扫描规划器 (ScanPlanner) 文档
概述
ScanPlanner 是一个用于无人机(UAV1)执行区域扫描任务的ROS节点，主要功能包括：

生成Z型扫描路径
管理无人机状态机
与其他无人机(UAV2/UAV3)协同工作
与EGO规划器交互实现避障飞行
主要功能

状态机管理
无人机工作流程通过状态机实现，包含以下状态：
INIT: 初始化状态
SCANNING: 执行Z型扫描
WAITING: 等待其他无人机完成扫描
TRANSITION: 转移到下一个扫描区域
COMPLETED: 任务完成
2. Z型扫描路径生成
根据以下参数生成Z型扫描路径：

scan_width: 扫描区域宽度(米)
scan_length: 扫描区域长度(米)
lane_spacing: 扫描线间距(米)
region_center_: 扫描区域中心点
3. 多机协同
通过订阅/发布以下话题实现协同：

/uav2/scan_status: UAV2状态
/uav3/scan_status: UAV3状态
/uav1/scan_status: 发布UAV1状态
接口说明
订阅话题
/human/pose: 接收人员位置(待改造为命令接口)
/uav2/scan_status: UAV2状态信息
/uav3/scan_status: UAV3状态信息
/uav1/waypoint_status: EGO规划器反馈的航点状态
发布话题
/move_base_simple/goal: 发送目标位置给EGO规划器
/uav1/detected_mines: 模拟检测到的地雷信息
/uav1/scan_status: 发布UAV1的扫描状态
参数配置
可通过ROS参数服务器配置:

yaml
scan_width: 4.0 # 扫描区域宽度(米)
scan_length: 4.0 # 扫描区域长度(米)
lane_spacing: 2.0 # 扫描线间距(米)
forward_offset: 2.5 # 前向偏移量(米)