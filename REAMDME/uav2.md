
# UAV2扫描验证节点使用说明文档

## 1. 概述

`uav2_scan_planner_node`是一个用于无人机地雷验证的ROS节点，它配合UAV1工作，负责对UAV1在区域扫描过程中发现的潜在地雷进行二次验证。UAV2采用最短路径规划算法优化验证路径，依次飞往各个地雷点，进行悬停和验证，并发布确认为真实的地雷信息。

## 2. 状态机设计

节点采用状态机管理UAV2的行为流程：

| 状态                                                                                                                           | 描述                     |
| ------------------------------------------------------------------------------------------------------------------------------ | ------------------------ |
| [WAIT](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)               | 等待UAV1完成区域扫描     |
| [MOVING](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)             | 正在飞往地雷点进行验证   |
| [HOVERING](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)           | 在地雷点上方悬停固定时间 |
| [VERIFYING](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)          | 正在验证地雷真伪         |
| [COMPLETED](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)          | 完成当前区域所有地雷验证 |
| [MOVING_TO_BOUNDARY](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html) | 移动到下一区域边界等待   |

## 3. 话题通信

### 订阅话题

* `/uav1/scan_status`: 接收UAV1的区域完成状态信息
* `/uav1/detected_mines`: 接收UAV1探测到的潜在地雷位置
* `/uav2/odom`: 获取UAV2的当前位置信息
* `/uav2/waypoint_status`: 接收EGO-Planner的航点到达状态

### 发布话题

* `/uav2/scan_status`: 发布UAV2的验证完成状态
* `/uav2/verified_mines`: 发布经验证确认的真实地雷
* `/uav2/move_base_simple/goal`: 发布目标点给EGO-Planner

## 4. 参数配置

| 参数名                       | 类型   | 默认值 | 描述                |
| ---------------------------- | ------ | ------ | ------------------- |
| `hover_time`               | double | 5.0    | 地雷点悬停时间(秒)  |
| `arrival_threshold`        | double | 0.08   | 到达判定阈值(米)    |
| `verification_probability` | double | 0.8    | 地雷验证为真的概率  |
| `region_width`             | double | 4.0    | 扫描区域宽度        |
| `region_length`            | double | 6.0    | 扫描区域长度        |
| `region_start_x`           | double | -10.0  | 第一个区域起始X坐标 |
| `region_start_y`           | double | -9.0   | 区域Y轴中心坐标     |

## 5. 核心功能

### 最短路径优化

使用贪心最近邻算法优化地雷验证路径，减少总飞行距离：

**optimizeMinePath() 函数采用贪心策略从当前位置开始，**

**每次选择最近的未访问地雷点，显著减少总飞行距离。**

### 地雷验证流程

1. 飞往地雷点
2. 悬停指定时间（默认5秒）
3. 使用概率模型验证地雷（默认80%概率验证为真）
4. 将真实地雷添加到已验证列表

### 智能区域边界等待

当某区域无地雷时，主动飞往下一区域边界等待：

**如果UAV1完成区域但没有检测到地雷，UAV2会飞往下一个**

**区域边界位置，提前做好准备，减少后续验证时间。**

### 重复地雷过滤

自动检测并过滤相距过近的重复地雷点：

**使用10厘米阈值判断地雷点是否重复，避免UAV2对同一位置**

**重复验证，提高效率。**

## 6. 操作流程

1. **初始化** ：UAV2启动后进入[WAIT](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)状态，等待UAV1扫描
2. **任务触发** ：UAV1完成区域扫描后，UAV2收到区域ID消息
3. **路径规划** ：如有地雷，使用最近邻算法优化访问顺序
4. **地雷验证** ：

* 飞往地雷点
* 到达后悬停5秒
* 以80%概率将地雷视为真实地雷

1. **结果发布** ：验证完毕后，发布真实地雷列表和完成状态
2. **边界等待** ：如无地雷，飞往下一区域边界等待UAV1

## 7. 到达判断机制

节点使用双重检测方式判断是否到达目标点：

1. **位置计算** ：通过计算当前位置与目标点欧几里得距离来判断
2. **状态回调** ：监听EGO-Planner发布的 `waypoint_status`消息

## 8. 与UAV1的协同工作

UAV2与UAV1通过区域ID进行协同：

1. UAV1扫描区域并发现潜在地雷
2. UAV1完成区域后发送区域ID和地雷信息
3. UAV2收到消息后开始验证该区域地雷
4. UAV2完成验证后通知UAV1，使UAV1可以继续工作
