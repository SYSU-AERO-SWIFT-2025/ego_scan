team_launch可以用于仿真集体启动所有无人机
每个无人机的扫描包应该有个类似uav1下的uav1.launch的启动文件（直接复制粘贴格式，修改参数）

具体启动流程：

```bash
catkin_make
roslaunch team_launch team.launch
```
