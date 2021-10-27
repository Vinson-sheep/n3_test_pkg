# 使用说明

该ros包基于[radio_proxy](https://github.com/Vinson-sheep/radio_proxy)，使用前务必部署好。

*ubuntu 18.04 / ros melodic*

## 测试前准备

确保**dji_sdk**和**radio_proxy**编译完成；

### 地面端

修改`.launch`

```
roscd n3_test_pkg/launch

gedit station_setup.launch
```

修改`uav_num`，即测试飞机的最大编号（默认从1开始）。

> 假设飞机最大编号是3，即使只有3号飞机进行测试，也要填3，因为地面站的命令是群发的。

### 飞机端

修改`~/.bashrc`，添加

```
// 根据需要填写id
export dji_id=1
```



## 飞行测试

1. 目标飞机指示灯绿色后使用遥控器挂P档起飞降落，确保遥控器正常；
2. 利用wifi连接无人机和地面站，确保ping命令正常工作；
3. 飞机端用尼龙绳捆绑一臂；
4. 使用ssh命令连接无人机和地面站；

###  地面端

```
roslaunch n3_test_pkg station_setup.launch
```

### 飞机端

```
roslaunch n3_test_pkg dji_setup.launch
```

> 飞机端命令可以设置开机启动，这样就无需使用ssh命令。



回到**地面端**，观察目标飞机在终端中的数据是否正常。遵循终端指令完成整套动作（起飞->移动->降落）。完成测试。