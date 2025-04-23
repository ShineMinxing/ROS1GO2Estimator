# Ros1Go2Estimator 🦾
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## ⚙️ unitree_sdk2 编译

在fusion_estimator_node.cpp中修改你的网卡地址
```bash
unitree::robot::ChannelFactory::Instance()->Init(0, "1234abcd5678efg");
```
然后执行
```bash
cd src/Ros1Go2Estimator/unitree_sdk2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=install
make -j4
make install
```
(下面是原理解释，一直到下一节都不需要再操作了)
编译出3个cmake文件
```bash
unitree_sdk2Config.cmake
unitree_sdk2ConfigVersion.cmake
unitree_sdk2Targets.cmake
```

然后放在fusion_estimator的CMakeList中

```bash
set(unitree_sdk2_DIR "${CMAKE_SOURCE_DIR}/../unitree_sdk2/build/install/lib/cmake/unitree_sdk2")
```

同时参考ros2版本的unitree_sdk2位置插入
```bash
find_package unitree_sdk2
include_directories ${unitree_sdk2_INCLUDE_DIRS}
target_link_libraries unitree_sdk2
```

## ⚙️ 安装指南
```bash
git clone https://github.com/ShineMinxing/Ros1Go2Estimator.git
cd Ros1Go2Estimator
source /opt/ros/noetic/setup.bash
catkin_make
source ~/Ros1Go2Estimator/devel/setup.bash 
roscore
rosrun fusion_estimator fusion_estimator_node
```

## 📄 相关文档
- 核心算法原理: [技术白皮书](https://github.com/ShineMinxing/FusionEstimation.git)
- 历史项目参考: [FusionEstimation项目](https://github.com/ShineMinxing/FusionEstimation.git)

## 📧 联系我们
``` 
博士团队: 401435318@qq.com  
研究所: 中国科学院光电技术研究所
```

> 📌 注意：当前为开发预览版，完整文档正在编写中
``