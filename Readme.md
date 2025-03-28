# Ros1Go2Estimator 🦾
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

只成功编译过，没有实机测试，需要根据ROS2GO2Estimator的fusion_estimator_node.cpp，自行修改信号的输入输出

## ⚙️ 安装指南
```bash
git clone --recursive https://github.com/ShineMinxing/Ros1Go2Estimator.git
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