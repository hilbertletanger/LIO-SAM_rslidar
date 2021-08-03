# LIO-SAM/AVP

**lio-sam在速腾激光雷达上的复现,然后添加了一个面向AVP项目的基于激光的停车位检测的功能**

## 主要修改点
* 在rslidar上的lio-sam复现

主要是imageProject里和params参数的修改，简单的适配的了一下rslidar

* 新增了一个功能，面向园区AVP（自动泊车）项目的基于简单激光目标检测的停车位检测

整体思路为，用简单的激光目标检测算法（KNN聚类）来进行障碍物检测，这里不考虑使用更高级更精确的算法，因为本来就不是为了做激光object Detection。然后用划定的ROI区域（代码中称为detectBox）和目标框之间进行碰撞检测，如果没有碰撞，则当前ROI即为可停车车位。如果当前ROI有碰撞（所以不可停车），那么返回的可停车车位是最近的车位（使用SLAM的定位结果来获得这个位置在当前自车坐标系下的位置）。
开源部分只是一个快速实现，没有考虑任何算法优化和内存占用，如有需要请自行修改即可，比如tempBox显然没必要在每帧重新开辟



## Paper 

Thank you for citing [LIO-SAM (IROS-2020)](./config/doc/paper.pdf) if you use any of this code. 
```
@inproceedings{liosam2020shan,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135-5142},
  year={2020},
  organization={IEEE}
}
```

