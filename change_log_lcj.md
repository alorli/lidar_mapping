# ndt_slam:
# edited by lichunjing 2022.05.23:

### 2022.05.23  ------------------------------------------------------------------------------
### 1. gnss_optimization.cc中，lidar_gnss_information_矩阵做了修改
2022-05-23之前使用：
// lidar_gnss_information_ << 25.0,          0.0,           0.0,
//                            0.0,            25.0,         0.0,
//                            0.0,            0.0,           10.0;


2022-05-23起重机园区建图时修改后：
lidar_gnss_information_ << 10.0,          0.0,           0.0,
                           0.0,            10.0,         0.0,
                           0.0,            0.0,           5.0;
