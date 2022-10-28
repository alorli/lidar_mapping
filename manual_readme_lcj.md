# ndt_slam:
# edited by lichunjing 2021.11.25:

# 建图可支持三种建图模式，不同模式需要关注的参数如下：
### ----------------------------------------------------------------------------------------------------------------------------------
### 1.只使用多线激光雷达建图
# 1.1 cfg.yaml中：ros_interface -> subscribe_message -> is_use_lidar设置成 true，其他改成 false
# 1.2 cfg.yaml中：directory -> project_directory 功能目录根据需要设置
# 1.3 cfg.yaml中：map_builder -> map_builder_parameter -> lidar_n_scans 目前只支持vlp16雷达，设置成16，其他型号雷达需要调试

### ----------------------------------------------------------------------------------------------------------------------------------
### 2.使用多线激光雷达建图 + 单天线RTK（RTK天线在雷达正上方）
# 2.1 cfg.yaml中：ros_interface -> subscribe_message -> is_use_lidar 设置成 true，其他改成 false
                 ros_interface -> subscribe_message -> is_use_gnss 设置成 true，其他改成 false
# 2.2 cfg.yaml中：directory -> project_directory 功能目录根据需要设置
# 2.3 cfg.yaml中：map_builder -> map_builder_parameter -> lidar_n_scans 目前只支持vlp16雷达，设置成16，其他型号雷达需要调试
# 2.4 cfg.yaml中：map_builder -> utm_localization 根据建图位置设置 utm分区
# 2.5 cfg.gnss_optimization -> gnss_optimization_parameter -> arm_rtk_antennna_in_vlp 设置雷达和rtk天线的高度差值，例如天线在雷达上方15厘米，设置成 0.15
# 2.6 cfg.yaml中：map_builder -> dual_antenna_parameter -> is_use_dual_antenna 设置成 false，不使用双天线建图

### ----------------------------------------------------------------------------------------------------------------------------------
### 3.使用多线激光雷达建图 + 双天线RTK
# 3.1 cfg.yaml中：ros_interface -> subscribe_message -> is_use_lidar 设置成 true，其他改成 false
#                ros_interface -> subscribe_message -> is_use_gnss 设置成 true，其他改成 false
# 3.2 cfg.yaml中：directory -> project_directory 功能目录根据需要设置
# 3.3 cfg.yaml中：map_builder -> map_builder_parameter -> lidar_n_scans 目前只支持vlp16雷达，设置成16，其他型号雷达需要调试
# 3.4 cfg.yaml中：map_builder -> utm_localization 根据建图位置设置 utm分区
# 3.5 cfg.gnss_optimization -> gnss_optimization_parameter -> arm_rtk_antennna_in_vlp 设置雷达和rtk天线的高度差值，例如天线在雷达上方15厘米，设置成 0.15
# 3.6 cfg.yaml中：map_builder -> dual_antenna_parameter -> is_use_dual_antenna 设置成 true，使用双天线建图
# 3.7 cfg.yaml中：map_builder -> dual_antenna_parameter -> ranges_main_antenna_to_lidar，设置主天线(定位天线)到雷达之间的距离，单位是米
# 3.8 cfg.yaml中：map_builder -> dual_antenna_parameter -> angle_dual_antenna_to_lidar，设置双天线基线到主天线到雷达连线之间的角度，逆时针为正，顺时针为负，单位是度

