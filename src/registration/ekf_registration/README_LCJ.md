# ekf_registration:
# edited by lichunjing 2022.11.30:


### 1. 为了和fast-lio时间戳戳判断一致，暂时将 ekf_registration.cc中PrepareMeasurements()函数的如下行大约380行进行替换，测试完成后恢复成正常的
```
if(imu_time > measurements_.lidar_end_time)   //正常代码
if(common::ToUniversalSeconds(imu_time) > common::ToUniversalSeconds(measurements_.lidar_end_time))  //为了和fast-lio时间戳戳判断一致，暂时替换的代码
```
