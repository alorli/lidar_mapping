# ekf_registration:
# edited by lichunjing 2022.11.30:


### 1. 为了和fast-lio时间戳戳判断一致，暂时将 ekf_registration.cc中PrepareMeasurements()函数的如下行大约380行进行替换，测试完成后恢复成正常的
```
if(imu_time > measurements_.lidar_end_time)   //正常代码
if(common::ToUniversalSeconds(imu_time) > common::ToUniversalSeconds(measurements_.lidar_end_time))  //为了和fast-lio时间戳戳判断一致，暂时替换的代码
```

### 2.最后一帧数据
------------------------------------------------------------------------------------------------------------------
---------allframe_id:2170   lidar_points:6104
------update lidar_position: -117.2306288169983190528001  -65.42242360972620929260302 -0.4873737209498899547455153
--------------delta_time:32.34455700000000177851689