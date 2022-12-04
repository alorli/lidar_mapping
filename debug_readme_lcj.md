# lidar_mapping:
# edited by lichunjing 2022.12.02:

### 1. map_builder.cc中 void MapBuilder::AddVlpPointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg)函数中，
下面两部分语句是否应该更换顺序，否则SaveVlpRawPcd()函数中保存的每一帧点云数据对应的位姿应该是上一个配准点云的位姿
```
第一部分：
if(is_save_vlp_raw_pcd_)
    {
        SaveVlpRawPcd(timed_id_pointcloud);

        // added by lichunjing 2021-04-14
        // SaveVlpRawPcd(timed_id_pointcloud_compensation);
    }
```
```
第二部分：
ndt_registration_.AddSensorData(timed_id_pointcloud);
```

