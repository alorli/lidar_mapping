# ndt_slam:
# edited by lichunjing 2020.12.13:


### 1. 编译安装 stlplus3 库
```
cd 3rd_party/stlplus3/
mkdir build
cd build 
cmake ..
make -j8
sudo cp libstlplus.so lidar_mapping/lib
```

### 2. 编译安装 proj-4.4.9 库
```
cd 3rd_party/proj-4.4.9/
sudo chmod 777 ./configure
./configure
make -j8
sudo make install
```

### 3. 编译安装 yaml-cpp 库
# 注意头文件和库文件都需要拷贝到指定目录
```
cd 3rd_party/yaml-cpp-yaml-cpp-0.6.0/
mkdir build 
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make -j8
sudo cp ./libyaml-cpp.so* lidar_mapping/lib/
cd ..
sudo cp ./inclue/* lidar_mapping/include
```
