# lidar_mapping:
# edited by lichunjing 2021.02.28:

### 1. 安装依赖库
依赖项主要有：
pcl
g2o
Sophus
stlplus3
proj-4.4.9
yaml-cpp


### 2. 安装 g2o-old
# 2.1 安装依赖项：
```
sudo apt-get install libsuitesparse-dev
sudo apt-get install qtdeclarative5-dev
sudo apt-get install qt5-qmake
sudo apt-get install libqglviewer2-qt5
```

# 2.2 编译安装：
```
mkdir ~/INSTALL_LCJ
cd lidar_mapping
cp 3rd_party/g2o-old ~/INSTALL_LCJ/
cd ~/INSTALL_LCJ/g2o-old/
mkdir build
cd build
cmake ..
make -j12
sudo make install
```

### 3. 安装 Sophus
# 3.1 编译安装：
```
mkdir ~/INSTALL_LCJ
cd lidar_mapping
cp 3rd_party/Sophus ~/INSTALL_LCJ/
cd ~/INSTALL_LCJ/Sophus/
mkdir build
cd build
cmake ..
make -j12
sudo make install
```
