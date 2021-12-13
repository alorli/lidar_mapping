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

# 2.3 如果有在其他架构编译.o和.lo的中间文件，首先进入 3rd_party/proj-4.4.9/src目录，删中间文件后，重新执行configure和后面的命令即可
# 
```
cd 3rd_party/proj-4.4.9/src
sudo rm -rf ./*.o ./*.so ./*.la ./*.a
./configure --enable-shared=yes --enable-static=no 
make -j8
sudo make install
```
# 最后将生成的库文件拷贝到 lidar_mapping/lib目录下
```
cd /usr/local/lib
sudo cp ./libproj.so* lidar_mapping/lib/
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

### 4. 修改CMakeLists.txt文件
# 将下面语句屏蔽掉
```
 #Compile as C++11, supported in ROS Kinetic and newer
  add_compile_options(-std=c++11)
```
# 替换成下面的语句
```
 #Compile as C++14, supported in ROS neotic and newer
  add_compile_options(-std=c++14)
```

# 增加下面的语句
```
# added by lichunjing 2021-03-01
SET(ACCEPT_USE_OF_DEPRECATED_PROJ_API_H "/usr/local/include/")
```

### 5. 3rd_party/graph/vertex_and_edge.cpp 中的 bool VertexSE3LieAlgebra::read(Eigen::Vector3f t, Eigen::Quaternionf q) 函数要加返回值
# 增加如下返回值语句
```
	// added by lichunjing 2021-03-05
	// C++14 必须要要有返回值
	return true;			
```

### 6. 其他问题
# Linux中 error while loading shared libraries错误解决办法
# 默认情况下，编译器只会使用/lib和/usr/lib这两个目录下的库文件，通常通过源码包进行安装时，如果不指定--prefix，会将库安装在/usr/local/lib目录下；
# 当运行程序需要链接动态库时，提示找不到相关的.so库，会报错。也就是说，/usr/local/lib目录不在系统默认的库搜索目录中，需要将目录"/usr/local/lib"加进去。
```
sudo gedit /etc/ld.so.conf
ldconfig
```
