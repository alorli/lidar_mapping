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
sudo apt-get install libqglviewer-dev
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
# 3.2 编译时可能出现如下错误：
```
/home/alorli/INSTALL_LCJ/Sophus/sophus/so2.cpp:32:26: error: lvalue required as left operand of assignment
   unit_complex_.real() = 1.;
                          ^~
/home/alorli/INSTALL_LCJ/Sophus/sophus/so2.cpp:33:26: error: lvalue required as left operand of assignment
   unit_complex_.imag() = 0.;
                          ^~
```
# 解决方法如下，打开 Sophus/sophus/so2.cpp文件，32，33行改成如下形式，重新编译：
```
  unit_complex_.real(1.0);
  unit_complex_.imag(0.0);
```


### 4. 编译安装 stlplus3 库
```
cd 3rd_party/stlplus3/
mkdir build
cd build 
cmake ..
make -j8
sudo cp libstlplus.so lidar_mapping/lib
```

### 5. 编译安装 proj-4.4.9 库
```
cd 3rd_party/proj-4.4.9/
sudo chmod 777 ./configure
./configure --build=arm-linux --host=arm-linux --target=arm-linux
make -j8
sudo make install
```
# 如果有在其他架构编译.o和.lo的中间文件，首先进入 3rd_party/proj-4.4.9/src目录，删中间文件后，重新执行configure和后面的命令即可
```
cd 3rd_party/proj-4.4.9/src
sudo rm -rf ./*.o ./*.so
./configure --build=arm-linux --host=arm-linux --target=arm-linux
make -j8
sudo make install
```
# 最后将生成的库文件拷贝到 lidar_mapping/lib目录下
```
cd /usr/local/lib
sudo cp ./libproj.so* lidar_mapping/lib/
```

### 6. 编译安装 yaml-cpp 库
# 注意头文件和库文件都需要拷贝到指定目录
```
cd 3rd_party/yaml-cpp-yaml-cpp-0.6.0/
mkdir build 
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make -j8
sudo cp ./libyaml-cpp.so* lidar_mapping/lib/
cd ..
sudo cp -r ./inclue/* lidar_mapping/include
```

### 7. 编译库版本
# CMakeLists_lib.txt 名字改成 CMakeLists.txt 并将 add_library，编译库指令打开，编译库
# 编译好的库放在 devel/lib目录下，将对应的库拷贝到 lidar_mapping/lib 目录下重新编译即可

### 7. 其他问题
# Linux中 error while loading shared libraries错误解决办法
# 默认情况下，编译器只会使用/lib和/usr/lib这两个目录下的库文件，通常通过源码包进行安装时，如果不指定--prefix，会将库安装在/usr/local/lib目录下；
# 当运行程序需要链接动态库时，提示找不到相关的.so库，会报错。也就是说，/usr/local/lib目录不在系统默认的库搜索目录中，需要将目录"/usr/local/lib"加进去。
```
sudo gedit /etc/ld.so.conf
ldconfig
```

