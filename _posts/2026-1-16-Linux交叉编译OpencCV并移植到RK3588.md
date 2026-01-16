# Linux交叉编译OpenCV并移植到RK3588

---

【**需求**】**：**

在PC端调用opencv完成相应功能，然后通过交叉编译将可执行文件、库、头文件等移植到板端运行。

**PC 端 → x86_64主机**

**板端 → RK3588（arm架构）**

# 1. PC端下载GCC交叉编译器

注：针对rk3588，其他arm架构的板子需下载对应的交叉编译工具

- 交叉编译器下载地址：
    
    板端为64位系统：https://releases.linaro.org/components/toolchain/binaries/6.3-2017.05/aarch64-linux-gnu/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu.tar.xz
    
- 解压软件包：

```bash
tar -xvf gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu.tar.xz
```

- 将解压后的gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu放到/opt/目录下

# 2. PC端下载OpenCV源码

- 以Opencv-4.5.4为例

```bash
# 下载OpenCV源码
git clone https://github.com/opencv/opencv.git

# 切换到相应分支（以4.5.4为例）
git checkout 4.5.4
```

- 编写编译脚本`build.sh`

注：编译过程文件放在build_arm目录下，生成的头文件和库安装在install_arm目录下。

```bash
#!/bin/bash
set -e
BUILD_DIR="build_arm"
INSTALL_DIR="install_arm"

export GCC_COMPILER=/opt/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

if command -v ${CC} >/dev/null 2>&1; then
    :
else
    echo "${CC} is not available"
    exit
fi

if [ ! -d $BUILD_DIR ]; then
    mkdir -p $BUILD_DIR
fi

if [ ! -d $INSTALL_DIR ]; then
    mkdir -p $INSTALL_DIR
fi

cd $BUILD_DIR

cmake \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=ARM \
    -DCMAKE_C_COMPILER=${CC} \
    -DCMAKE_CXX_COMPILER=${CXX} \
    -DCMAKE_INSTALL_PREFIX=$(pwd)/../${INSTALL_DIR} \
    -DCMAKE_FIND_ROOT_PATH=$(pwd)/../${INSTALL_DIR} \
    -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY \
    -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY \
    -DBUILD_SHARED_LIBS=ON \
    -DWITH_GTK=OFF \
    -DWITH_JPEG=ON \
    -DWITH_PNG=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DWITH_FFMPEG=OFF \
    ..

echo -e "\nStarting build process...\n"
make -j$(nproc)

echo -e "\nInstalling to ${INSTALL_DIR}...\n"
make install

echo "Finished!"
```

- 执行脚本，等待完成即可

# 3. PC端编写测试代码

- 新建测试文件夹、并完成测试代码编写、编译等。

```bash
mkdir -p opencv_test
cd opencv_test && touch CMakeListst.txt opencv_test.cpp
```

`opencv_test.cpp` ：

```cpp
#include <iostream>

#include <opencv2/opencv.hpp>
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return -1;
    }
    std::string img_path = argv[1];     

    cv::Mat img = cv::imread(img_path);
    if (img.empty())
    {
        std::cerr << "Failed to read image from " << img_path << std::endl;
    }

    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

    std::cout << "Image size: " << gray_img.cols << " x " << gray_img.rows << std::endl;

    return 0;
}
```

`CMakeLists.txt` ：

```bash
cmake_minimum_required(VERSION 3.10)
project(opencv_test)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# 设置为步骤1的路径
set(CROSS_COMPILE /opt/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu)
set(CMAKE_C_COMPILER ${CROSS_COMPILE}-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE}-g++)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# 设置为步骤2编译生成的头文件和库文件路径
set(Opencv_INCLUDE_DIRS /home/cyh/opencv/install_arm/include/opencv4)
set(Opencv_LIB_DIR /home/cyh/opencv/install_arm/lib)
set(Opencv_LIBS opencv_core opencv_imgcodecs opencv_imgproc)

add_executable(opencv_test opencv_test.cpp)
target_include_directories(opencv_test PRIVATE ${Opencv_INCLUDE_DIRS})
target_link_directories(opencv_test PRIVATE ${Opencv_LIB_DIR})
target_link_libraries(opencv_test PRIVATE ${Opencv_LIBS})
```

# 4. 移植到板端

- 在板端新建文件夹：`opencv_test_arm` ；
- 将PC端`opencv_test/build` 目录下的可执行文件和测试图片拷贝到板端`opencv_test_arm` 目录下；
- 将PC端中`opencv/install_arm/lib`和`opencv/install_arm/include/opencv4` 拷贝到板端`opencv_test_arm` 目录下；
- 板端`opencv_test_arm`目录结构：

       — opencv_test_arm

       —— inclued

       —— lib

       —— opencv_test

       —— bus.jpg

- 加载库到环境变量：

```bash
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
```

- 运行可执行文件，Finished！