# EKF融合GPS和IMU
---

**参考代码：**

[误差状态卡尔曼滤波器(ESKF)融合IMU与GPS数据](https://github.com/zm0612/eskf-gps-imu-fusion)

[基于导航信息的EKF滤波算法实现（附源码）](https://blog.csdn.net/qq_38650944/article/details/123594568?spm=1001.2014.3001.5502)

# 1. 准备数据

IMU数据：时间戳、三轴加速度、三轴角速度

GPS数据：时间戳、经纬高、三轴速度

本项目采用[gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim)生成的数据，数据组织如下：

| 文件 | 内容 | 单位 |
| --- | --- | --- |
| time.csv | IMU时间戳 | 秒(s) |
| accel-0.csv | IMU三轴加速度 | 米/秒^2(m/s^2) |
| gyro-0.csv | IMU三轴角速度 | 度/秒(deg/s) |
| gps_time.csv | GPS时间戳 | 秒(s) |
| gps_0.csv | GPS经纬高、NED三轴速度 | 度(deg)、米/秒(m/s) |
| ref_gps.csv | 真值（GPS经纬高、NED三轴速度） | 度(deg)、米/秒(m/s) |

# 2. 算法原理

扩展卡尔曼滤波针对的是运动或观测方程**非线性**的情况。

## 2.1 预测公式

预测阶段主要包含了**状态量**的预测和关于状态量的**协方差矩阵**的预测。

（1）关于状态量的预测公式为：

$$
\hat{x_k} = F(x_{k-1}, u_k)+Bw_k\tag{1}

$$

其中：

$\hat{x_{k} }$——预测的当前时刻状态量（先验）

$F$—— 状态转移矩阵，表示将k-1时刻的状态量转移至k时刻的状态量

$x_{k-1}$—— 上一时刻经优化后的状态量（后验）

$B$——噪声映射矩阵，将k-1时刻的噪声向量转移到k时刻

$w_k$——陀螺仪和加速度计噪声

（2）关于状态量的协方差矩阵的预测公式为：

$$
\hat{P_k} = FP_{k-1}F^T + BQB^T\tag2
$$

其中：

$\hat{P_k}$——预测的当前时刻关于状态量的协方差矩阵（先验）

$P_{k-1}$——上一时刻状态量的协方差矩阵（后验）

$Q$——噪声的协方差矩阵，描述噪声的不确定性

## 2.2 更新公式

待完善…

# 3. 代码实现

## 3.1 目录结构

```bash
├── CMakeLists.txt
├── README.md
├── app
│   └── main.cpp
├── config
│   └── config.yaml
├── data
│   └── raw_data
│       ├── accel-0.csv
│       ├── gps-0.csv
│       ├── gps_time.csv
│       ├── gyro-0.csv
│       ├── ref_accel.csv
│       ├── ref_att_quat.csv
│       ├── ref_gps.csv
│       ├── ref_gyro.csv
│       ├── ref_pos.csv
│       └── time.csv
├── include
│   ├── config_parameters.h
│   ├── ekf.h
│   ├── fusion_flow.h
│   ├── gps_data.h
│   ├── gps_tool.h
│   ├── imu_data.h
│   ├── imu_tool.h
│   └── utils.h
├── scripts
│   └── vis_raw_data.py
└── src
    ├── ekf.cpp
    ├── fusion_flow.cpp
    ├── gps_tool.cpp
    ├── imu_tool.cpp
    └── utils.cpp
```

## 3.2 代码实现流程

### 3.2.1 IMU和GPS数据定义

`imu_data.h` 

```cpp
#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <Eigen/Dense>

namespace Filter {

class IMUData {
   public:
    IMUData() {}

    double timestamp = 0.0;

    // 三轴线加速度
    Eigen::Vector3d line_acc = Eigen::Vector3d::Zero();

    // 三轴角速度
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
};

}  // namespace Filter

#endif
```

`gps_data.h`

```cpp
#ifndef GPS_DATA_H
#define GPS_DATA_H

#include <Eigen/Dense>

namespace Filter {
class GPSData {
   public:
    GPSData() = default;

    double timestamp = 0.0;

    Eigen::Vector3d lla_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d ned_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d ned_position = Eigen::Vector3d::Zero();

    Eigen::Vector3d true_velocity     = Eigen::Vector3d::Zero();
    Eigen::Vector3d true_lla_position = Eigen::Vector3d::Zero();
};
}  // namespace Filter

#endif
```

### 3.2.2 读取IMU和GPS数据

跳过文件中的第一行（标题行），将数据依次读取到`imu_data_buff`和`gps_data_buff`中。

为了后续数据处理的坐标系统一，需要将GPS的LLA转换到NED坐标系下，转换方法如下：

（1）调用GeographicLib三方库进行转换，由于转换后的结果是在ENU坐标系下，还得将其转换到NED坐标系下。

（2）手动实现：LLA —>ECEF—>ENU—>NED

- 经纬度单位转换至rad：

```cpp
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
```

- LLA转换到ECEF：

```cpp
// WGS-84椭球参数
constexpr double a    = 6378137.0;            // 长半轴（米）
constexpr double f    = 1.0 / 298.257223563;  // 扁率
constexpr double e_sq = f * (2 - f);          // 第一偏心率平方

// LLA转ECEF
Eigen::Vector3d xyz_ecef = Eigen::Vector3d::Zero();
double N                 = a / std::sqrt(1 - e_sq * sin(lat) * sin(lat));
xyz_ecef.x()             = (N + alt) * cos(lat) * cos(lon);
xyz_ecef.y()             = (N + alt) * cos(lat) * sin(lon);
xyz_ecef.z()             = (N * (1 - e_sq) + alt) * sin(lat);
```

- ECEF转换到ENU：

```cpp
constexpr double ref_lat = 31.508183;   // 参考纬度
constexpr double ref_lon = 120.401989;  // 参考经度
constexpr double ref_alt = 0.0;         // 参考海拔高度

// 计算坐标差
double dx = ecef.x() - ref_ecef.x();
double dy = ecef.y() - ref_ecef.y();
double dz = ecef.z() - ref_ecef.z();

// 参考点经纬度弧度
double ref_lat_deg = deg2rad(ref_lat);
double ref_lon_deg = deg2rad(ref_lon);

// 计算ENU坐标
double sin_lat = sin(ref_lat_deg);
double cos_lat = cos(ref_lat_deg);
double sin_lon = sin(ref_lon_deg);
double cos_lon = cos(ref_lon_deg);

double east  = -sin_lon * dx + cos_lon * dy;
double north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
double up    = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
```

- ENU转换到NED：

```cpp
gps_data.ned_position.x() = north;
gps_data.ned_position.y() = east;
gps_data.ned_position.z() = -up;
```

### 3.2.3 IMU和GPS时间同步

目前只采用了简单的时间差是否超过阈值的判断方法，可考虑线性插值。

### 3.2.4 EKF状态量初始化

定义16×1维度的状态向量X_，其包含：位置(3)，速度(3)，四元数(4)，加速度偏置(3)，陀螺仪偏置(3)。

- 位置初始化：gps转换后的NED位置：

```cpp
X_.block<3, 1>(INDEX_STATE_POS, 0) = curr_gps_data.ned_position;
```

- 速度初始化：这里利用真实的速度测量值：

```cpp
X_.block<3, 1>(INDEX_STATE_VEL, 0) = curr_gps_data.true_velocity;
```

- 由于IMU也是在ENU坐标系下的，需要转换到NED坐标系下：

```cpp
// IMU坐标系从右(x)前(y)上(z)转换到前(x)右(y)下(z)：ENU ---> NED
// 原坐标系先绕X轴旋转180°，再绕Y轴旋转0°，最后绕Z轴逆时针旋转90°
Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
```

- 四元数初始化：

```cpp
X_(INDEX_STATE_QUAT + 0, 0) = Q.w();
X_(INDEX_STATE_QUAT + 1, 0) = Q.x();
X_(INDEX_STATE_QUAT + 2, 0) = Q.y();
X_(INDEX_STATE_QUAT + 3, 0) = Q.z();
```

### 3.2.5 EKF预测部分

预测部分的离散化公式：

$$
\hat{x_k} = (Fx*dt)x_{k-1} + (B_k*dt)w_k + g_t*dt\tag3
$$

（1）计算雅可比矩阵$F$

因为状态量是16维的，所以$F$为16×16。$F$是所有行数对状态向量所有分量求一阶偏导，即：

$$
F=
\begin{bmatrix}
I_{3×3} & I_{3×3} & 0_{3×4} & 0_{3×3} & 0_{3×3} \\
0_{3×3} & I_{3×3} & F_{vq} & 0_{3×3} & C^n_b \\
0_{4×3} & 0_{4×3} & F_{qq} & F_{q\xi} & 0_{4×3} \\
0_{3×3} & 0_{3×3} & 0_{3×4} & I_{3×3} & 0_{3×3} \\
0_{3×3} & 0_{3×3} & 0_{3×4} & 0_{3×3} & I_{3×3}
\end{bmatrix}\tag4
$$

$$
F_{vq} = 
\begin{bmatrix}
F_{vq_0} & F_{vq_1} & F_{vq_2} & F_{vq_3} \\
-F_{vq_3} & -F_{vq_2} & F_{vq_1} & F_{vq_0} \\
F_{vq_2} & -F_{vq_3} & -F_{vq_0} & F_{vq_1}
\end{bmatrix}\tag5
$$

$$
F_{vq_0} = 2 * (q_0a_x-q_3a_y+q_2a_z)\\
F_{vq_1} = 2 * (q_1a_x+q_2a_y+q_3a_z)\\
F_{vq_2} = 2 * (-q_2a_x+q_1a_y+q_0a_z)\\
F_{vq_3} = 2 * (-q_3a_x-q_0a_y+q_1a_z)\tag6
$$

$$
F_{qq} =\frac{1}{2}\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y\\ 
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}\tag7
$$

$$
F_{q\xi} =\frac{1}{2}\begin{bmatrix}
-q_1 & -q_2 & -q_3\\
q_0 & -q_3 & q_2\\
q_3 & q_0 & -q_1\\
-q_2 & q_1 & q_0
\end{bmatrix}\tag8
$$

$C^n_b$为IMU到GPS坐标系的姿态变换矩阵（3×3）。

（2）计算雅可比矩阵$B$

$B$是每个状态量对陀螺仪和加速度计求一阶导数，所以$B$为16×6。

$$
B = 
\begin{bmatrix}
0_{3×3} & 0_{3×3}\\
0_{3×3} & C^n_b\\
F_{q\xi} & 0_{4×3}\\
0_{3×3} & 0_{3×3}\\
0_{3×3} & 0_{3×3}
\end{bmatrix}
$$

（3）陀螺仪和加速度计的方差$w$

$w$为6×1的向量。

$$
w = 
\begin{bmatrix}
\omega_{noise}\\
\omega_{noise}\\
\omega_{noise}\\
a_{noise}\\
a_{noise}\\
a_{noise}\\
\end{bmatrix}
$$

（4）加速度向量$g_t$

$g_t$为16×1的向量

$$
g_t =
\begin{bmatrix}
0\\
0\\
-g_{earth}\\
...\\
0
\end{bmatrix}
$$