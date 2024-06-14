## Diff_robot

Robot điều khiển vi sai phát triển trên ROS 2

Gói này chứa các tệp cấu hình, khởi chạy robot.

## :mechanical_arm: Cài đặt

### Nền tảng

- ROS 2: Humble Hawksbill
- OS:
  - Ubuntu 22.04 (WSL2)
  - Ubuntu 22.04 (Trên Raspberry Pi 4B)

### Build from Source

#### Phụ thuộc

1. Cài đặt [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Cài đặt [colcon](https://colcon.readthedocs.io/en/released/user/installation.html)

#### colcon workspace

1. Tạo không gian làm việc

```
mkdir -p ~/dev_ws/src
```

2. Clone các package vào thư mục src

```
cd ~/ws/src
```

```
git clone https://github.com/Huysiunhan02/diff_robot.git
```

3. Install dependencies via `rosdep`

```
cd ~/dev_ws
```

```
rosdep install --from-paths src --ignore-src -i -y
```

4. Build packages

```
colcon build
```

5. Source the built packages

```
source install/setup.bash
```

### Slam

```
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/diff_robot/config/mapper_params_online_async.yaml use_sim_time:=false
```

## :star2: Tham khảo
 * [articubot_one](https://github.com/joshnewans/articubot_one)
 * [andino](https://github.com/Ekumen-OS/andino)
