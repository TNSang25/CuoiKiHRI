# CuoiKiHRI — Social Navigation cho TurtleBot 4

Robot TurtleBot 4 đi xuyên qua đám đông trong Gazebo Classic. Điều hướng bằng Nav2 MPPI, kết hợp
social costmap và các critic tính đến người. Người được phát hiện chỉ bằng LiDAR (leg tracker).

Môi trường: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.

## Package

| Package | Nội dung |
|---|---|
| `social_nav_msgs` | Message/service: `Agents`, `People`, ... |
| `social_force_model` | Thư viện header Social Force Model |
| `human_agents` | Người mô phỏng: nạp scenario, Behavior Tree + Social Force |
| `human_gazebo` | Plugin Gazebo cho người (actor + chân), world generator, model người |
| `social_nav_levels` | 5 level: world, map, scenario, behavior tree, tool sinh level |
| `tb4_classic_description` | URDF TurtleBot 4 cho Gazebo Classic |
| `people_tracker` | Leg tracker LiDAR (`/people`) và adapter ground truth |
| `social_nav_plugins` | Plugin Nav2: social layer, DynamicHuman / SocialSpeed / PassingSide critic |
| `social_nav_bringup` | Launch, cấu hình Nav2, RViz, ghi kết quả chạy |

## Cài đặt và build

```bash
cd ~/CuoiKiHRI
sudo apt install ros-humble-navigation2 ros-humble-gazebo-ros-pkgs   # nếu chưa có
python3 scripts/prepare_deps.py     # tải dependency còn thiếu vào .deps/ (không cần sudo)
bash scripts/build.bash
```

## Chạy

```bash
bash scripts/run_demo.bash                    # level 1, mở Gazebo + RViz
bash scripts/run_demo.bash world:=level3      # level1 .. level5
```

Robot tự đi tới GOAL của level. Kết quả của mỗi lần chạy được ghi vào `.runtime/results/mission_*.json`.
Dừng bằng `Ctrl+C` hoặc đóng cửa sổ Gazebo.

| Level | Kịch bản |
|---|---|
| 1 | Phòng trống 6×6 m, 1 người đi cắt ngang |
| 2 | Hành lang 2.5 m, 1 người đi ngược chiều |
| 3 | Hành lang chữ L, 2 người (1 người xuất hiện sau góc khuất) |
| 4 | Phòng có bàn ghế, nhóm 3 người đứng và 2 người đi |
| 5 | Sảnh có cửa 1.2 m, 6 người |

Tham số hay dùng:

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `social` | `true` | `false`: chỉ dùng MPPI gốc (baseline) |
| `people_source` | `leg_tracker` | `ground_truth`: lấy vị trí người trực tiếp từ simulator |
| `max_speed` | `0.65` | Tốc độ tối đa của robot [m/s] |
| `humans_yield` | `false` | `true`: người cũng né robot |
| `gui` / `rviz` | `true` | Tắt cửa sổ Gazebo / RViz |
| `auto_start` | `true` | `false`: không tự gửi goal |

Ví dụ: `bash scripts/run_demo.bash world:=level5 social:=false gui:=false`

## Benchmark

```bash
source scripts/env.bash
python3 scripts/run_benchmark.py --levels 1 2 3 --runs 3       # social và baseline, không mở GUI
```

Kết quả nằm trong `.runtime/results/benchmark_*.csv`.

## Sinh lại level / URDF

```bash
python3 src/social_nav_levels/tools/generate_levels.py          # sau khi sửa file mô tả level
source scripts/env.bash && python3 src/tb4_classic_description/tools/make_turtlebot4_urdf.py
bash scripts/build.bash
```

Nguồn gốc mã kế thừa: xem [THIRD_PARTY.md](THIRD_PARTY.md).
