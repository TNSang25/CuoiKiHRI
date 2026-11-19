# CuoiKiHRI — Bot Human Navigation

Bộ package ROS 2 của workspace CuoiKiHRI, phát triển từ HuNavSim v2.0.
Môi trường đã dùng: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.
Các repo clone cũ đã được thay bằng cấu trúc phẳng dưới `src/`.
Xem [THIRD_PARTY.md](THIRD_PARTY.md) về phần mã/tài nguyên kế thừa.

Đã kiểm tra ngày 19/11/2026: build sạch đủ 8 package; chạy cảnh mặc định với
3 người; chạy thêm 6 tác nhân dùng cây tự sinh cho cả 6 kiểu hành vi; nạp panel
RViz/bản đồ; gọi service đánh giá và xuất CSV (33 cột, không thiếu giá trị).
Kết quả kiểm tra lưu trong `.runtime/smoke-test.json`,
`.runtime/six-behaviors-test.json` và `.runtime/evaluation-test.json`.

| Package | Chức năng chính |
|---|---|
| `bot_interfaces` | Message và service cho người, robot, hành vi, đánh giá; bao gồm kiểu People/Person. |
| `bot_social_force` | Thư viện header Social Force Model, xuất dependency qua ament. |
| `bot_agent_control` | Nạp scenario, thực thi Behavior Tree, tính chuyển động và xuất trạng thái. |
| `bot_gazebo` | Plugin người đi bộ, sinh world và tài nguyên mô phỏng. |
| `bot_evaluation` | Thu thập dữ liệu, tính chỉ số và xuất kết quả. |
| `bot_rviz_panel` | Giao diện sửa người, mục tiêu, Behavior Tree và chọn chỉ số. |
| `bot_behavior_tools` | Sinh Behavior Tree cơ bản từ YAML; hỗ trợ thêm công cụ LLM tùy cấu hình. |
| `bot_bringup` | Launch toàn hệ thống và kiểm tra chuyển động thực tế. |

## Chuẩn bị và build

```bash
cd /home/ngocsang/CuoiKiHRI
python3 scripts/prepare_deps.py
bash scripts/build.bash
```

`prepare_deps.py` tải và giải nén các gói ROS/Gazebo còn thiếu vào `.deps/`,
không cần sudo. Không còn dependency vào repo hay package `hunav_*`,
`people_msgs`, hoặc đường dẫn header `/usr/local/include/lightsfm`.

## Chạy mô phỏng

```bash
bash scripts/run_demo.bash
```

Mặc định mở cảnh kho hàng với 3 người đi bộ và một robot hình trụ màu xanh
đứng yên. Các phản ứng/hành vi được điều khiển bằng Behavior Tree. Evaluator
được mở cùng hệ thống nhưng chỉ ghi dữ liệu khi gọi service bắt đầu.
Đóng Gazebo hoặc nhấn `Ctrl+C` để dừng.

```bash
# Chạy không mở cửa sổ
bash scripts/run_demo.bash gui:=false

# Cảnh nhà
bash scripts/run_demo.bash world:=house

# Tắt evaluator
bash scripts/run_demo.bash evaluation:=false
```

Nếu chạy bằng ros2 trực tiếp, nạp môi trường của workspace trước:

```bash
source scripts/env.bash
ros2 launch bot_bringup simulation.launch.py
```

Môi trường mặc định dùng ROS domain 65 và Gazebo master `localhost:11365`.
Đặt `BOT_ROS_DOMAIN_ID`/`BOT_GAZEBO_MASTER_URI` trước khi source env để đổi.
Không chạy hai instance trên cùng ROS domain và Gazebo master.

## Chỉnh scenario và Behavior Tree

Scenario: `src/bot_gazebo/scenarios/agents_warehouse.yaml`.
Khóa node YAML là `bot_agent_loader`; file XML tương ứng nằm trong
`src/bot_gazebo/behavior_trees/` theo mẫu `{yaml_base_name}__agent_{id}_bt.xml`.

```bash
source scripts/env.bash
ros2 launch bot_rviz_panel panel.launch.py
```

Sinh các cây hành vi cơ bản mà không cần LLM:

```bash
ros2 run bot_behavior_tools generate_trees \
  src/bot_gazebo/scenarios/agents_warehouse.yaml \
  --output-dir .runtime/generated_trees

bash scripts/run_demo.bash behavior_tree_directory:="$PWD/.runtime/generated_trees"
```

Công cụ này hỗ trợ Regular, Impassive, Surprised, Curious, Scared, Threatening.
Nó không ghi đè XML có sẵn nếu chưa có `--force`.
Để dùng `generator_cli` dựa trên LLM, cài thư viện Python `openai` và cấu hình
`BOT_LLM_API_KEY`, `BOT_LLM_BASE_URL`, `BOT_LLM_MODEL`; phần này không tự chạy
khi khởi động mô phỏng.

## Kiểm tra và đánh giá

Trong terminal khác khi mô phỏng đang chạy:

```bash
source scripts/env.bash
ros2 run bot_bringup check_simulation
ros2 topic list --no-daemon
```

Phép kiểm tra xác nhận có `/clock`, `/human_states`, `/robot_states` và cả 3
người đổi vị trí hơn 0,1 m trong khoảng 10 giây. Kết quả:
`.runtime/smoke-test.json`. Dùng `--agents` để kiểm tra scenario có số người khác.

Evaluator dùng các service `/bot_start_recording` (`bot_interfaces/srv/StartEvaluation`)
và `/bot_stop_recording` (`std_srvs/srv/Empty`). Cung cấp `robot_goal` khi đo
các chỉ số liên quan đến đích. Kết quả CSV nằm trong `.runtime/results/`;
cấu hình chỉ số tại `src/bot_evaluation/config/metrics.yaml`.

World sinh ra và log ROS nằm trong `.runtime/`; log build nằm trong `log/`.
Demo hiện dùng robot đứng yên, chưa tích hợp robot tự điều hướng bằng Nav2.
Các wrapper Gazebo Fortress, Isaac Sim, Webots và robot PMB2 không thuộc bộ package này.
