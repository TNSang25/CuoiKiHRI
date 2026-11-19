# bot_gazebo

Gazebo Classic world generation, people plugin and simulation assets.

Package của workspace CuoiKiHRI, tổ chức lại và phát triển từ HuNavSim v2.0.
Thông tin nguồn gốc và giấy phép được ghi ở `THIRD_PARTY.md` tại workspace.
Xem `README.md` của workspace để build và chạy các thành phần cùng nhau.

Người đi bộ dùng `walk.dae` khi di chuyển và `stand.dae` khi đứng yên.
Hai clip COLLADA dùng cùng cấu trúc xương với các skin đi kèm; tránh trộn
những clip BVH cần bộ căn chỉnh xương khác nhau trên cùng một actor.

Khi di chuyển, hướng thân được lấy từ độ dời XY thực tế của chính bước cập
nhật đó. Khi đứng yên, actor giữ hướng nhìn do bộ điều khiển yêu cầu.
Thời gian hoạt ảnh tăng theo quãng đường XY và được hiệu chỉnh từ chuyển
động gốc của clip, nên tốc độ chân thay đổi theo tốc độ đi. Tư thế đứng
được giữ ở clip idle, không bị đóng băng giữa một bước chân.

Chuyển động xoay hông của clip được giữ trong tư thế actor để phần cột sống
cân bằng thân đúng như dữ liệu gốc. Hướng điều hướng được lưu riêng để chuyển
động nhỏ của hông không làm đổi hướng đi hoặc quay mô hình va chạm.

`test_gait_balance` dựng lại chuỗi xương từ file đi/đứng thật và kiểm tra độ
nghiêng ngang của thân. Bật `BUILD_TESTING=ON` khi build để chạy kiểm tra này.

Kiểm tra chuyển động và lệch hướng bằng `ros2 run bot_bringup check_simulation`
sau khi source `scripts/env.bash`; phép kiểm tra yêu cầu sai lệch hướng
ở phân vị 95 nhỏ hơn 5 độ.
