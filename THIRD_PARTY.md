# Nguồn gốc mã và tài nguyên

Bộ `bot_*` được tổ chức lại và phát triển từ các nguồn sau. Đây là mã phái sinh, không phải toàn bộ thuật toán được viết mới. Các thông báo bản quyền trong mã nguồn và tài nguyên được giữ nguyên.

- [hunav_sim](https://github.com/robotics-upo/hunav_sim.git), commit `35c2d81fc0e8166a3d124558fd718dd03e8a914c`.
- [hunav_gazebo_wrapper](https://github.com/robotics-upo/hunav_gazebo_wrapper.git), commit `566cf2455dd1fc79f1eec41e7ace996d2dd8c0ea`.
- [people](https://github.com/wg-perception/people.git), commit `0ae47f6e0208cedd84d19d066743fdc1d05fcafa`.
- [lightsfm](https://github.com/robotics-upo/lightsfm.git), commit `b30327cca189af2fb90443a5d0040cceb46d7195`.

`bot_agent_control`, `bot_gazebo`, `bot_evaluation`, `bot_rviz_panel` và phần LLM trong `bot_behavior_tools` kế thừa HuNavSim/Gazebo wrapper (MIT).
`bot_social_force` kế thừa các header của lightsfm (BSD-3-Clause).
`bot_interfaces` kế thừa message/service HuNavSim (MIT) và message people_msgs (giấy phép BSD theo package.xml gốc).

Launch, công cụ sinh Behavior Tree cơ bản không dùng LLM, cấu hình dependency cục bộ, cách đóng gói thư viện và các sửa đổi tích hợp thuộc workspace CuoiKiHRI.

Giấy phép của mesh/texture/mô hình bên thứ ba được giữ cùng các thư mục tài nguyên trong `src/bot_gazebo/media/`.
Metadata maintainer dùng email example.com làm chỗ điền; hãy cập nhật trước khi phân phối.
