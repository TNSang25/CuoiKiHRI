# Nguồn gốc mã và tài nguyên

Các package trong `src/` được tổ chức lại và phát triển từ các nguồn sau. Đây là mã phái sinh, không phải toàn bộ thuật toán được viết mới. Các thông báo bản quyền trong mã nguồn và tài nguyên được giữ nguyên.

- [hunav_sim](https://github.com/robotics-upo/hunav_sim.git), commit `35c2d81fc0e8166a3d124558fd718dd03e8a914c`.
- [hunav_gazebo_wrapper](https://github.com/robotics-upo/hunav_gazebo_wrapper.git), commit `566cf2455dd1fc79f1eec41e7ace996d2dd8c0ea`.
- [people](https://github.com/wg-perception/people.git), commit `0ae47f6e0208cedd84d19d066743fdc1d05fcafa`.
- [lightsfm](https://github.com/robotics-upo/lightsfm.git), commit `b30327cca189af2fb90443a5d0040cceb46d7195`.

`human_agents` và `human_gazebo` kế thừa HuNavSim/Gazebo wrapper (MIT).
`social_force_model` kế thừa các header của lightsfm (BSD-3-Clause).
`social_nav_msgs` kế thừa message/service HuNavSim (MIT) và message people_msgs (giấy phép BSD theo package.xml gốc).

`social_nav_plugins`, `people_tracker`, `social_nav_levels`, `social_nav_bringup`, công cụ sinh URDF TurtleBot 4, cấu hình dependency cục bộ và các sửa đổi tích hợp thuộc workspace CuoiKiHRI. URDF TurtleBot 4 sinh từ `turtlebot4_description` / `irobot_create_description` (Apache-2.0).

Giấy phép của mesh/texture/mô hình bên thứ ba được giữ cùng các thư mục tài nguyên trong `src/human_gazebo/models/`.
Metadata maintainer dùng email example.com làm chỗ điền; hãy cập nhật trước khi phân phối.
