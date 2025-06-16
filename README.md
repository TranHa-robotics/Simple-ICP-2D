# Simple-ICP-2D

Yêu cầu cần có thư viện Eigen, ros2 humble
Cài đặt:
Nếu sử dụng ros
`cd ros2_ws/src`
`git clone git@github.com:TranHa-robotics/Simple-ICP-2D.git`
`cd ..`
`colcon build --packages-select icp_2d`
`source install/setup.bash`
`ros2 run icp_2d main`

Compile không cần ros: 
`cd icp_2d/src/`
`g++ src/main.cpp -I/usr/include/eigen3 -o icp -std=c++17`
`./icp`

Kết quả

![image](https://github.com/user-attachments/assets/4b0288de-e483-47e4-ba61-1aeef1806441)
