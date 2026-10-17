// px4 offboard_control 
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

// sensor 
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//tf2 
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// vision 
#include <cannyedge_test1_interfaces/msg/hole_info.hpp> // Add Vision Team Camera msg

//rviz 시각화
#include <visualization_msgs/msg/marker.hpp>
//KD 트리 추가 
#include <pcl/kdtree/kdtree_flann.h> 
//occupancy grid
#include <nav_msgs/msg/occupancy_grid.hpp>

// ros 
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <array>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("planning")
  {
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    //rviz 시각화
    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planned_path_marker", 1);
    waypoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/waypoint_markers", 1);
    rrt_nodes_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_nodes_marker", 1); // RRT 모든 노드 시각화

    rclcpp::SensorDataQoS qos;
    local_position_subscription_ = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
        {
          current_position_[0] = msg->position[0];
          current_position_[1] = msg->position[1];
          current_position_[2] = msg->position[2];

   

          float dx = current_position_[0] + 0.0f;
          float dz = current_position_[2] + 1.0f;
          float dist = std::sqrt(dx * dx + dz * dz);
          if (dist < 0.1f && !reached_target_position_)
          {
            reached_target_position_ = true;
            RCLCPP_INFO(this->get_logger(), "Setpoint 위치 도달 및 정렬 시작!");
          }
          std::array<float, 3> obstacle_position_= {current_position_[1], current_position_[0],current_position_[2]};
          auto map_pos = transform_pose("odom", "map", obstacle_position_);
            map_position_ = map_pos;

            if (!path_ready_) {
                std::pair<float, float> current_xy = {map_position_[0], map_position_[1]};
                planned_path_ = generate_path(current_xy, obstacle_goal_position_);  // 경로 재계획
                publish_path_marker(planned_path_); //rviz 시각화
                publish_waypoints_marker(planned_path_);
                path_ready_ = true;

                if (planned_path_.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "초기 경로 생성 실패! 실행을 중단합니다.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "초기 경로 생성 완료. 웨이포인트 수: %lu", planned_path_.size());
                }
            }
        });
    
    //장애물 포인트 클라우드 받아오기 
    cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/filtering_map", rclcpp::SensorDataQoS(),
        std::bind(&OffboardControl::cloud_callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
    //occupancy grid 구독
    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, 
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            occupancy_grid_ = msg;
            map_update_count_++;

            if (move_done_) {
                RCLCPP_INFO(this->get_logger(), "✅ 회피 완료 상태이므로 더 이상 경로를 생성하지 않습니다.");
                return;
            }
             // 맵 수신 시 debounce 타이머가 작동 중이 아니라면 타이머 시작 짧은시간에 맵이 여러번 들어오는 걸 방지하기 위해 일정시간 후 한번만 경로를 생성하기 위해
            if (!debounce_active_) {
                debounce_active_ = true;
                debounce_triggered_ = true;

            debounce_timer_ = this->create_wall_timer(
              1s,
              [this]() {
                  if (debounce_triggered_) {

                      std::pair<float, float> current_xy = {map_position_[0], map_position_[1]};
                      auto new_path = generate_path(current_xy, obstacle_goal_position_);

                      float new_path_length = 0.0;
                      for (size_t i = 1; i < new_path.size(); ++i) {
                          float dx = new_path[i].first - new_path[i - 1].first;
                          float dy = new_path[i].second - new_path[i - 1].second;
                          new_path_length += std::hypot(dx, dy);
                      }

                      if (!first_path_generated_) {
                          // 첫 경로는 무조건 채택
                          planned_path_ = new_path;
                          waypoint_idx_ = 0;
                          move_done_ = false;
                          path_ready_ = true;
                          first_path_generated_ = true;
                          // 경로 생성 시작 시간 기록
                          move_start_time_ = this->now();
                          move_timer_started_ = true;

                          RCLCPP_INFO(this->get_logger(), "✅ 첫 경로 생성 (%.2fm)", new_path_length);
                          publish_path_marker(planned_path_);
                          publish_waypoints_marker(planned_path_);
                      } else {
                          // 이후부터는 기존 경로와 비교
                          float remaining_path_length = 0.0;
                          if (!planned_path_.empty() && waypoint_idx_ < planned_path_.size()) {
                              float dx = planned_path_[waypoint_idx_].first - map_position_[0];
                              float dy = planned_path_[waypoint_idx_].second - map_position_[1];
                              remaining_path_length += std::hypot(dx, dy);

                              for (size_t i = waypoint_idx_ + 1; i < planned_path_.size(); ++i) {
                                  float dx = planned_path_[i].first - planned_path_[i - 1].first;
                                  float dy = planned_path_[i].second - planned_path_[i - 1].second;
                                  remaining_path_length += std::hypot(dx, dy);
                              }
                          }

                          if (new_path_length < remaining_path_length) {
                              planned_path_ = new_path;
                              waypoint_idx_ = 0;
                              move_done_ = false;
                              path_ready_ = true;

                              RCLCPP_INFO(this->get_logger(), "☑️ 짧은 새 경로 채택 (%.2fm < 기존 남은 %.2fm)", new_path_length, remaining_path_length);
                              publish_path_marker(planned_path_);
                              publish_waypoints_marker(planned_path_);
                          } else {
                              RCLCPP_WARN(this->get_logger(), "❌ 새 경로 무시 (%.2fm ≥ 기존 남은 %.2fm)", new_path_length, remaining_path_length);
                          }
                      }

                      debounce_triggered_ = false;
                  }

                  debounce_timer_->cancel();
                  debounce_active_ = false;
              });

                RCLCPP_INFO(this->get_logger(), "🗺️ 새로운 맵 수신됨. 디바운싱 타이머 시작 (1초 후 경로 재계획)");

            } else {
                // 타이머가 이미 돌고 있으면, 다음 실행을 한 번만 하도록 플래그만 켬
                debounce_triggered_ = true;
                RCLCPP_INFO(this->get_logger(), "⏳ 디바운싱 중... (최신 맵 저장됨)");
            }
        });

    //occupancy 장애물 표시 시각화
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle_marker", 10);
    occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);    
    // 노드가 시작된 시점(첫 번째 run) 저장
    start_time_ = this->now();

    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
  }

    void arm();
    void disarm();
    void land();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr local_position_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<cannyedge_test1_interfaces::msg::HoleInfo>::SharedPtr hole_info_subscription_;
  //rtviz 시각화
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_nodes_marker_pub_;
  //kd트리
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_raw_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr cloud_kdtree_;
  //occupancy grid 생성 및 장애물 처리
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
  //occupancy 장애물 표시 시각화
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;  

  
  // 필요 변수 선언 
  // 장애물 회피 변수 선언 
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::array<float, 3> map_position_ = {0.0, 0.0, 0.0};
  std::array<float, 3> current_position_{0.0, 0.0, 0.0};
  std::array<float, 3> goal_position_{-2.0f,6.5f,-1.0f};
  std::vector<std::pair<float, float>> planned_path_;
  size_t waypoint_idx_ = 0;
  bool path_ready_ = false;
  bool move_done_ = false;
  const std::pair<float, float> obstacle_goal_position_ = {5.0f, 0.0f};  // 목적지 좌표 
  rclcpp::Time start_time_;            // 비행 시작 시점을 저장할 변수
  bool second_path_generated_ = false; // 15초 이후 한 번만 재계획이 일어나도록 할 플래그
  int pointcloud_received_count_ = 0;  // 멤버 변수 추가
  bool map_ready_ = false;      
  bool already_landed_ = false;

  // 맵 주기 설정 
  rclcpp::TimerBase::SharedPtr debounce_timer_;
  bool debounce_active_ = false;
  bool debounce_triggered_ = false;
  int map_update_count_ = 0;  // 맵이 몇 번 들어왔는지 추적
  bool first_path_generated_ = false; // 첫 맵 받은 경로는 무조건 따라가
  // 웨이포인트개수 기반 간단 경로 길이 추정
  float waypoint_spacing_ = 0.1f;  // 웨이포인트 간 거리 (고정)
  int reached_waypoint_count_ = 0; // 실제 도달한 웨이포인트 개수
  bool distance_printed_ = false;
  // 시간 관련 변수
  rclcpp::Time move_start_time_;
  rclcpp::Time move_end_time_;
  bool move_timer_started_ = false;
  // 구멍 통과 변수 선언 
  // cannyedge_test1_interfaces::msg::HoleInfo hole_info_msg_;
  bool reached_target_position_ = false;
  // int current_case_ = 0;
  // float current_yaw = 1.57f;
  // float move_x = current_position_[0];
  // bool move_right = true;
  // int movement_step_count = 0;
  // bool before_pass = false;
  uint64_t offboard_setpoint_counter_ = 0;
  // float pass_x;
  // float pass_z;

  
  std::array<float, 3> transform_pose(const std::string& from, const std::string& to, const std::array<float, 3>& pos) {
      geometry_msgs::msg::PoseStamped in, out;
      in.header.stamp = rclcpp::Time(0);
      in.header.frame_id = from;
      in.pose.position.x = pos[0];
      in.pose.position.y = pos[1];
      in.pose.position.z = pos[2];
      in.pose.orientation.w = 1.0;

      if (!tf_buffer_->canTransform(to, from, tf2::TimePointZero, tf2::durationFromSec(0.5))) {
          RCLCPP_WARN(this->get_logger(), "TF (%s -> %s) 아직 준비되지 않음", from.c_str(), to.c_str());
          return pos;
      }

      try {
          out = tf_buffer_->transform(in, to);
          return {
              static_cast<float>(out.pose.position.x),
              static_cast<float>(out.pose.position.y),
              static_cast<float>(out.pose.position.z)
          };
      } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "TF 변환 실패 (%s -> %s): %s", from.c_str(), to.c_str(), ex.what());
         return pos;
      }
  }

  //보간 함수 추가
  std::vector<std::pair<float, float>> interpolate_path(
      const std::vector<std::pair<float, float>>& original_path,
      float interval) {

      std::vector<std::pair<float, float>> interpolated;

      if (original_path.size() < 2) return original_path;

      for (size_t i = 0; i < original_path.size() - 1; ++i) {
          auto [x0, y0] = original_path[i];
          auto [x1, y1] = original_path[i + 1];

          float dx = x1 - x0;
          float dy = y1 - y0;
          float dist = std::hypot(dx, dy);

          int steps = std::max(1, static_cast<int>(dist / interval));

          for (int j = 0; j < steps; ++j) {
              float t = static_cast<float>(j) / steps;
              float xi = x0 + t * dx;
              float yi = y0 + t * dy;
              interpolated.emplace_back(xi, yi);
          }
      }

      // 마지막 점은 항상 추가
      interpolated.emplace_back(original_path.back());

      return interpolated;
  }  
  
  void timer_callback()
{
    publish_path_marker(planned_path_);
    publish_waypoints_marker(planned_path_);

    if (offboard_setpoint_counter_ == 10) {
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        this->arm();
    }

    publish_offboard_control_mode();

    if (!move_done_) {
        obstacle_avoidance();  // 장애물 회피만 수행
    }
    else if (!already_landed_) {
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0); // 착륙 명령
        already_landed_ = true;
        RCLCPP_INFO(this->get_logger(), " 착륙 명령 전송 완료");
    }

    if (offboard_setpoint_counter_ < 11)
        offboard_setpoint_counter_++;
}

  
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
  {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
  }

  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
  }


  void publish_trajectory_setpoint(float x, float y, float z, float yaw) // setpoint x, y, z, yaw
  {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

  //------------------------------------------------------------------------------------------
  void publish_final_setpoint(float x, float y, float z) // setpoint x, y, z, yaw
  {
    TrajectorySetpoint msg{};
    msg.position[0] = y;
    msg.position[1] = x;
    msg.position[2] = z;
    msg.yaw = 1.57f;
    trajectory_setpoint_publisher_->publish(msg);
  }

  void obstacle_avoidance() 
  { 
    auto [target_x, target_y] = planned_path_[waypoint_idx_];
    float target_z = -1.0f; // 목표 고도 설정 ( 1.5m)

    //RCLCPP_INFO(this->get_logger(), "🧭 [경로 좌표] map 기준: x=%.2f, y=%.2f", target_x, target_y);

    auto odom_target = transform_pose("map", "odom", {target_x, target_y, target_z});
    //RCLCPP_INFO(this->get_logger(), "🧭 [변환 좌표] odom 기준: x=%.2f, y=%.2f, z=%.2f", 
    //            odom_target[0], odom_target[1], odom_target[2]);
    float dx = target_x - map_position_[0];
    float dy = target_y - map_position_[1];
    float dz = target_z - map_position_[2];

    if (std::hypot(dx, dy) < 0.15 && std::fabs(dz) < 0.15) {  // 웨이포인트에 도달했는지 확인
        if (waypoint_idx_ < planned_path_.size() - 1) {
            waypoint_idx_++;
            reached_waypoint_count_++;
        } else {
            move_done_ = true;
            RCLCPP_INFO(this->get_logger(), "장애물 회피 완료");
            // 웨이포인트 개수 기반 이동 거리 계산
            float traveled_distance = reached_waypoint_count_ * waypoint_spacing_;
            RCLCPP_INFO(this->get_logger(), "📏📏📏 최종 이동 거리: %.2f m (웨이포인트 %d개)", 
                        traveled_distance, reached_waypoint_count_);
            distance_printed_ = true;
            // 장애물 회피 시간 측정
            if (move_timer_started_) {
            move_end_time_ = this->now();
            rclcpp::Duration elapsed = move_end_time_ - move_start_time_;
            RCLCPP_INFO(this->get_logger(), "⏱️⏱️⏱️ 장애물 회피 시간: %.2f 초", elapsed.seconds());
            move_timer_started_ = false;

            // ✅ 로그 파일로 저장
            std::ofstream logfile("/home/rth/Mygit/planning_logs.txt", std::ios::app);  // 경로는 자유롭게 변경 가능
            if (logfile.is_open()) {
                std::time_t now = std::time(nullptr);
                logfile << "==== [로그 기록] ====\n";
                logfile << "🕒 시간: " << std::asctime(std::localtime(&now));
                
                logfile << "📏 최종 이동 거리: " << traveled_distance << " m (" << reached_waypoint_count_ << "개 웨이포인트)\n";
                logfile << "⏱ 장애물 회피 시간: " << elapsed.seconds() << " 초\n";
                logfile << "======================\n\n";
                logfile.close();
            }
        }
          //current_case_ = -1; //다음 케이스 진행 안 함 장애물 회피만
          return; 
          }
    }
    //------------------------------------------------------------------------------
    publish_final_setpoint(odom_target[0], odom_target[1], odom_target[2]); // 목표 위치로 setpoint 발행
  }
  
  
  // 장애물 회피 관련 
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        //원하는 고도 필터링 enu기준 고도 1.2~1.7
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : pcl_cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
            if (pt.z >= 0.3 && pt.z <= 1.5){
                filtered_cloud->points.emplace_back(pt);
            }
        }

        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;

        cloud_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        cloud_kdtree_->setInputCloud(filtered_cloud);
        pointcloud_raw_ = filtered_cloud;

        // 1. OccupancyGrid 생성 파라미터 정의
        float resolution = 0.05; // 5cm 셀 크기
        int width = 400; // 10m / 0.05m
        int height = 400;
        float origin_x = -10.0;
        float origin_y = -10.0;

        std::vector<int8_t> data(width * height, 0);
        // 2. 포인트 클라우드를 OccupancyGrid에 반영
        // 팽창 반경: 0.05m × 4 = 0.2m (총 0.4m 직경 영역)
        //int inflation_cells = 8; 

        float drone_radius = 0.3;       // 드론 반지름 (m)
        float safety_margin = 0.1;       // 추가 여유 (m)
        int inflation_cells = static_cast<int>((drone_radius + safety_margin) / resolution);

        for (const auto& pt : filtered_cloud->points) {
            int mx = static_cast<int>((pt.x - origin_x) / resolution);
            int my = static_cast<int>((pt.y - origin_y) / resolution);

            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    int nx = mx + dx;
                    int ny = my + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int idx = ny * width + nx;
                        data[idx] = 100;
                    }
                }
            }
        }
        // 3. OccupancyGrid 메시지 생성
        nav_msgs::msg::OccupancyGrid map;
        map.header.stamp = this->now();
        map.header.frame_id = "map";
        map.info.resolution = resolution;
        map.info.width = width;
        map.info.height = height;
        map.info.origin.position.x = origin_x;
        map.info.origin.position.y = origin_y;
        map.info.origin.orientation.w = 1.0;
        map.data = data;

        pointcloud_received_count_++;
        if (pointcloud_received_count_ >= 2) {
            // OccupancyGrid 퍼블리시
            occupancy_pub_->publish(map);
            map_ready_ = true;
            pointcloud_received_count_ = 0;  // 리셋
        }

        // 4. 맵 퍼블리시 & Marker 시각화
        occupancy_pub_->publish(map);
        publish_obstacle_marker(map, marker_pub_);
    }

    bool is_occupied(float x, float y) {

            if (!occupancy_grid_) return false;

            const auto& info = occupancy_grid_->info;
            int mx = static_cast<int>((x - info.origin.position.x) / info.resolution);
            int my = static_cast<int>((y - info.origin.position.y) / info.resolution);

            // 맵 바깥은 무조건 점유로 간주
            if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height))
                return true;

            int index = my * info.width + mx;
            int8_t value = occupancy_grid_->data[index];

            // 점유된 셀(> 50) 또는 미탐지 셀(-1)인 경우 장애물로 간주
            return (value == -1 || value > 50);
    }

    //rviz 생성 경로 시각화
        void publish_path_marker(const std::vector<std::pair<float, float>>& path) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "planned_path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        for (const auto& p : path) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = 0.8;  // 경로의 고도
            marker.points.push_back(pt);
        } 
        path_marker_pub_->publish(marker);
    }
    // waypoint 시각화
        void publish_waypoints_marker(const std::vector<std::pair<float, float>>& path) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "waypoints";
        marker.id = 1;  // path marker와 다른 ID여야 함
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        for (const auto& p : path) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = 0.8;
            marker.points.push_back(pt);
        }
        waypoint_marker_pub_->publish(marker);
    }

    // RRT 모든 노드 시각화
        void publish_rrt_nodes_marker(const std::vector<std::pair<float, float>>& node_positions) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "rrt_nodes";
        marker.id = 3;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        for (const auto& p : node_positions) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = 0.8;  // z 고도는 적당히 설정
            marker.points.push_back(pt);
        }
        rrt_nodes_marker_pub_->publish(marker);
    }
    void publish_selected_rrt_nodes_marker(const std::vector<std::pair<float, float>>& selected_rrt_nodes) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "rrt_selected_nodes";  // 네임스페이스 다르게!
        marker.id = 4;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = 0.0;
        marker.color.g = 1.0;  // ✅ 초록색 (선택된 노드)
        marker.color.b = 0.0;
        marker.color.a = 0.7;

        for (const auto& p : selected_rrt_nodes) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = 1.0;  // 선택된 노드는 더 높게 표시
            marker.points.push_back(pt);
        }
        rrt_nodes_marker_pub_->publish(marker);
    }

    // 장애물 마커 시각화
    void publish_obstacle_marker(const nav_msgs::msg::OccupancyGrid &map_msg,
                             const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &marker_pub)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "occupied_cells";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = map_msg.info.resolution;
        marker.scale.y = map_msg.info.resolution;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.8;

        for (unsigned int y = 0; y < map_msg.info.height; ++y) {
            for (int x = 0; x < map_msg.info.width; ++x) {
                int index = y * map_msg.info.width + x;
                if (map_msg.data[index] == 100) {
                    geometry_msgs::msg::Point p;
                    p.x = x * map_msg.info.resolution + map_msg.info.origin.position.x + map_msg.info.resolution / 2.0;
                    p.y = y * map_msg.info.resolution + map_msg.info.origin.position.y + map_msg.info.resolution / 2.0;
                    p.z = 0.8;
                    marker.points.push_back(p);
                }
            }
        }
        marker_pub->publish(marker);
    }


    // RRT 경로 생성
    std::vector<std::pair<float, float>> generate_path(const std::pair<float, float>& start,
                                                    const std::pair<float, float>& goal) {
        // 경로 생성 시작 시간 측정
        auto start_time = std::chrono::high_resolution_clock::now(); 
        struct RRTNode {
            float x, y;  // 랜덤한 샘플 뽑기 
            int parent_idx;  
            RRTNode(float _x, float _y, int _parent_idx)
                : x(_x), y(_y), parent_idx(_parent_idx) {}
        };

        std::vector<std::pair<float, float>> path;
        std::vector<RRTNode> nodes;
        std::vector<float> cost; // 비용 계산을 위한 벡터
        nodes.emplace_back(start.first, start.second, -1);
        cost.push_back(0.0f);  // 시작 노드의 cost는 0

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(-1.0, 5.5);
        std::uniform_real_distribution<> dis_y(-4.0, 5.5);  //y 축 랜덤샘플링 범위 조정 
        //std::uniform_real_distribution<> dis_goal(0.0, 1.0);   // 목표점 샘플링 비율

        float step_size = 0.1f; // 웨이포인트 간격 20cm
        float goal_thresh = 0.2f;  // 목표 도달 기준 20cm 
       // float margin = 0.4f;  //장애물과 경로 사이의 최소 거리 cm 
        float goal_sample_rate = 0.12f; // 목표 샘플링 비율 5%
        float rewire_radius = 0.4f;  // 재연결 반경 40cm (rewire_radius)
        
        for (int i = 0; i < 10000; ++i) {
            float rx, ry;

            // 목표점 샘플링 비율 적용
            if (std::generate_canonical<float, 10>(gen) < goal_sample_rate) {
                //goal biasing: 5% 확률로 목표 지점 샘플링
                rx = goal.first;
                ry = goal.second;
            } else {
                // 랜덤 샘플링
                rx = dis_x(gen);
                ry = dis_y(gen);
            }

            // 가장 가까운 노드 찾기
            float nearest_dist = 1e6;
            int nearest_idx = -1;
            for (size_t j = 0; j < nodes.size(); ++j) {
                float d = std::hypot(rx - nodes[j].x, ry - nodes[j].y);
                if (d < nearest_dist) {
                    nearest_dist = d;
                    nearest_idx = j;
                }
            }
            if (nearest_idx == -1) continue;
            float theta = std::atan2(ry - nodes[nearest_idx].y, rx - nodes[nearest_idx].x);
            float nx = nodes[nearest_idx].x + step_size * std::cos(theta);
            float ny = nodes[nearest_idx].y + step_size * std::sin(theta);
            bool collision = false;
            std::vector<int> indices;
            std::vector<float> distances;
            
            if (is_occupied(nx, ny)) {
                collision = true;
            }
            if (collision) continue;
            //RCLCPP_WARN(this->get_logger(), "충돌된 노드: (%f, %f)", nx, ny);

        // 새 노드 추가 및 cost 계산
        float new_cost = cost[nearest_idx] + std::hypot(nx - nodes[nearest_idx].x, ny - nodes[nearest_idx].y);
        int new_idx = nodes.size();
        nodes.emplace_back(nx, ny, nearest_idx);
        cost.push_back(new_cost);

        // // 리와이어링
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (i == new_idx) continue;
            float dx = nodes[i].x - nx;
            float dy = nodes[i].y - ny;
            float d = std::hypot(dx, dy);

            if (d < rewire_radius && cost[new_idx] + d < cost[i]) {
                // 직선 경로 충돌 확인 (OccupancyGrid 기반)
                bool rewire_collision = false;
                int steps = 25; // 보간 횟수

                for (int j = 1; j <= steps; ++j) {
                    float ix = nx + j * (nodes[i].x - nx) / steps;
                    float iy = ny + j * (nodes[i].y - ny) / steps;
                    if (is_occupied(ix, iy)) {
                        rewire_collision = true;
                        break;
                    }
                }
                if (!rewire_collision) {
                    nodes[i].parent_idx = new_idx;
                    cost[i] = cost[new_idx] + d;
                }
            }
        }

        // 목표 근처 도달 시 경로 생성
        if (std::hypot(nx - goal.first, ny - goal.second) < goal_thresh) {
            int current_idx = new_idx;
            int path_node_count = 0;  // ✅ 사용된 노드 수 카운트 변수 추가

            while (current_idx != -1) {
                path.emplace_back(nodes[current_idx].x, nodes[current_idx].y);
                //RCLCPP_INFO(this->get_logger(), "📌 자 드가자(선택노드): %.2f, %.2f", nodes[current_idx].x, nodes[current_idx].y);
                current_idx = nodes[current_idx].parent_idx;
                path_node_count++;  // ✅ 경로에 포함된 노드마다 +1
            }
            std::reverse(path.begin(), path.end());
            // 선택된 노드 path 시각화 (드가자 노드)
            publish_selected_rrt_nodes_marker(path);
            RCLCPP_INFO(this->get_logger(), " # 최종 경로에 사용된 노드 수: %d", path_node_count);
            RCLCPP_INFO(this->get_logger(), " # 전체 RRT* 트리 노드 수: %ld", nodes.size());

            // 🔴 RRT 전체 노드 시각화
            std::vector<std::pair<float, float>> rrt_node_positions;
            for (const auto& node : nodes) {
                rrt_node_positions.emplace_back(node.x, node.y);
                //RCLCPP_INFO(this->get_logger(), "🌸 전체 노드: %.2f, %.2f", node.x, node.y);
            }
            publish_rrt_nodes_marker(rrt_node_positions);

            // 경로 정보 출력
            double total_length = 0.0;
            for (size_t i = 1; i < path.size(); ++i) {
                float dx = path[i].first - path[i - 1].first;
                float dy = path[i].second - path[i - 1].second;

                total_length += std::hypot(dx, dy);
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), " ✔️ RRT* 경로 생성 시간: %ld ms", duration);
            RCLCPP_INFO(this->get_logger(), " ✔️ 생성된 경로 길이: %.2f m", total_length);
            
            break;
        }
    }
        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "generate_path() 실패. fallback 직선 경로 사용");
            path.emplace_back(start);
            path.emplace_back(goal);
        }
        //리와이어링 후 일정간격으로 보간 
        float interval = 0.1f;  // 10cm 간격
        path = interpolate_path(path, interval);

        return path;
    }
    
  
};

void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Landing command sent");
}

int main(int argc, char *argv[])
{
  std::cout << "Starting offboard control node..." << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
