#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

class ReactiveGapFollow : public rclcpp::Node
{
public:
  ReactiveGapFollow()
  : Node("reactive_gap_follow")
  {
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ReactiveGapFollow::lidar_callback, this, std::placeholders::_1));
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);
  }

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<float> proc_ranges = preprocess_lidar(
      msg->ranges, msg->angle_min,
      msg->angle_increment);

    float min_distance = find_min_distance(proc_ranges);

    proc_ranges = safety_bubble(
      20, proc_ranges, std::distance(
        proc_ranges.begin(),
        std::find(proc_ranges.begin(), proc_ranges.end(), min_distance)));

    std::vector<float> max_gap = find_max_gap(proc_ranges);

    float best_point = find_best_point(max_gap);
    float angle = msg->angle_min +
      (std::distance(
        proc_ranges.begin(),
        std::find(proc_ranges.begin(), proc_ranges.end(), best_point)) * msg->angle_increment);
    float speed = 1.0;
    publish_drive_msg(angle, speed);
  }

  std::vector<float> preprocess_lidar(
    const std::vector<float> & ranges, float angle_min,
    float angle_increment)
  {
    left_boundary_ = static_cast<int>((M_PI / 180.0 * 70 - angle_min) / angle_increment);
    right_boundary_ = static_cast<int>((M_PI / 180.0 * -70 - angle_min) / angle_increment);

    std::vector<float> proc_ranges(ranges.begin(), ranges.end());
    for (size_t i = 0; i < proc_ranges.size(); ++i) {
      if (proc_ranges[i] > 3.0) {
        proc_ranges[i] = 0.0;
      }
    }
    return proc_ranges;
  }

  float find_min_distance(const std::vector<float> & ranges)
  {
    float min_distance = std::numeric_limits<float>::max();
    for (int i = right_boundary_; i < left_boundary_; ++i) {
      if (ranges[i] != 0.0 && ranges[i] < min_distance) {
        min_distance = ranges[i];
      }
    }
    return min_distance;
  }

  std::vector<float> safety_bubble(
    int safety_bubble_radius, std::vector<float> & ranges,
    size_t min_distance_idx)
  {
    int left_boundary = std::max(static_cast<int>(min_distance_idx) - safety_bubble_radius, 0);
    int right_boundary = std::min(
      static_cast<int>(min_distance_idx) + safety_bubble_radius,
      static_cast<int>(ranges.size()));

    for (int i = left_boundary; i < right_boundary; ++i) {
      ranges[i] = 0.0;
    }
    return ranges;
  }

  std::vector<float> find_max_gap(const std::vector<float> & free_space_ranges)
  {
    std::vector<float> max_gap;
    std::vector<float> temp;
    int max_gap_len = -std::numeric_limits<int>::max();

    for (int i = right_boundary_; i < left_boundary_; ++i) {
      if (free_space_ranges[i] > 2.0) {
        temp.push_back(free_space_ranges[i]);
        if (static_cast<int>(temp.size()) > max_gap_len) {
          max_gap = temp;
          max_gap_len = static_cast<int>(max_gap.size());
        }
      } else {
        temp.clear();
      }
    }
    return max_gap;
  }

  float find_best_point(const std::vector<float> & max_gap)
  {
    if (max_gap.empty()) {
      return 0.0;
    }
    return max_gap[max_gap.size() / 2];
  }

  void publish_drive_msg(float angle, float speed)
  {
    auto msg = ackermann_msgs::msg::AckermannDriveStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "laser";
    msg.drive.steering_angle = angle;
    msg.drive.speed = speed;
    drive_publisher_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
  int left_boundary_;
  int right_boundary_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReactiveGapFollow>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
