#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "rclcpp/rclcpp.hpp"

namespace venom_px4_bridge
{

namespace
{

rclcpp::QoS px4_qos()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

std::string join_topic(const std::string & prefix, const std::string & suffix)
{
  if (prefix.empty()) {
    return suffix;
  }
  if (prefix.back() == '/') {
    return prefix.substr(0, prefix.size() - 1) + suffix;
  }
  return prefix + suffix;
}

// Variance fallback: use default when input is zero, negative, or non-finite
float safe_variance(float v, float def)
{
  return (std::isfinite(v) && v > 0.0f) ? v : def;
}

}  // namespace

class Px4ExternalPoseBridge : public rclcpp::Node
{
public:
  Px4ExternalPoseBridge()
  : Node("px4_external_pose_bridge")
  {
    const auto fmu_prefix = this->declare_parameter<std::string>("fmu_prefix", "/fmu");

    input_odom_topic_ = this->declare_parameter<std::string>(
      "input_odom_topic", "/lio/vps/odometry");
    timesync_status_topic_ = this->declare_parameter<std::string>(
      "timesync_status_topic", join_topic(fmu_prefix, "/out/timesync_status"));
    output_visual_odom_topic_ = this->declare_parameter<std::string>(
      "output_visual_odom_topic", join_topic(fmu_prefix, "/in/vehicle_visual_odometry"));

    use_input_covariance_ = this->declare_parameter<bool>("use_input_covariance", true);

    // pose_frame: "ned" assumes Lio map x=North (rare); "frd" lets EKF2 estimate the heading
    // offset — use "frd" for any VPS that starts with arbitrary orientation (default).
    const auto pose_frame_str = this->declare_parameter<std::string>("pose_frame", "frd");
    pose_frame_ned_ = (pose_frame_str == "ned");

    // velocity_frame: "world" = Lio publishes velocity in world/ENU frame (Point-LIO default)
    //                 "body"  = velocity in body/FLU frame → bridge uses VELOCITY_FRAME_BODY_FRD
    const auto vel_frame_str = this->declare_parameter<std::string>("velocity_frame", "world");
    velocity_in_body_frame_ = (vel_frame_str == "body");

    const auto pos_var = this->declare_parameter<std::vector<double>>(
      "default_position_variance", {0.05, 0.05, 0.10});
    const auto ori_var = this->declare_parameter<std::vector<double>>(
      "default_orientation_variance", {0.02, 0.02, 0.05});
    const auto vel_var = this->declare_parameter<std::vector<double>>(
      "default_velocity_variance", {0.05, 0.05, 0.10});
    default_pos_var_ = {pos_var[0], pos_var[1], pos_var[2]};
    default_ori_var_ = {ori_var[0], ori_var[1], ori_var[2]};
    default_vel_var_ = {vel_var[0], vel_var[1], vel_var[2]};

    max_pose_jump_m_ = this->declare_parameter<double>("max_pose_jump_m", 0.5);
    max_velocity_mps_ = this->declare_parameter<double>("max_velocity_jump_mps", 30.0);
    timesync_timeout_ms_ = this->declare_parameter<int>("timesync_timeout_ms", 1500);

    visual_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
      output_visual_odom_topic_, px4_qos());

    timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
      timesync_status_topic_, px4_qos(),
      [this](const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
        last_timesync_ = *msg;
        timesync_received_at_ = std::chrono::steady_clock::now();
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, rclcpp::QoS(10),
      std::bind(&Px4ExternalPoseBridge::on_odometry, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "px4_external_pose_bridge started: %s → %s",
      input_odom_topic_.c_str(), output_visual_odom_topic_.c_str());
  }

private:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  bool timesync_fresh() const
  {
    if (timesync_received_at_ == SteadyTimePoint{}) {
      return false;
    }
    const auto age = std::chrono::steady_clock::now() - timesync_received_at_;
    return age <= std::chrono::milliseconds(timesync_timeout_ms_);
  }

  bool pose_jump_ok(const nav_msgs::msg::Odometry & msg) const
  {
    if (!last_odom_) {
      return true;
    }
    const auto & p0 = last_odom_->pose.pose.position;
    const auto & p1 = msg.pose.pose.position;
    const double d = std::sqrt(
      (p1.x - p0.x) * (p1.x - p0.x) +
      (p1.y - p0.y) * (p1.y - p0.y) +
      (p1.z - p0.z) * (p1.z - p0.z));
    return d <= max_pose_jump_m_;
  }

  bool velocity_ok(const nav_msgs::msg::Odometry & msg) const
  {
    const auto & v = msg.twist.twist.linear;
    const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return speed <= max_velocity_mps_;
  }

  // Convert ROS wall-clock time to PX4 boot-time microseconds.
  // TimesyncStatus.estimated_offset = ros_us - px4_boot_us
  // → px4_boot_us = ros_us - estimated_offset
  uint64_t to_px4_timestamp(const rclcpp::Time & ros_time) const
  {
    const int64_t ros_us = ros_time.nanoseconds() / 1000;
    const int64_t px4_us = ros_us - last_timesync_->estimated_offset;
    return static_cast<uint64_t>(std::max(int64_t{0}, px4_us));
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!timesync_fresh()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Timesync stale — dropping visual odometry");
      return;
    }

    if (!pose_jump_ok(*msg)) {
      RCLCPP_WARN(this->get_logger(), "Pose jump > %.2f m detected — dropping", max_pose_jump_m_);
      return;
    }

    if (!velocity_ok(*msg)) {
      RCLCPP_WARN(this->get_logger(), "Velocity > %.1f m/s detected — dropping", max_velocity_mps_);
      return;
    }

    using namespace px4_ros_com::frame_transforms;

    // Position: ENU → NED
    const Eigen::Vector3d pos_enu{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    const Eigen::Vector3d pos_ned = enu_to_ned_local_frame(pos_enu);

    // Orientation: ROS (baselink/ENU) → PX4 (aircraft/NED)
    const auto & q_in = msg->pose.pose.orientation;
    const Eigen::Quaterniond q_enu_flu{q_in.w, q_in.x, q_in.y, q_in.z};
    const Eigen::Quaterniond q_ned_frd = ros_to_px4_orientation(q_enu_flu);

    // Velocity conversion depends on what frame Lio publishes in.
    // Point-LIO (and most LIO algorithms): world frame → use enu_to_ned_local_frame.
    // Some algorithms publish in body frame (FLU) → use baselink_to_aircraft_body_frame.
    Eigen::Vector3d vel_out;
    uint8_t velocity_frame;
    if (velocity_in_body_frame_) {
      vel_out = baselink_to_aircraft_body_frame(Eigen::Vector3d{
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z});
      velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    } else {
      vel_out = enu_to_ned_local_frame(Eigen::Vector3d{
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z});
      velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    }

    // Angular velocity: always in body-fixed FRD (hardcoded by VehicleOdometry spec)
    const Eigen::Vector3d ang_flu{
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z};
    const Eigen::Vector3d ang_frd = baselink_to_aircraft_body_frame(ang_flu);

    // Build VehicleOdometry
    px4_msgs::msg::VehicleOdometry vo;
    // timestamp      = message receipt/processing time (PX4 boot us)
    // timestamp_sample = actual measurement time (EKF2 uses this for delay compensation)
    const uint64_t ts = to_px4_timestamp(msg->header.stamp);
    vo.timestamp = ts;
    vo.timestamp_sample = ts;

    // POSE_FRAME_NED: EKF2 expects x=True North — only if Lio map is North-aligned.
    // POSE_FRAME_FRD: EKF2 estimates the offset between Lio frame and NED — correct for
    //                 VPS that starts with arbitrary heading (default).
    vo.pose_frame = pose_frame_ned_
      ? px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED
      : px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    vo.position[0] = static_cast<float>(pos_ned.x());
    vo.position[1] = static_cast<float>(pos_ned.y());
    vo.position[2] = static_cast<float>(pos_ned.z());

    // PX4 quaternion layout: [w, x, y, z]  (Hamiltonian convention)
    vo.q[0] = static_cast<float>(q_ned_frd.w());
    vo.q[1] = static_cast<float>(q_ned_frd.x());
    vo.q[2] = static_cast<float>(q_ned_frd.y());
    vo.q[3] = static_cast<float>(q_ned_frd.z());

    vo.velocity_frame = velocity_frame;
    vo.velocity[0] = static_cast<float>(vel_out.x());
    vo.velocity[1] = static_cast<float>(vel_out.y());
    vo.velocity[2] = static_cast<float>(vel_out.z());

    vo.angular_velocity[0] = static_cast<float>(ang_frd.x());
    vo.angular_velocity[1] = static_cast<float>(ang_frd.y());
    vo.angular_velocity[2] = static_cast<float>(ang_frd.z());

    fill_covariance(vo, *msg);

    vo.reset_counter = 0;
    vo.quality = 100;

    visual_odom_pub_->publish(vo);
    last_odom_ = *msg;
  }

  // Map covariance from nav_msgs/Odometry (ENU) to VehicleOdometry (NED).
  // NED axes: x=North, y=East, z=Down. ENU axes: x=East, y=North, z=Up.
  // Permutation: NED[0] ↔ ENU[1], NED[1] ↔ ENU[0], NED[2] = ENU[2].
  // Diagonal covariance indices in 6x6 row-major: 0(xx), 7(yy), 14(zz), 21(rr), 28(pp), 35(yy).
  void fill_covariance(
    px4_msgs::msg::VehicleOdometry & vo,
    const nav_msgs::msg::Odometry & odom) const
  {
    if (use_input_covariance_) {
      const auto & pc = odom.pose.covariance;
      const auto & tc = odom.twist.covariance;

      // NED North var = ENU North (y) var; NED East var = ENU East (x) var
      vo.position_variance[0] = safe_variance(
        static_cast<float>(pc[7]), static_cast<float>(default_pos_var_[0]));
      vo.position_variance[1] = safe_variance(
        static_cast<float>(pc[0]), static_cast<float>(default_pos_var_[1]));
      vo.position_variance[2] = safe_variance(
        static_cast<float>(pc[14]), static_cast<float>(default_pos_var_[2]));

      vo.orientation_variance[0] = safe_variance(
        static_cast<float>(pc[21]), static_cast<float>(default_ori_var_[0]));
      vo.orientation_variance[1] = safe_variance(
        static_cast<float>(pc[28]), static_cast<float>(default_ori_var_[1]));
      vo.orientation_variance[2] = safe_variance(
        static_cast<float>(pc[35]), static_cast<float>(default_ori_var_[2]));

      vo.velocity_variance[0] = safe_variance(
        static_cast<float>(tc[7]), static_cast<float>(default_vel_var_[0]));
      vo.velocity_variance[1] = safe_variance(
        static_cast<float>(tc[0]), static_cast<float>(default_vel_var_[1]));
      vo.velocity_variance[2] = safe_variance(
        static_cast<float>(tc[14]), static_cast<float>(default_vel_var_[2]));
    } else {
      for (int i = 0; i < 3; ++i) {
        vo.position_variance[i] = static_cast<float>(default_pos_var_[i]);
        vo.orientation_variance[i] = static_cast<float>(default_ori_var_[i]);
        vo.velocity_variance[i] = static_cast<float>(default_vel_var_[i]);
      }
    }
  }

  std::string input_odom_topic_;
  std::string timesync_status_topic_;
  std::string output_visual_odom_topic_;
  bool use_input_covariance_{true};
  bool pose_frame_ned_{false};       // false = POSE_FRAME_FRD (default, arbitrary heading)
  bool velocity_in_body_frame_{false};
  std::array<double, 3> default_pos_var_{0.05, 0.05, 0.10};
  std::array<double, 3> default_ori_var_{0.02, 0.02, 0.05};
  std::array<double, 3> default_vel_var_{0.05, 0.05, 0.10};
  double max_pose_jump_m_{0.5};
  double max_velocity_mps_{30.0};
  int timesync_timeout_ms_{1500};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;

  std::optional<px4_msgs::msg::TimesyncStatus> last_timesync_;
  SteadyTimePoint timesync_received_at_;
  std::optional<nav_msgs::msg::Odometry> last_odom_;
};

}  // namespace venom_px4_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<venom_px4_bridge::Px4ExternalPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
