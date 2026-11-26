#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/JointAxis.hh>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace tron2_gz
{
namespace sim = gz::sim;
using JointState = sensor_msgs::msg::JointState;
using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

struct Motor
{
  std::string name;
  sim::Joint joint;
  bool wheel;
  double kp, kd, max_effort, low, high;
  double q = 0, dq = 0, effort = 0, target = 0;
};

class WheelFootSystem : public sim::System, public sim::ISystemConfigure, public sim::ISystemPreUpdate
{
public:
  void Configure(const sim::Entity &entity, const std::shared_ptr<const sdf::Element> &,
                 sim::EntityComponentManager &ecm, sim::EventManager &) override
  {
    sim::Model model(entity);
    base = sim::Link(model.LinkByName(ecm, "base_Link"));
    base.EnableVelocityChecks(ecm);
    base.EnableAccelerationChecks(ecm);
    const auto dir = ament_index_cpp::get_package_share_directory("tron2_description");
    const auto cfg = YAML::LoadFile(dir + "/config/wf_control.yaml")["PointfootCfg"];
    const auto control = cfg["control"];
    const auto names = cfg["init_state"]["joint_names"].as<std::vector<std::string>>();
    const auto modes = control["joint_control_modes"].as<std::vector<std::string>>();
    for (size_t i = 0; i < names.size(); ++i) {
      const auto name = names[i] + "_Joint";
      sim::Joint joint(model.JointByName(ecm, name));
      const auto axes = joint.Axis(ecm);
      if (!axes || axes->empty()) throw std::runtime_error("Missing revolute joint: " + name);
      joint.EnablePositionCheck(ecm);
      joint.EnableVelocityCheck(ecm);
      const bool wheel = modes.at(i) == "velocity";
      const std::string group = wheel ? "wheel_joint" :
          (name.find("_yaw_") == std::string::npos ? "leg_joint" : "light_joint");
      indices[name] = motors.size();
      motors.push_back({name, joint, wheel, wheel ? 0.0 : control[group+"_stiffness"].as<double>(),
          control[group+"_damping"].as<double>(), control[group+"_torque_limit"].as<double>(),
          axes->front().Lower(), axes->front().Upper()});
    }
    // The WF model has a 0.1 m rolling radius, as used by the operator UI.
    wheel_speed = cfg["commands"]["max"]["lin_vel_x"].as<double>() / 0.1;
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    node = std::make_shared<rclcpp::Node>("gz_joint_control", "/tron2", options);
    rclcpp::ExecutorOptions exec_options;
    exec_options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec_options);
    executor->add_node(node);
    states = node->create_publisher<JointState>("joint_states", 10);
    imu = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    enabled = node->create_publisher<std_msgs::msg::Bool>("panel_motion_enabled", 1);
    tf = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    commands = node->create_subscription<JointState>("wf_targets", 1,
        [this](JointState::ConstSharedPtr msg) { receive(*msg); });
  }

  void PreUpdate(const sim::UpdateInfo &info, sim::EntityComponentManager &ecm) override
  {
    if (!node) return;
    executor->spin_some();
    const auto now = Clock::now();
    if (active && (now - received > Seconds(0.25) || info.paused || info.dt.count() < 0)) {
      active = false;
      release_required = true;
    }
    if (info.paused) return;
    for (auto &m : motors) {
      const auto q = m.joint.Position(ecm), dq = m.joint.Velocity(ecm);
      if (!q || !dq || q->empty() || dq->empty()) return;
      m.q = q->front();
      m.dq = dq->front();
      const double v = active && m.wheel ? m.target : 0.0;
      const double p_error = active && !m.wheel ? m.target - m.q : 0.0;
      m.effort = engaged ? std::clamp(m.kp*p_error + m.kd*(v-m.dq), -m.max_effort, m.max_effort) : 0.0;
      m.joint.SetForce(ecm, {m.effort});
    }
    if (now - last_publish >= Seconds(0.02)) {
      publish(ecm);
      last_publish = now;
    }
  }

private:
  void receive(const JointState &msg)
  {
    active = false;
    if (msg.name.empty()) { release_required = false; return; }
    const auto age = (node->now() - rclcpp::Time(msg.header.stamp)).seconds();
    if (release_required || age < -0.05 || age > 0.25 || msg.name.size() != motors.size() ||
        msg.position.size() != motors.size() || msg.velocity.size() != motors.size()) return;
    std::vector<bool> seen(motors.size());
    for (size_t i = 0; i < motors.size(); ++i) {
      const auto item = indices.find(msg.name[i]);
      if (item == indices.end() || seen[item->second]) return;
      seen[item->second] = true;
      auto &m = motors[item->second];
      const double p = msg.position[i], v = msg.velocity[i];
      if (!std::isfinite(p) || !std::isfinite(v) || p < m.low || p > m.high ||
          std::abs(v) > wheel_speed + 1e-9 || (m.wheel ? p != 0 : v != 0)) return;
      m.target = m.wheel ? v : p;
    }
    active = engaged = true;
    received = Clock::now();
  }

  void publish(const sim::EntityComponentManager &ecm)
  {
    JointState js;
    js.header.stamp = node->now();
    for (const auto &m : motors) {
      js.name.push_back(m.name);
      js.position.push_back(m.q);
      js.velocity.push_back(m.dq);
      js.effort.push_back(m.effort);
    }
    states->publish(js);
    std_msgs::msg::Bool permission;
    permission.data = true;
    enabled->publish(permission);
    const auto pose = sim::worldPose(base.Entity(), ecm);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = js.header.stamp;
    t.header.frame_id = "world";
    t.child_frame_id = "tron2/base_Link";
    auto &p = t.transform.translation;
    p.x = pose.Pos().X(); p.y = pose.Pos().Y(); p.z = pose.Pos().Z();
    auto &r = t.transform.rotation;
    r.x = pose.Rot().X(); r.y = pose.Rot().Y(); r.z = pose.Rot().Z(); r.w = pose.Rot().W();
    tf->sendTransform(t);
    const auto omega = base.WorldAngularVelocity(ecm), accel = base.WorldLinearAcceleration(ecm);
    if (!omega || !accel) return;
    // base_imu is coincident with base_Link. Report specific force in the IMU frame.
    const auto gyro = pose.Rot().RotateVectorReverse(*omega);
    const auto specific = pose.Rot().RotateVectorReverse(*accel - gz::math::Vector3d(0, 0, -9.80665));
    sensor_msgs::msg::Imu sample;
    sample.header.stamp = js.header.stamp;
    sample.header.frame_id = "tron2/base_imu";
    sample.orientation = r;
    sample.angular_velocity.x = gyro.X(); sample.angular_velocity.y = gyro.Y(); sample.angular_velocity.z = gyro.Z();
    sample.linear_acceleration.x = specific.X();
    sample.linear_acceleration.y = specific.Y();
    sample.linear_acceleration.z = specific.Z();
    imu->publish(sample);
  }

  sim::Link base;
  std::vector<Motor> motors;
  std::map<std::string, size_t> indices;
  bool active = false, engaged = false, release_required = false;
  double wheel_speed = 0;
  Clock::time_point received{}, last_publish{};
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<JointState>::SharedPtr states;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled;
  rclcpp::Subscription<JointState>::SharedPtr commands;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
};
}
IGNITION_ADD_PLUGIN(tron2_gz::WheelFootSystem, gz::sim::System,
                    tron2_gz::WheelFootSystem::ISystemConfigure, tron2_gz::WheelFootSystem::ISystemPreUpdate)
