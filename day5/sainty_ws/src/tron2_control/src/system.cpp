#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace tron2_control
{
using State = sensor_msgs::msg::JointState;
using Imu = sensor_msgs::msg::Imu;
using Clock = std::chrono::steady_clock;
using Callback = hardware_interface::CallbackReturn;
using namespace std::chrono_literals;

// All three devices expose the fixed WF packet contract. The selected transport is
// an external process: native LimX SDK (network / loopback) or the Gazebo physics system.
class WheelFootSystem : public hardware_interface::SystemInterface
{
public:
  Callback on_init(const hardware_interface::HardwareInfo &info) override
  {
    if (SystemInterface::on_init(info) != Callback::SUCCESS) return Callback::ERROR;
    for (const auto &j : info.joints) {
      names.push_back(j.name);
      velocity_mode.push_back(j.command_interfaces.at(0).name == "velocity");
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (auto &v : state) v.assign(names.size(), nan);
    targets.assign(names.size(), 0.);
    imu_values.fill(nan);
    const auto prefix = info.hardware_parameters.at("io_prefix");
    node = std::make_shared<rclcpp::Node>("hardware_transport", "/tron2",
            rclcpp::NodeOptions().use_global_arguments(false));
    executor.add_node(node);
    pub = node->create_publisher<State>(prefix + "/wf_targets", 1);
    feedback = node->create_subscription<State>(prefix + "/joint_states", 1,
        [this](State::ConstSharedPtr m) { read_joints(*m); });
    permission = node->create_subscription<std_msgs::msg::Bool>(prefix + "/panel_motion_enabled", 1,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          allowed = m->data; last_permission = Clock::now();
        });
    imu = node->create_subscription<Imu>(prefix + "/imu", 1, [this](Imu::ConstSharedPtr m) {
      const auto &q = m->orientation; const auto &w = m->angular_velocity;
      const auto &a = m->linear_acceleration;
      imu_values = {q.x,q.y,q.z,q.w,w.x,w.y,w.z,a.x,a.y,a.z};
    });
    return Callback::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> out;
    const std::array<std::string,3> fields{"position","velocity","effort"};
    for (size_t i=0; i<names.size(); ++i)
      for (size_t j=0; j<fields.size(); ++j) out.emplace_back(names[i],fields[j],&state[j][i]);
    for (size_t i=0; i<imu_values.size(); ++i)
      out.emplace_back("base_imu",info_.sensors.at(0).state_interfaces.at(i).name,&imu_values[i]);
    return out;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> out;
    for (size_t i=0; i<names.size(); ++i)
      out.emplace_back(names[i],velocity_mode[i] ? "velocity" : "position",&targets[i]);
    out.emplace_back("panel","enable",&enable);
    return out;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    executor.spin_some();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const auto now = Clock::now();
    if (enable != 1.0 || !allowed || now-last_state > 250ms || now-last_permission > 250ms) {
      release();
    } else if (!sending || now-last_send >= 20ms || targets != previous) {
      State m;
      m.header.stamp = node->now();
      m.name=names; m.position.assign(names.size(),0.0); m.velocity.assign(names.size(),0.0);
      for (size_t i=0; i<names.size(); ++i)
        (velocity_mode[i] ? m.velocity : m.position)[i] = targets[i];
      pub->publish(m);
      previous=targets; sending=true; last_send=now;
    }
    return hardware_interface::return_type::OK;
  }

  Callback on_deactivate(const rclcpp_lifecycle::State &) override
  {
    enable=0.0; release(); return Callback::SUCCESS;
  }

private:
  void read_joints(const State &m)
  {
    const auto n=names.size();
    if (m.name.size()!=n || m.position.size()!=n || m.velocity.size()!=n || m.effort.size()!=n) return;
    std::array<size_t,10> order{};
    if (n!=order.size()) return;
    for (size_t i=0; i<n; ++i) {
      const auto it=std::find(m.name.begin(),m.name.end(),names[i]);
      if (it==m.name.end()) return;
      const size_t j=it-m.name.begin();
      if (!std::isfinite(m.position[j]) || !std::isfinite(m.velocity[j]) || !std::isfinite(m.effort[j])) return;
      order[i]=j;
    }
    for (size_t i=0; i<n; ++i) {
      const auto j=order[i];
      state[0][i]=m.position[j]; state[1][i]=m.velocity[j]; state[2][i]=m.effort[j];
    }
    last_state=Clock::now();
  }
  void release()
  {
    if (sending) { pub->publish(State{}); sending=false; }
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<State>::SharedPtr pub;
  rclcpp::Subscription<State>::SharedPtr feedback;
  rclcpp::Subscription<Imu>::SharedPtr imu;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission;
  std::vector<std::string> names;
  std::vector<bool> velocity_mode;
  std::vector<double> targets,previous;
  std::array<std::vector<double>,3> state;
  std::array<double,10> imu_values;
  double enable=0.0;
  bool sending=false, allowed=false;
  Clock::time_point last_state{}, last_permission{}, last_send{};
};
}
PLUGINLIB_EXPORT_CLASS(tron2_control::WheelFootSystem, hardware_interface::SystemInterface)
