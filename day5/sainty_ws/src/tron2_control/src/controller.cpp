#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace tron2_control
{
using Callback = controller_interface::CallbackReturn;
using Config = controller_interface::InterfaceConfiguration;
using JointState = sensor_msgs::msg::JointState;

struct Target {
  std::array<double,10> values{};
  rclcpp::Time stamp{0,0,RCL_ROS_TIME};
  bool enabled=false;
};

class WheelFootController : public controller_interface::ControllerInterface
{
public:
  Callback on_init() override
  {
    auto_declare<std::vector<std::string>>("joints", {});
    return Callback::SUCCESS;
  }
  Callback on_configure(const rclcpp_lifecycle::State &) override
  {
    joints=get_node()->get_parameter("joints").as_string_array();
    if (joints.size()!=10) return Callback::ERROR;
    names.clear();
    for (const auto &name : joints)
      names.push_back(name + (is_wheel(name) ? "/velocity" : "/position"));
    names.push_back("panel/enable");
    subscription=get_node()->create_subscription<JointState>("~/targets",1,
        [this](JointState::ConstSharedPtr m){ receive(*m); });
    return Callback::SUCCESS;
  }
  Config command_interface_configuration() const override
  { return {controller_interface::interface_configuration_type::INDIVIDUAL,names}; }
  Config state_interface_configuration() const override
  { return {controller_interface::interface_configuration_type::NONE,{}}; }

  Callback on_activate(const rclcpp_lifecycle::State &) override
  {
    // Controller manager can loan interfaces in a different order.
    slots.clear();
    for (const auto &name : names) {
      auto it=std::find_if(command_interfaces_.begin(),command_interfaces_.end(),
                           [&name](const auto &j){return j.get_name()==name;});
      if (it==command_interfaces_.end()) return Callback::ERROR;
      slots.push_back(it-command_interfaces_.begin());
    }
    buffer.writeFromNonRT(Target{});
    command_interfaces_[slots.back()].set_value(0.);
    return Callback::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &) override
  {
    const auto *m=buffer.readFromRT();
    const double age=(time-m->stamp).seconds();
    const bool enabled=m->enabled && age>=-0.05 && age<=0.25;
    if (enabled)
      for (size_t i=0; i<joints.size(); ++i) command_interfaces_[slots[i]].set_value(m->values[i]);
    command_interfaces_[slots.back()].set_value(enabled ? 1. : 0.);
    return controller_interface::return_type::OK;
  }
  Callback on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (!slots.empty()) command_interfaces_[slots.back()].set_value(0.);
    buffer.writeFromNonRT(Target{});
    return Callback::SUCCESS;
  }

private:
  static bool is_wheel(const std::string &n) {return n.rfind("wheel_",0)==0;}
  void receive(const JointState &m)
  {
    Target t;
    if (m.name.size()==joints.size() && m.position.size()==joints.size() && m.velocity.size()==joints.size()) {
      t.stamp=rclcpp::Time(m.header.stamp);
      t.enabled=true;
      for (size_t i=0; i<joints.size(); ++i) {
        const auto it=std::find(m.name.begin(),m.name.end(),joints[i]);
        if (it==m.name.end()) {t.enabled=false; break;}
        const auto j=it-m.name.begin();
        const bool wheel=is_wheel(joints[i]);
        if (!std::isfinite(m.position[j]) || !std::isfinite(m.velocity[j]) ||
            (wheel ? m.position[j] : m.velocity[j]) != 0.) {t.enabled=false; break;}
        t.values[i]=wheel ? m.velocity[j] : m.position[j];
      }
    }
    buffer.writeFromNonRT(t);
  }

  std::vector<std::string> joints,names;
  std::vector<size_t> slots;
  rclcpp::Subscription<JointState>::SharedPtr subscription;
  realtime_tools::RealtimeBuffer<Target> buffer{Target{}};
};
}
PLUGINLIB_EXPORT_CLASS(tron2_control::WheelFootController, controller_interface::ControllerInterface)
