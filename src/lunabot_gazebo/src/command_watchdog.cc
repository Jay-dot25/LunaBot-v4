// Actuator-side timeout for LunaBot. Uses command receipt and time, never world pose.
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <chrono>
#include <cmath>
#include <mutex>

namespace lunabot {
class CommandWatchdog final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate {
  using Clock = std::chrono::steady_clock;
  ignition::transport::Node node;
  ignition::transport::Node::Publisher commandPub, statePub;
  std::mutex mutex;
  ignition::msgs::Twist command;
  Clock::time_point receipt{}, lastPublish{};
  std::chrono::steady_clock::duration previousSim{};
  double timeout{0.25};
  bool received{false}, armed{false};
  std::string state{"STARTUP_INHIBIT"};
 public:
  ~CommandWatchdog() override {
    node.Unsubscribe("/lunabot/actuator_request");
    std::lock_guard<std::mutex> lock(mutex);
  }
  void Configure(const ignition::gazebo::Entity &,
      const std::shared_ptr<const sdf::Element> &sdf,
      ignition::gazebo::EntityComponentManager &,
      ignition::gazebo::EventManager &) override {
    timeout=sdf->Get<double>("timeout_sec",0.25).first;
    if (!std::isfinite(timeout) || timeout<=0) throw std::runtime_error("invalid watchdog timeout");
    commandPub=node.Advertise<ignition::msgs::Twist>("/cmd_vel");
    statePub=node.Advertise<ignition::msgs::StringMsg>("/lunabot/actuator_watchdog/state");
    node.Subscribe("/lunabot/actuator_request",&CommandWatchdog::OnCommand,this);
  }
  void OnCommand(const ignition::msgs::Twist &msg) {
    std::lock_guard<std::mutex> lock(mutex);
    const double v[]={msg.linear().x(),msg.linear().y(),msg.linear().z(),
                      msg.angular().x(),msg.angular().y(),msg.angular().z()};
    for (double x:v) if (!std::isfinite(x)) {
      armed=false; received=false; state="INVALID_COMMAND"; return;
    }
    if (v[1]!=0 || v[2]!=0 || v[3]!=0 || v[4]!=0) {
      armed=false; received=false; state="INVALID_COMMAND"; return;
    }
    command=msg; receipt=Clock::now(); received=true;
    if (v[0]==0 && v[5]==0) { armed=true; state="READY"; }
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
      ignition::gazebo::EntityComponentManager &) override {
    std::lock_guard<std::mutex> lock(mutex);
    auto now=Clock::now();
    if (info.simTime<previousSim) {
      armed=false; received=false; state="CLOCK_RESET";
    }
    previousSim=info.simTime;
    if (received && std::chrono::duration<double>(now-receipt).count()>timeout) {
      armed=false; state="TIMEOUT_LATCHED";
    }
    if (std::chrono::duration<double>(now-lastPublish).count()<0.02) return;
    ignition::msgs::Twist out;
    if (armed && received) out=command;
    commandPub.Publish(out);
    ignition::msgs::StringMsg status; status.set_data(state); statePub.Publish(status);
    lastPublish=now;
  }
};
}
IGNITION_ADD_PLUGIN(lunabot::CommandWatchdog, ignition::gazebo::System,
                    lunabot::CommandWatchdog::ISystemConfigure,
                    lunabot::CommandWatchdog::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(lunabot::CommandWatchdog,"lunabot::CommandWatchdog")
