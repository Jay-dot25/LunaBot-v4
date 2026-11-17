#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <ignition/msgs/clock.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>

namespace {
std::atomic<bool> running{true};
void Stop(int) { running.store(false); }

class TruthRecorder {
 public:
  explicit TruthRecorder(const std::string &path) : out(path) {
    out << "steady_ns,sim_time,x,y,z,qx,qy,qz,qw,yaw\n";
    out.flush();
  }

  bool Start() {
    const bool clockOk =
        node.Subscribe("/clock", &TruthRecorder::OnClock, this);
    const bool poseOk = node.Subscribe(
        "/world/lunar_world/pose/info", &TruthRecorder::OnPose, this);
    return clockOk && poseOk;
  }

 private:
  void OnClock(const ignition::msgs::Clock &msg) {
    const double t = msg.sim().sec() + 1e-9 * msg.sim().nsec();
    simTime.store(t, std::memory_order_relaxed);
    clockCount.fetch_add(1, std::memory_order_relaxed);
  }

  void OnPose(const ignition::msgs::Pose_V &msg) {
    const double t = simTime.load(std::memory_order_relaxed);
    if (clockCount.load(std::memory_order_relaxed) == 0 ||
        !std::isfinite(t))
      return;
    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto &p = msg.pose(i);
      if (p.name() != modelName)
        continue;
      const auto &pos = p.position();
      const auto &q = p.orientation();
      const double yaw = std::atan2(
          2.0 * (q.w() * q.z() + q.x() * q.y()),
          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      std::lock_guard<std::mutex> guard(mutex);
      out << ns << ',' << std::setprecision(12) << t << ','
          << pos.x() << ',' << pos.y() << ',' << pos.z() << ','
          << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w()
          << ',' << yaw << '\n';
      out.flush();
      count.fetch_add(1, std::memory_order_relaxed);
    }
  }

  ignition::transport::Node node;
  std::ofstream out;
  std::mutex mutex;
  std::atomic<double> simTime{-1.0};
  std::atomic<unsigned long long> clockCount{0};
  std::atomic<unsigned long long> count{0};
  const std::string modelName{"lunabot_v4"};
};
}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: capture_gazebo_truth OUTPUT.csv\n";
    return 2;
  }
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  TruthRecorder recorder(argv[1]);
  if (!recorder.Start()) {
    std::cerr << "failed to subscribe to /clock or world pose info\n";
    return 3;
  }
  while (running.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return 0;
}
