#include <mc_radar_sdk/radar_sdk.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace {

// The signal handler only changes a lock-free flag. SDK shutdown runs in main.
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Signal handling requires lock-free bool atomics");
std::atomic<bool> stop_requested{false};

void HandleSignal(int) {
  stop_requested.store(true, std::memory_order_relaxed);
}

void PrintJsonFrame(const mc_radar_sdk::RadarFrame& frame) {
  std::cout << "{\"frame\":" << frame.header.frame_count
            << ",\"host_timestamp_ns\":" << frame.host_timestamp_ns
            << ",\"device_timestamp_us\":";
  if (frame.has_device_timestamp) {
    std::cout << frame.device_timestamp_us;
  } else {
    std::cout << "null";
  }
  std::cout << ",\"points\":[";
  bool first = true;
  for (const auto& point : frame.detections) {
    if (!first) {
      std::cout << ',';
    }
    first = false;
    std::cout << "{\"id\":" << point.id
              << ",\"x\":" << point.position.x
              << ",\"y\":" << point.position.y
              << ",\"z\":" << point.position.z
              << ",\"vel\":" << point.vel
              << ",\"rcs\":" << point.rcs
              << ",\"snr\":" << point.snr << '}';
  }
  std::cout << "]}\n" << std::flush;
}

void PrintFrame(const mc_radar_sdk::RadarFrame& frame) {
  std::cout << "\nframe=" << frame.header.frame_count
            << " target_points=" << frame.detections.size()
            << " source=" << frame.source_ip << ':' << frame.source_port
            << " device_timestamp_us=";
  if (frame.has_device_timestamp) {
    std::cout << frame.device_timestamp_us;
  } else {
    std::cout << "unavailable";
  }
  std::cout << '\n';

  if (!frame.detections.empty()) {
    std::cout << "   id      x(m)      y(m)      z(m)  range(m)  vel(m/s)"
                 "   az(deg)   el(deg)       rcs   snr_raw  exist_conf\n";
    for (const auto& point : frame.detections) {
      std::cout << std::setw(5) << point.id
                << std::setw(10) << point.position.x
                << std::setw(10) << point.position.y
                << std::setw(10) << point.position.z
                << std::setw(10) << point.range
                << std::setw(10) << point.vel
                << std::setw(10) << point.azimuth
                << std::setw(10) << point.elevation
                << std::setw(10) << point.rcs
                << std::setw(10) << point.snr
                << std::setw(12) << static_cast<unsigned int>(point.exist_confidence)
                << '\n';
    }
  }
  std::cout << std::flush;
}

}  // namespace

int main(int argc, char** argv) {
  const bool json_output = argc == 2 && std::string(argv[1]) == "--json";
  if (argc != 1 && !json_output) {
    const bool help = argc == 2 && std::string(argv[1]) == "--help";
    std::cerr << "Usage: " << argv[0] << " [--json]\n"
              << "3D viewer: python3 tests/view_points.py\n";
    return help ? 0 : 1;
  }
  // Change these three settings to match your host and radar network.
  mc_radar_sdk::RadarConfig config;
  config.bind_ip = "10.10.106.100";
  config.port = 9999;
  config.radar_ip = "10.10.106.106";

  if (std::signal(SIGINT, HandleSignal) == SIG_ERR ||
      std::signal(SIGTERM, HandleSignal) == SIG_ERR) {
    std::cerr << "Failed to install signal handlers\n";
    return 1;
  }

  mc_radar_sdk::RadarSdk radar;
  if (!radar.Start(config)) {
    std::cerr << "Failed to start radar: " << radar.GetStatus().last_error << '\n';
    return 1;
  }

  std::ostream& notices = json_output ? std::cerr : std::cout;
  notices << "Listening on " << config.bind_ip << ':' << config.port
          << ", radar=" << config.radar_ip << ". Press Ctrl+C to stop.\n"
          << std::flush;
  if (json_output) {
    std::cout << std::setprecision(std::numeric_limits<float>::max_digits10);
  } else {
    std::cout << std::fixed << std::setprecision(3);
  }

  auto last_notice = std::chrono::steady_clock::now();
  int exit_code = 0;
  while (!stop_requested.load(std::memory_order_relaxed)) {
    mc_radar_sdk::FramePtr frame;
    if (radar.WaitForFrame(frame, std::chrono::milliseconds(200))) {
      if (json_output) {
        PrintJsonFrame(*frame);
      } else {
        PrintFrame(*frame);
      }
      last_notice = std::chrono::steady_clock::now();
      continue;
    }
    const auto status = radar.GetStatus();
    if (!status.running) {
      std::cerr << "Radar receiver stopped: " << status.last_error << '\n';
      exit_code = 1;
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_notice >= std::chrono::seconds(1)) {
      std::cerr << "Waiting for a complete frame: received=" << status.received_datagrams
                << " invalid=" << status.invalid_datagrams
                << " incomplete=" << status.incomplete_frames << '\n';
      last_notice = now;
    }
  }

  radar.Stop();
  const auto status = radar.GetStatus();
  notices << "\nStopped. frames=" << status.completed_frames
          << " invalid=" << status.invalid_datagrams
          << " incomplete=" << status.incomplete_frames
          << " dropped=" << status.dropped_frames << '\n';
  return exit_code;
}
