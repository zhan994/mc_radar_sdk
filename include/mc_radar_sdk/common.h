#ifndef MC_RADAR_SDK_COMMON_H_
#define MC_RADAR_SDK_COMMON_H_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mc_radar_sdk {

enum class MessageId : std::uint16_t {
  kTimestamp = 0x001,
  kHeader = 0x100,
  kDetections = 0x101,
  kEnd = 0x1FF,
};

constexpr std::size_t kMessageOverhead = 8;
constexpr std::size_t kDetectionSize = 24;

// Radar coordinates in meters: x forward, y left, z up.
struct Point3D {
  float x;
  float y;
  float z;
};

struct FrameHeader {
  std::uint16_t version = 0;
  std::uint16_t frame_count = 0;
  std::uint16_t detection_number = 0;
  std::uint16_t reserved = 0;
};

struct Detection {
  std::uint16_t id = 0;
  std::uint16_t flags = 0;
  float range = 0;      // meters, wire value / 100
  float vel = 0;        // radial velocity in m/s, wire value / 100
  float azimuth = 0;    // degrees, positive left, wire value / 100
  float elevation = 0;  // degrees, positive down, wire value / 100
  float rcs = 0;        // wire value / 256
  std::int16_t snr = 0; // raw wire value; physical scale is not yet confirmed
  std::uint8_t doppler_confidence = 0;
  std::uint8_t exist_confidence = 0;
  std::uint16_t frame = 0;
  std::uint8_t beam = 0;
  std::uint8_t ext_counter = 0;
  Point3D position{0, 0, 0};
};

struct RadarFrame {
  FrameHeader header;
  // Device clock epoch/synchronization is unknown; never assume Unix time.
  bool has_device_timestamp = false;
  std::uint64_t device_timestamp_us = 0;
  // Host system_clock time when the header datagram was received, in ns.
  std::uint64_t host_timestamp_ns = 0;
  std::string source_ip;
  std::uint16_t source_port = 0;
  // Preserved for diagnostics, without assuming a counter increment rule.
  std::uint16_t first_message_counter = 0;
  std::uint16_t last_message_counter = 0;
  std::vector<Detection> detections;
};

using FramePtr = std::shared_ptr<const RadarFrame>;

struct RadarConfig {
  std::string bind_ip = "0.0.0.0";
  std::uint16_t port = 9999; // 0 requests an ephemeral local port.
  std::string radar_ip;     // Empty accepts the first valid timestamp/header source.
  std::size_t max_queued_frames = 32;
  std::chrono::milliseconds frame_timeout{500};
  std::chrono::milliseconds connection_timeout{5000};
};

struct RadarStatus {
  bool running = false;
  bool connected = false;   // Recently received a valid message from selected source.
  bool frame_valid = false; // Recently completed a frame (including an empty frame).
  std::uint16_t bound_port = 0;
  std::uint64_t received_datagrams = 0;
  std::uint64_t ignored_datagrams = 0;
  std::uint64_t invalid_datagrams = 0;
  std::uint64_t unknown_messages = 0;
  std::uint64_t completed_frames = 0;
  std::uint64_t incomplete_frames = 0;
  std::uint64_t dropped_frames = 0; // Completed frames evicted from the output queue.
  std::string last_error;
};

}  // namespace mc_radar_sdk

#endif  // MC_RADAR_SDK_COMMON_H_
