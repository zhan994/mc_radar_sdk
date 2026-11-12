#ifndef MC_RADAR_SDK_RADAR_SDK_HPP_
#define MC_RADAR_SDK_RADAR_SDK_HPP_

#include <arpa/inet.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include "common.h"

namespace mc_radar_sdk {

class RadarSdk {
 public:
  RadarSdk() = default;
  ~RadarSdk() { Stop(); }

  RadarSdk(const RadarSdk&) = delete;
  RadarSdk& operator=(const RadarSdk&) = delete;

  // Linux UDP receiver. Start/Stop are serialized; errors are in GetStatus().
  // Start resets queued frames/statistics and selects a new source endpoint.
  bool Start(const RadarConfig& config = RadarConfig()) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (running_.load()) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      status_.last_error = "receiver is already running";
      return false;
    }
    StopLocked(); // Reap a thread that may have exited after a socket error.
    std::lock_guard<std::mutex> lock(data_mutex_);
    status_ = RadarStatus();
    frames_.clear();
    config_ = config;
    ++generation_;
    source_selected_ = false;
    pending_timestamp_ = false;
    last_packet_ = Clock::time_point();
    last_frame_ = Clock::time_point();

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(config.port);
    in_addr radar_address{};
    if (inet_pton(AF_INET, config.bind_ip.c_str(), &local.sin_addr) != 1 ||
        (!config.radar_ip.empty() &&
         inet_pton(AF_INET, config.radar_ip.c_str(), &radar_address) != 1) ||
        config.max_queued_frames == 0 || config.frame_timeout.count() <= 0 ||
        config.connection_timeout.count() <= 0) {
      status_.last_error = "invalid IPv4 address, queue capacity, or timeout";
      return false;
    }
    radar_address_ = radar_address.s_addr;
    socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_fd_ == -1) {
      SetSystemError("socket", errno);
      return false;
    }
    if (bind(socket_fd_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) == -1) {
      SetSystemError("bind", errno);
      CloseDescriptors();
      return false;
    }
    socklen_t local_size = sizeof(local);
    if (getsockname(socket_fd_, reinterpret_cast<sockaddr*>(&local), &local_size) == -1) {
      SetSystemError("getsockname", errno);
      CloseDescriptors();
      return false;
    }
    stop_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (stop_fd_ == -1) {
      SetSystemError("eventfd", errno);
      CloseDescriptors();
      return false;
    }
    status_.bound_port = ntohs(local.sin_port);
    running_.store(true);
    try {
      receive_thread_ = std::thread(&RadarSdk::ReceiveLoop, this);
    } catch (const std::exception& error) {
      running_.store(false);
      status_.last_error = error.what();
      CloseDescriptors();
      return false;
    }
    return true;
  }

  // Wakes both poll() and frame waiters. Descriptors close only after joining.
  // Consumers must finish their calls before the RadarSdk object is destroyed.
  void Stop() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    StopLocked();
  }

  // Nonblocking: drains completed frames. Returned frames are immutable and
  // remain valid across subsequent Stop/Start calls and SDK destruction.
  std::vector<FramePtr> GetFrames() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<FramePtr> result;
    result.reserve(frames_.size());
    while (!frames_.empty()) {
      result.push_back(std::move(frames_.front()));
      frames_.pop_front();
    }
    return result;
  }

  // Consumers share one queue: each frame is delivered to one consumer only.
  // False means timeout, stopped with an empty queue, or a changed session.
  bool WaitForFrame(FramePtr& frame, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    const std::uint64_t generation = generation_;
    frame.reset();
    frame_ready_.wait_for(lock, timeout, [this, generation] {
      return !frames_.empty() || !running_.load() || generation_ != generation;
    });
    if (generation_ != generation || frames_.empty()) {
      return false;
    }
    frame = std::move(frames_.front());
    frames_.pop_front();
    return true;
  }

  void ClearFrames() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    frames_.clear();
  }

  RadarStatus GetStatus() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    RadarStatus result = status_;
    result.running = running_.load();
    const auto now = Clock::now();
    result.connected = result.running && last_packet_ != Clock::time_point() &&
                       now - last_packet_ < config_.connection_timeout;
    result.frame_valid = result.running && last_frame_ != Clock::time_point() &&
                         now - last_frame_ < config_.connection_timeout;
    return result;
  }

  // Range is in meters; angles are in degrees.
  // Azimuth is positive to the left; elevation is positive downward.
  static Point3D Polar2XYZ(float range, float azimuth, float elevation) noexcept {
    constexpr float kDegToRad = 0.01745329251994329577f;
    const float az = azimuth * kDegToRad;
    const float el = elevation * kDegToRad;
    const float horizontal_range = range * std::cos(el);

    return {horizontal_range * std::cos(az),
            horizontal_range * std::sin(az),
            -range * std::sin(el)};
  }

 private:
  using Clock = std::chrono::steady_clock;

  // Protocol is explicitly little-endian, matching the legacy SDK on x86.
  static std::uint16_t ReadU16(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0]) |
           (static_cast<std::uint16_t>(data[1]) << 8);
  }

  static std::int16_t ReadI16(const std::uint8_t* data) {
    const std::uint16_t value = ReadU16(data);
    return static_cast<std::int16_t>(value <= 0x7FFF ? value :
                                    static_cast<std::int32_t>(value) - 0x10000);
  }

  static std::int32_t ReadI32(const std::uint8_t* data) {
    const std::uint32_t value = static_cast<std::uint32_t>(ReadU16(data)) |
                              (static_cast<std::uint32_t>(ReadU16(data + 2)) << 16);
    return static_cast<std::int32_t>(value <= 0x7FFFFFFFU ? value :
                                    static_cast<std::int64_t>(value) - 0x100000000LL);
  }

  static std::uint64_t ReadU64(const std::uint8_t* data) {
    std::uint64_t value = 0;
    for (unsigned int i = 0; i < 8; ++i) {
      value |= static_cast<std::uint64_t>(data[i]) << (8 * i);
    }
    return value;
  }

  static Detection ParseDetection(const std::uint8_t* data) {
    Detection point;
    point.id = ReadU16(data);
    point.flags = ReadU16(data + 2);
    point.range = ReadI32(data + 4) / 100.0f;
    point.vel = ReadI16(data + 8) / 100.0f;
    point.azimuth = ReadI16(data + 10) / 100.0f;
    point.elevation = ReadI16(data + 12) / 100.0f;
    point.rcs = ReadI16(data + 14) / 256.0f;
    point.snr = ReadI16(data + 16) / 100.0f;
    point.doppler_confidence = data[18];
    point.exist_confidence = data[19];
    point.frame = ReadU16(data + 20);
    point.beam = data[22];
    point.ext_counter = data[23];
    point.position = Polar2XYZ(point.range, point.azimuth, point.elevation);
    return point;
  }

  // lifecycle_mutex_ is held. The receiver never takes lifecycle_mutex_.
  void StopLocked() {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      running_.store(false);
    }
    frame_ready_.notify_all();
    if (stop_fd_ != -1) {
      const std::uint64_t wake = 1;
      while (write(stop_fd_, &wake, sizeof(wake)) == -1 && errno == EINTR) {}
    }
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    CloseDescriptors();
    std::lock_guard<std::mutex> lock(data_mutex_);
    DiscardAssembly();
    pending_timestamp_ = false;
  }

  void CloseDescriptors() {
    if (socket_fd_ != -1) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
    if (stop_fd_ != -1) {
      close(stop_fd_);
      stop_fd_ = -1;
    }
  }

  // data_mutex_ is held by callers of all parsing/state helpers.
  void SetSystemError(const char* operation, int error) {
    status_.last_error = std::string(operation) + ": " +
                         std::error_code(error, std::system_category()).message();
  }

  void DiscardAssembly() {
    if (assembling_) {
      ++status_.incomplete_frames;
      assembling_ = false;
      current_frame_ = RadarFrame();
    }
  }

  void ExpireAssembly(Clock::time_point now) {
    if (assembling_ && now - frame_started_ >= config_.frame_timeout) {
      DiscardAssembly();
    }
    if (pending_timestamp_ && now - timestamp_received_ >= config_.frame_timeout) {
      pending_timestamp_ = false;
    }
  }

  void InvalidDatagram() {
    ++status_.invalid_datagrams;
    DiscardAssembly();
    pending_timestamp_ = false;
  }

  void ProcessDatagram(const std::uint8_t* data, std::size_t size,
                       const sockaddr_in& source, Clock::time_point now,
                       std::uint64_t host_timestamp_ns) {
    if ((!config_.radar_ip.empty() && source.sin_addr.s_addr != radar_address_) ||
        (source_selected_ &&
         (source.sin_addr.s_addr != source_endpoint_.sin_addr.s_addr ||
          source.sin_port != source_endpoint_.sin_port))) {
      ++status_.ignored_datagrams;
      return;
    }
    ++status_.received_datagrams;
    if (size < kMessageOverhead) {
      InvalidDatagram();
      return;
    }
    const auto id = static_cast<MessageId>(ReadU16(data));
    const std::size_t payload_size = ReadU16(data + 2);
    if (size != payload_size + kMessageOverhead ||
        (id == MessageId::kTimestamp && payload_size < 8) ||
        (id == MessageId::kHeader && payload_size != 8) ||
        (id == MessageId::kDetections && payload_size % kDetectionSize != 0)) {
      InvalidDatagram();
      return;
    }
    if (id != MessageId::kTimestamp && id != MessageId::kHeader &&
        id != MessageId::kDetections && id != MessageId::kEnd) {
      ++status_.unknown_messages;
      return;
    }
    // One source endpoint per session prevents interleaved radars mixing frames.
    if (!source_selected_) {
      if (id != MessageId::kTimestamp && id != MessageId::kHeader) {
        ++status_.ignored_datagrams;
        return;
      }
      source_endpoint_ = source;
      source_selected_ = true;
    }
    last_packet_ = now;
    const std::uint16_t counter = ReadU16(data + 4);
    const std::uint8_t* payload = data + 6;
    // The two trailing CRC bytes are not validated: the legacy SDK does not
    // specify the CRC algorithm. Counter sequence semantics are also unknown.
    if (id == MessageId::kTimestamp) {
      // The legacy SDK reads the first eight bytes as microseconds without
      // specifying a total payload size. Any extension bytes are not interpreted.
      device_timestamp_ = ReadU64(payload);
      pending_timestamp_ = true;
      timestamp_received_ = now;
    } else if (id == MessageId::kHeader) {
      DiscardAssembly();
      current_frame_ = RadarFrame();
      current_frame_.header.version = ReadU16(payload);
      current_frame_.header.frame_count = ReadU16(payload + 2);
      current_frame_.header.detection_number = ReadU16(payload + 4);
      current_frame_.header.reserved = ReadU16(payload + 6);
      current_frame_.has_device_timestamp = pending_timestamp_;
      current_frame_.device_timestamp_us = pending_timestamp_ ? device_timestamp_ : 0;
      pending_timestamp_ = false;
      current_frame_.host_timestamp_ns = host_timestamp_ns;
      char address[INET_ADDRSTRLEN]{};
      inet_ntop(AF_INET, &source.sin_addr, address, sizeof(address));
      current_frame_.source_ip = address;
      current_frame_.source_port = ntohs(source.sin_port);
      current_frame_.first_message_counter = counter;
      current_frame_.last_message_counter = counter;
      current_frame_.detections.reserve(current_frame_.header.detection_number);
      frame_started_ = now;
      assembling_ = true;
    } else if (id == MessageId::kDetections && assembling_) {
      const std::size_t count = payload_size / kDetectionSize;
      if (current_frame_.detections.size() + count > current_frame_.header.detection_number) {
        InvalidDatagram();
        return;
      }
      for (std::size_t offset = 0; offset < payload_size; offset += kDetectionSize) {
        current_frame_.detections.push_back(ParseDetection(payload + offset));
      }
      current_frame_.last_message_counter = counter;
    } else if (id == MessageId::kEnd && assembling_) {
      if (current_frame_.detections.size() != current_frame_.header.detection_number) {
        DiscardAssembly();
        return;
      }
      current_frame_.last_message_counter = counter;
      FramePtr frame = std::make_shared<RadarFrame>(std::move(current_frame_));
      frames_.push_back(std::move(frame));
      assembling_ = false;
      if (frames_.size() > config_.max_queued_frames) {
        frames_.pop_front();
        ++status_.dropped_frames;
      }
      ++status_.completed_frames;
      last_frame_ = now;
      frame_ready_.notify_one();
    }
  }

  void ReceiveLoop() noexcept {
    try {
      // Covers the entire IPv4 UDP payload, so oversized datagrams cannot be
      // silently truncated into apparently valid radar packets.
      std::array<std::uint8_t, 65536> buffer;
      pollfd descriptors[2] = {{socket_fd_, POLLIN, 0}, {stop_fd_, POLLIN, 0}};
      while (running_.load()) {
        const int ready = poll(descriptors, 2, 20);
        const int poll_error = errno;
        const auto now = Clock::now();
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!running_.load() || (descriptors[1].revents & POLLIN)) {
          break;
        }
        ExpireAssembly(now);
        if (ready == -1) {
          if (poll_error == EINTR) {
            continue;
          }
          SetSystemError("poll", poll_error);
          break;
        }
        if (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
          status_.last_error = "UDP socket poll error";
          break;
        }
        if (!(descriptors[0].revents & POLLIN)) {
          continue;
        }
        sockaddr_in source{};
        socklen_t source_size = sizeof(source);
        const ssize_t size = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                      reinterpret_cast<sockaddr*>(&source), &source_size);
        if (size == -1) {
          const int receive_error = errno;
          if (receive_error == EAGAIN || receive_error == EWOULDBLOCK || receive_error == EINTR) {
            continue;
          }
          SetSystemError("recvfrom", receive_error);
          break;
        }
        const auto host_time = std::chrono::system_clock::now().time_since_epoch();
        const auto host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(host_time).count();
        ProcessDatagram(buffer.data(), static_cast<std::size_t>(size), source, now,
                        static_cast<std::uint64_t>(host_ns));
      }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      status_.last_error = error.what();
    } catch (...) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      status_.last_error = "unexpected receiver exception";
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      running_.store(false);
      DiscardAssembly();
    }
    frame_ready_.notify_all();
  }

  std::mutex lifecycle_mutex_;
  mutable std::mutex data_mutex_;
  std::condition_variable frame_ready_;
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  int socket_fd_ = -1;
  int stop_fd_ = -1;
  RadarConfig config_;
  RadarStatus status_;
  std::deque<FramePtr> frames_;
  std::uint64_t generation_ = 0;

  bool source_selected_ = false;
  sockaddr_in source_endpoint_{};
  std::uint32_t radar_address_ = 0;
  bool assembling_ = false;
  RadarFrame current_frame_;
  Clock::time_point frame_started_{};
  bool pending_timestamp_ = false;
  std::uint64_t device_timestamp_ = 0;
  Clock::time_point timestamp_received_{};
  Clock::time_point last_packet_{};
  Clock::time_point last_frame_{};
};

}  // namespace mc_radar_sdk

#endif  // MC_RADAR_SDK_RADAR_SDK_HPP_
