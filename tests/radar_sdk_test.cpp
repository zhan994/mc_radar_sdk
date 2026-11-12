#include <mc_radar_sdk/radar_sdk.hpp>

#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

namespace {

using namespace mc_radar_sdk;
using Bytes = std::vector<std::uint8_t>;
using Ms = std::chrono::milliseconds;

void Require(bool condition, const char* expression, int line) {
  if (!condition) {
    throw std::runtime_error(std::string("line ") + std::to_string(line) + ": " + expression);
  }
}
#define CHECK(expression) Require(static_cast<bool>(expression), #expression, __LINE__)

bool Near(float a, float b) { return std::fabs(a - b) < 0.0001f; }

template <typename Predicate>
bool Until(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + Ms(2000);
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(Ms(1));
  }
  return true;
}

void Put16(Bytes& bytes, std::uint16_t value) {
  bytes.push_back(static_cast<std::uint8_t>(value));
  bytes.push_back(static_cast<std::uint8_t>(value >> 8));
}

void Put32(Bytes& bytes, std::uint32_t value) {
  Put16(bytes, static_cast<std::uint16_t>(value));
  Put16(bytes, static_cast<std::uint16_t>(value >> 16));
}

Bytes Packet(MessageId id, const Bytes& payload, std::uint16_t counter = 0) {
  Bytes bytes;
  Put16(bytes, static_cast<std::uint16_t>(id));
  Put16(bytes, static_cast<std::uint16_t>(payload.size()));
  Put16(bytes, counter);
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  Put16(bytes, 0xBEEF); // Arbitrary trailer: CRC algorithm is unspecified.
  return bytes;
}

Bytes Header(std::uint16_t frame, std::uint16_t count, std::uint16_t counter = 0) {
  Bytes bytes;
  Put16(bytes, 3);
  Put16(bytes, frame);
  Put16(bytes, count);
  Put16(bytes, 0x1234);
  return Packet(MessageId::kHeader, bytes, counter);
}

Bytes Point() {
  Bytes bytes;
  Put16(bytes, 42);
  Put16(bytes, 0xA55A);
  Put32(bytes, 1000);
  Put16(bytes, static_cast<std::uint16_t>(-123));
  Put16(bytes, 4500);
  Put16(bytes, 3000);
  Put16(bytes, static_cast<std::uint16_t>(-384));
  Put16(bytes, static_cast<std::uint16_t>(-7));
  bytes.push_back(200);
  bytes.push_back(240);
  Put16(bytes, 65535);
  bytes.push_back(7);
  bytes.push_back(255);
  CHECK(bytes.size() == kDetectionSize);
  return bytes;
}

class Sender {
 public:
  Sender() {
    fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    CHECK(fd_ != -1);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    CHECK(bind(fd_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) == 0);
    socklen_t size = sizeof(local);
    CHECK(getsockname(fd_, reinterpret_cast<sockaddr*>(&local), &size) == 0);
    port = ntohs(local.sin_port);
  }
  ~Sender() { close(fd_); }
  Sender(const Sender&) = delete;
  Sender& operator=(const Sender&) = delete;

  void Send(std::uint16_t destination, const Bytes& bytes) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(destination);
    const ssize_t size = sendto(fd_, bytes.data(), bytes.size(), 0,
                               reinterpret_cast<const sockaddr*>(&address), sizeof(address));
    CHECK(size == static_cast<ssize_t>(bytes.size()));
  }

  std::uint16_t port = 0;

 private:
  int fd_ = -1;
};

RadarConfig LocalConfig() {
  RadarConfig config;
  config.bind_ip = "127.0.0.1";
  config.port = 0;
  return config;
}

void SendEmpty(Sender& sender, std::uint16_t port, std::uint16_t frame) {
  sender.Send(port, Header(frame, 0));
  sender.Send(port, Packet(MessageId::kEnd, {}));
}

void TestRoundTrip() {
  RadarSdk sdk;
  CHECK(sdk.Start(LocalConfig()));
  CHECK(!sdk.GetStatus().connected);
  Sender sender;
  const auto port = sdk.GetStatus().bound_port;
  const std::uint64_t timestamp = 0xFEDCBA9876543210ULL;
  Bytes time_bytes;
  for (unsigned int i = 0; i < 8; ++i) {
    time_bytes.push_back(static_cast<std::uint8_t>(timestamp >> (i * 8)));
  }
  sender.Send(port, Packet(MessageId::kTimestamp, time_bytes));
  sender.Send(port, Header(65535, 2, 65534));
  sender.Send(port, Packet(MessageId::kDetections, Point(), 65535));
  sender.Send(port, Packet(MessageId::kDetections, Point(), 0));
  sender.Send(port, Packet(MessageId::kEnd, {0}, 1));
  FramePtr frame;
  CHECK(sdk.WaitForFrame(frame, Ms(2000)));
  CHECK(frame->header.version == 3 && frame->header.frame_count == 65535);
  CHECK(frame->header.reserved == 0x1234 && frame->header.detection_number == 2);
  CHECK(frame->has_device_timestamp && frame->device_timestamp_us == timestamp);
  CHECK(frame->host_timestamp_ns > 0);
  CHECK(frame->source_ip == "127.0.0.1" && frame->source_port == sender.port);
  CHECK(frame->first_message_counter == 65534 && frame->last_message_counter == 1);
  CHECK(frame->detections.size() == 2);
  const auto& point = frame->detections.front();
  CHECK(point.id == 42 && point.flags == 0xA55A);
  CHECK(Near(point.range, 10) && Near(point.vel, -1.23f));
  CHECK(Near(point.azimuth, 45) && Near(point.elevation, 30));
  CHECK(Near(point.rcs, -1.5f) && point.snr == -7);
  CHECK(point.doppler_confidence == 200 && point.exist_confidence == 240);
  CHECK(point.frame == 65535 && point.beam == 7 && point.ext_counter == 255);
  CHECK(Near(point.position.x, 6.12372436f));
  CHECK(Near(point.position.y, 6.12372436f) && Near(point.position.z, -5));
  CHECK(sdk.GetStatus().connected && sdk.GetStatus().frame_valid);
  CHECK(sdk.GetFrames().empty());

  const FramePtr retained = frame;
  SendEmpty(sender, port, 0);
  CHECK(sdk.WaitForFrame(frame, Ms(2000)));
  CHECK(frame->detections.empty() && !frame->has_device_timestamp);
  CHECK(frame->device_timestamp_us == 0);
  CHECK(!sdk.WaitForFrame(frame, Ms(1)) && !frame);
  sdk.Stop();
  CHECK(retained->detections.size() == 2);
  CHECK(!sdk.GetStatus().running && !sdk.GetStatus().connected);
  CHECK(sdk.Start(LocalConfig()));
  CHECK(sdk.GetStatus().completed_frames == 0);
  CHECK(retained->header.frame_count == 65535);
}

void TestExtendedTimestamp() {
  RadarSdk sdk;
  CHECK(sdk.Start(LocalConfig()));
  Sender sender;
  const auto port = sdk.GetStatus().bound_port;
  // Real MC.TG.45 payload layout: u64 timestamp followed by eight extra bytes.
  // Nonzero extension bytes must not change the decoded timestamp.
  Bytes payload(16, 0xA5);
  const std::uint64_t timestamp = 0x000F016724D23A2FULL;
  for (unsigned int i = 0; i < 8; ++i) {
    payload[i] = static_cast<std::uint8_t>(timestamp >> (8 * i));
  }
  sender.Send(port, Packet(MessageId::kTimestamp, payload));
  SendEmpty(sender, port, 1);
  FramePtr frame;
  CHECK(sdk.WaitForFrame(frame, Ms(2000)));
  CHECK(frame->has_device_timestamp && frame->device_timestamp_us == timestamp);
  CHECK(sdk.GetStatus().invalid_datagrams == 0);
  // The legacy reader specifies a minimum of eight bytes, not an exact size.
  payload.resize(24, 0x5A);
  sender.Send(port, Packet(MessageId::kTimestamp, payload));
  SendEmpty(sender, port, 2);
  CHECK(sdk.WaitForFrame(frame, Ms(2000)));
  CHECK(frame->has_device_timestamp && frame->device_timestamp_us == timestamp);
  CHECK(sdk.GetStatus().invalid_datagrams == 0);
  SendEmpty(sender, port, 3);
  CHECK(sdk.WaitForFrame(frame, Ms(2000)) && !frame->has_device_timestamp);
}

void TestMalformedAndRecovery() {
  RadarSdk sdk;
  auto config = LocalConfig();
  config.frame_timeout = Ms(50);
  CHECK(sdk.Start(config));
  Sender sender;
  const auto port = sdk.GetStatus().bound_port;
  sender.Send(port, {});
  sender.Send(port, Packet(MessageId::kTimestamp, Bytes(7)));
  sender.Send(port, Packet(MessageId::kHeader, Bytes(7)));
  auto bad_length = Header(0, 0);
  bad_length.push_back(0);
  sender.Send(port, bad_length);
  sender.Send(port, Packet(MessageId::kDetections, Bytes(25)));
  // More than the legacy 2048-byte receive buffer, but a valid detection batch.
  sender.Send(port, Header(1, 100));
  Bytes batch;
  const auto point = Point();
  for (int i = 0; i < 100; ++i) {
    batch.insert(batch.end(), point.begin(), point.end());
  }
  sender.Send(port, Packet(MessageId::kDetections, batch));
  sender.Send(port, Packet(MessageId::kEnd, {}));
  FramePtr frame;
  CHECK(sdk.WaitForFrame(frame, Ms(2000)) && frame->detections.size() == 100);
  CHECK(sdk.GetStatus().invalid_datagrams == 5);

  sender.Send(port, Header(2, 2)); // Missing a detection.
  sender.Send(port, Packet(MessageId::kDetections, Point()));
  sender.Send(port, Packet(MessageId::kEnd, {}));
  sender.Send(port, Header(3, 0)); // Too many detections.
  sender.Send(port, Packet(MessageId::kDetections, Point()));
  sender.Send(port, Packet(MessageId::kEnd, {}));
  sender.Send(port, Header(4, 1)); // Replaced by the following header.
  SendEmpty(sender, port, 5);
  CHECK(sdk.WaitForFrame(frame, Ms(2000)) && frame->header.frame_count == 5);
  CHECK(sdk.GetStatus().incomplete_frames == 3);
  CHECK(sdk.GetStatus().invalid_datagrams == 6);

  sender.Send(port, Header(6, 0)); // Missing end marker; count alone is insufficient.
  CHECK(Until([&] { return sdk.GetStatus().incomplete_frames == 4; }));
  sender.Send(port, Packet(MessageId::kEnd, {})); // Late end must not publish frame 6.
  sender.Send(port, Packet(static_cast<MessageId>(0x999), {1, 2}));
  SendEmpty(sender, port, 7);
  CHECK(sdk.WaitForFrame(frame, Ms(2000)) && frame->header.frame_count == 7);
  CHECK(sdk.GetStatus().unknown_messages == 1);
  CHECK(sdk.GetStatus().completed_frames == 3);

  const auto received = sdk.GetStatus().received_datagrams;
  sender.Send(port, Packet(MessageId::kTimestamp, Bytes(8, 1)));
  CHECK(Until([&] { return sdk.GetStatus().received_datagrams == received + 1; }));
  std::this_thread::sleep_for(Ms(80));
  SendEmpty(sender, port, 8);
  CHECK(sdk.WaitForFrame(frame, Ms(2000)) && !frame->has_device_timestamp);
}

void TestQueueAndSources() {
  RadarSdk sdk;
  auto config = LocalConfig();
  config.max_queued_frames = 2;
  config.connection_timeout = Ms(50);
  CHECK(sdk.Start(config));
  Sender sender;
  Sender other;
  const auto port = sdk.GetStatus().bound_port;
  for (int i = 0; i < 10; ++i) {
    SendEmpty(sender, port, static_cast<std::uint16_t>(i));
  }
  CHECK(Until([&] { return sdk.GetStatus().completed_frames == 10; }));
  CHECK(sdk.GetStatus().dropped_frames == 8);
  auto frames = sdk.GetFrames();
  CHECK(frames.size() == 2);
  CHECK(frames[0]->header.frame_count == 8 && frames[1]->header.frame_count == 9);
  SendEmpty(other, port, 99);
  CHECK(Until([&] { return sdk.GetStatus().ignored_datagrams == 2; }));
  CHECK(sdk.GetFrames().empty());
  CHECK(Until([&] { return !sdk.GetStatus().connected && !sdk.GetStatus().frame_valid; }));
  SendEmpty(sender, port, 10);
  FramePtr frame;
  CHECK(sdk.WaitForFrame(frame, Ms(2000)));
  CHECK(sdk.GetStatus().connected && sdk.GetStatus().frame_valid);
  sdk.Stop();

  config.radar_ip = "127.0.0.2";
  CHECK(sdk.Start(config));
  SendEmpty(sender, sdk.GetStatus().bound_port, 11);
  CHECK(Until([&] { return sdk.GetStatus().ignored_datagrams == 2; }));
  CHECK(!sdk.GetStatus().connected);
}

void TestLifecycle() {
  RadarSdk sdk;
  Sender occupied;
  auto config = LocalConfig();
  config.port = occupied.port;
  CHECK(!sdk.Start(config));
  CHECK(!sdk.GetStatus().running && !sdk.GetStatus().last_error.empty());
  config = LocalConfig();
  config.bind_ip = "invalid";
  CHECK(!sdk.Start(config));
  config = LocalConfig();
  config.max_queued_frames = 0;
  CHECK(!sdk.Start(config));
  config = LocalConfig();
  config.frame_timeout = Ms(0);
  CHECK(!sdk.Start(config));
  CHECK(sdk.Start(LocalConfig()));
  CHECK(!sdk.Start(LocalConfig()));

  std::promise<void> waiting;
  auto waiter = std::async(std::launch::async, [&] {
    waiting.set_value();
    FramePtr frame;
    return sdk.WaitForFrame(frame, Ms(5000));
  });
  waiting.get_future().wait();
  const auto before = std::chrono::steady_clock::now();
  sdk.Stop();
  CHECK(std::chrono::steady_clock::now() - before < Ms(500));
  CHECK(waiter.wait_for(Ms(500)) == std::future_status::ready);
  CHECK(!waiter.get());
  sdk.Stop();
  CHECK(sdk.Start(LocalConfig()));
  CHECK(sdk.GetStatus().last_error.empty());
  sdk.Stop();

  // Destroying an idle active receiver must release its port and join its thread.
  std::uint16_t released_port = 0;
  {
    RadarSdk scoped;
    CHECK(scoped.Start(LocalConfig()));
    released_port = scoped.GetStatus().bound_port;
  }
  config = LocalConfig();
  config.port = released_port;
  CHECK(sdk.Start(config));
}

void TestConcurrentConsumers() {
  RadarSdk sdk;
  CHECK(sdk.Start(LocalConfig()));
  Sender sender;
  const auto port = sdk.GetStatus().bound_port;
  std::atomic<bool> consuming{true};
  std::atomic<bool> duplicate{false};
  std::mutex seen_mutex;
  std::set<std::uint16_t> seen;
  const auto collect = [&](const FramePtr& frame) {
    std::lock_guard<std::mutex> lock(seen_mutex);
    if (!seen.insert(frame->header.frame_count).second) {
      duplicate.store(true);
    }
  };
  std::thread waiter([&] {
    while (consuming.load()) {
      FramePtr frame;
      if (sdk.WaitForFrame(frame, Ms(10))) {
        collect(frame);
      }
    }
  });
  std::thread drainer([&] {
    while (consuming.load()) {
      for (const auto& frame : sdk.GetFrames()) {
        collect(frame);
      }
      (void)sdk.GetStatus();
      std::this_thread::yield();
    }
  });
  bool all_completed = true;
  for (std::uint16_t i = 0; i < 100; ++i) {
    SendEmpty(sender, port, i);
    if (!Until([&] { return sdk.GetStatus().completed_frames == i + 1U; })) {
      all_completed = false;
      break;
    }
  }
  const bool all_received = all_completed && Until([&] {
    std::lock_guard<std::mutex> lock(seen_mutex);
    return seen.size() == 100;
  });
  sdk.Stop();
  consuming.store(false);
  waiter.join();
  drainer.join();
  CHECK(all_completed && all_received);
  CHECK(!duplicate.load());
  CHECK(sdk.GetStatus().dropped_frames == 0);
}

void TestConcurrentLifecycle() {
  RadarSdk sdk;
  std::atomic<bool> observing{true};
  std::thread observer([&] {
    while (observing.load()) {
      (void)sdk.GetStatus();
      (void)sdk.GetFrames();
      sdk.ClearFrames();
      FramePtr frame;
      sdk.WaitForFrame(frame, Ms(1));
    }
  });
  const auto restart = [&] {
    for (int i = 0; i < 50; ++i) {
      sdk.Start(LocalConfig());
      sdk.Stop();
    }
  };
  std::thread first(restart);
  std::thread second(restart);
  first.join();
  second.join();
  sdk.Stop();
  observing.store(false);
  observer.join();
  CHECK(!sdk.GetStatus().running);
}

} // namespace

int main() {
  try {
    TestRoundTrip();
    TestExtendedTimestamp();
    TestMalformedAndRecovery();
    TestQueueAndSources();
    TestLifecycle();
    TestConcurrentConsumers();
    TestConcurrentLifecycle();
    std::cout << "UDP parsing, recovery, queue, source isolation, and concurrency checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
