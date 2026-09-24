#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace piper_hardware {

// Non-blocking SocketCAN transport for the Piper arm CAN protocol.
//
// The socket is opened in non-blocking mode on purpose: read()/write() are
// called from the ros2_control update loop, so neither of them may ever block.
class PiperCan {
 public:
  struct Frame {
    uint32_t id{0};
    uint8_t length{0};
    std::array<uint8_t, 8> data{};
  };

  PiperCan() = default;
  ~PiperCan();

  PiperCan(const PiperCan&) = delete;
  PiperCan& operator=(const PiperCan&) = delete;

  // Binds a raw CAN socket to `interface_name` (for example "can0").
  // Returns false and fills `error` when the interface is missing or down.
  bool open(const std::string& interface_name, std::string* error);

  void close();

  [[nodiscard]] bool isOpen() const { return socket_ >= 0; }

  // Forwards to close(), so a socket can always be released the same way.
  void reset() { close(); }

  // Reads one pending frame. Returns false when the queue is empty, which is
  // the normal case and must not be treated as an error.
  bool tryRead(Frame* frame, std::string* error);

  bool write(const Frame& frame, std::string* error);

  // Number of frames dropped because the kernel receive buffer was full.
  [[nodiscard]] uint64_t droppedFrames() const { return dropped_frames_; }

 private:
  int socket_{-1};
  uint64_t dropped_frames_{0};
};

}  // namespace piper_hardware
