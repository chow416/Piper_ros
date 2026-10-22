#include "piper_hardware/piper_can.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace piper_hardware {
namespace {

std::string errnoMessage(const char* what) {
  return std::string(what) + ": " + std::strerror(errno);
}

}  // namespace

PiperCan::~PiperCan() { close(); }

bool PiperCan::open(const std::string& interface_name, std::string* error) {
  close();

  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    *error = errnoMessage("socket(PF_CAN)");
    return false;
  }

  struct ifreq interface_request {};
  // ifr_name is a fixed-size array, so truncation is the only safe behaviour
  // if the caller passes something absurd.
  if (interface_name.size() >= IFNAMSIZ) {
    *error = "CAN interface name is too long: " + interface_name;
    close();
    return false;
  }
  std::strncpy(interface_request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_, SIOCGIFINDEX, &interface_request) < 0) {
    *error = errnoMessage(("ioctl(SIOCGIFINDEX) on " + interface_name).c_str());
    close();
    return false;
  }

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (::bind(socket_, reinterpret_cast<struct sockaddr*>(&address),
             sizeof(address)) < 0) {
    *error = errnoMessage(("bind(" + interface_name + ")").c_str());
    close();
    return false;
  }

  const int flags = ::fcntl(socket_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
    *error = errnoMessage("fcntl(O_NONBLOCK)");
    close();
    return false;
  }

  return true;
}

void PiperCan::close() {
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
  dropped_frames_ = 0;
}

bool PiperCan::tryRead(Frame* frame, std::string* error) {
  if (socket_ < 0) {
    *error = "CAN socket is not open";
    return false;
  }

  struct can_frame native {};
  const ssize_t bytes = ::read(socket_, &native, sizeof(native));
  if (bytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return false;  // Empty queue: not an error.
    }
    if (errno == ENOBUFS) {
      // The arm streams at ~3 kHz, so an undersized receive buffer is the most
      // likely cause. Drop this frame but keep the interface usable.
      ++dropped_frames_;
      return false;
    }
    *error = errnoMessage("read(CAN)");
    return false;
  }
  if (bytes != static_cast<ssize_t>(sizeof(native))) {
    return false;  // Truncated or unexpected payload length; ignore it.
  }

  frame->id = native.can_id & CAN_EFF_MASK;
  frame->length = native.can_dlc > 8 ? 8 : native.can_dlc;
  frame->data.fill(0);
  std::memcpy(frame->data.data(), native.data, frame->length);
  return true;
}

bool PiperCan::write(const Frame& frame, std::string* error) {
  if (socket_ < 0) {
    *error = "CAN socket is not open";
    return false;
  }

  struct can_frame native {};
  native.can_id = frame.id;
  native.can_dlc = frame.length > 8 ? 8 : frame.length;
  std::memcpy(native.data, frame.data.data(), native.can_dlc);

  const ssize_t bytes = ::write(socket_, &native, sizeof(native));
  if (bytes != static_cast<ssize_t>(sizeof(native))) {
    // A full SocketCAN transmit queue reports ENOBUFS (not EAGAIN — EAGAIN is
    // for a full socket buffer). Both are transient: the control loop resends a
    // fresh command a few ms later, so a skipped frame costs nothing.
    // Returning an empty *error marks it as transient for the caller; putting a
    // message here makes PiperSystem::write() abort the hardware component.
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS || errno == EINTR) {
      error->clear();
      return false;
    }
    *error = errnoMessage("write(CAN)");
    return false;
  }
  return true;
}

}  // namespace piper_hardware
