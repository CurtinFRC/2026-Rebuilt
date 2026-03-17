#include "repulsor3d/TruthSocketReceiver.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>

#include <nlohmann/json.hpp>

#if defined(_WIN32)
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;
#endif

namespace repulsor3d {
namespace {

void CloseSocket(const SocketHandle s) {
#if defined(_WIN32)
  closesocket(s);
#else
  close(s);
#endif
}

bool SetReuseAddr(const SocketHandle s) {
  const int opt = 1;
  return setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt)) == 0;
}

bool SetTimeoutMs(const SocketHandle s, const int timeoutMs) {
#if defined(_WIN32)
  const DWORD timeout = static_cast<DWORD>(timeoutMs);
  return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) == 0;
#else
  timeval tv{};
  tv.tv_sec = timeoutMs / 1000;
  tv.tv_usec = (timeoutMs % 1000) * 1000;
  return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
#endif
}

bool SetNonBlocking(const SocketHandle s) {
#if defined(_WIN32)
  u_long mode = 1;
  return ioctlsocket(s, FIONBIO, &mode) == 0;
#else
  const int flags = fcntl(s, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

bool IsWouldBlock(const int err) {
#if defined(_WIN32)
  return err == WSAEWOULDBLOCK || err == WSAETIMEDOUT;
#else
  return err == EWOULDBLOCK || err == EAGAIN;
#endif
}

bool RecvExact(const SocketHandle s, std::uint8_t* out, const std::size_t count, const std::atomic<bool>& stop) {
  std::size_t readTotal = 0;
  while (readTotal < count && !stop.load()) {
#if defined(_WIN32)
    const int n = recv(s, reinterpret_cast<char*>(out + readTotal), static_cast<int>(count - readTotal), 0);
#else
    const int n = static_cast<int>(recv(s, out + readTotal, count - readTotal, 0));
#endif
    if (n == 0) {
      return false;
    }
    if (n < 0) {
#if defined(_WIN32)
      const int err = WSAGetLastError();
#else
      const int err = errno;
#endif
      if (IsWouldBlock(err)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      return false;
    }
    readTotal += static_cast<std::size_t>(n);
  }
  return readTotal == count;
}

}  // namespace

TruthSocketReceiver::TruthSocketReceiver(std::string host, const int port)
    : host_(std::move(host)), port_(port) {}

TruthSocketReceiver::~TruthSocketReceiver() { Stop(); }

void TruthSocketReceiver::Start() {
  if (running_.exchange(true)) {
    return;
  }
  stop_.store(false);
  thread_ = std::thread(&TruthSocketReceiver::Run, this);
}

void TruthSocketReceiver::Stop() {
  stop_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
  running_.store(false);
}

std::vector<FieldVisionObject> TruthSocketReceiver::Latest() const {
  std::scoped_lock lock(mutex_);
  return latest_;
}

void TruthSocketReceiver::Run() {
#if defined(_WIN32)
  WSADATA wsData;
  if (WSAStartup(MAKEWORD(2, 2), &wsData) != 0) {
    std::cerr << "Truth socket: WSAStartup failed\n";
    return;
  }
#endif

  SocketHandle server = socket(AF_INET, SOCK_STREAM, 0);
  if (server == kInvalidSocket) {
    std::cerr << "Truth socket: failed to create socket\n";
#if defined(_WIN32)
    WSACleanup();
#endif
    return;
  }

  SetReuseAddr(server);
  SetNonBlocking(server);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port_));
  if (host_ == "0.0.0.0" || host_ == "" || host_ == "*") {
    addr.sin_addr.s_addr = INADDR_ANY;
  } else if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
    std::cerr << "Truth socket: invalid host " << host_ << "\n";
    CloseSocket(server);
#if defined(_WIN32)
    WSACleanup();
#endif
    return;
  }

  if (bind(server, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::cerr << "Truth socket: bind failed on " << host_ << ":" << port_ << "\n";
    CloseSocket(server);
#if defined(_WIN32)
    WSACleanup();
#endif
    return;
  }

  if (listen(server, 1) != 0) {
    std::cerr << "Truth socket: listen failed\n";
    CloseSocket(server);
#if defined(_WIN32)
    WSACleanup();
#endif
    return;
  }

  while (!stop_.load()) {
    sockaddr_in clientAddr{};
#if defined(_WIN32)
    int len = sizeof(clientAddr);
#else
    socklen_t len = sizeof(clientAddr);
#endif
    SocketHandle conn = accept(server, reinterpret_cast<sockaddr*>(&clientAddr), &len);
    if (conn == kInvalidSocket) {
#if defined(_WIN32)
      const int err = WSAGetLastError();
#else
      const int err = errno;
#endif
      if (IsWouldBlock(err)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }

    SetTimeoutMs(conn, 250);

    while (!stop_.load()) {
      std::array<std::uint8_t, 4> header{};
      if (!RecvExact(conn, header.data(), header.size(), stop_)) {
        break;
      }

      const std::uint32_t size = (static_cast<std::uint32_t>(header[0]) << 24U) |
                                 (static_cast<std::uint32_t>(header[1]) << 16U) |
                                 (static_cast<std::uint32_t>(header[2]) << 8U) |
                                 static_cast<std::uint32_t>(header[3]);
      if (size == 0U || size > 5'000'000U) {
        break;
      }

      std::vector<std::uint8_t> payload(size);
      if (!RecvExact(conn, payload.data(), payload.size(), stop_)) {
        break;
      }

      try {
        const auto parsed = nlohmann::json::parse(payload.begin(), payload.end());
        const auto& fuel = parsed.contains("fuel") ? parsed.at("fuel") : nlohmann::json::array();

        std::vector<FieldVisionObject> out;
        if (fuel.is_array()) {
          for (std::size_t i = 0; i + 2 < fuel.size(); i += 3) {
            FieldVisionObject obj;
            obj.oid = std::to_string(i / 3);
            obj.type = "fuel";
            obj.x = fuel.at(i + 0).get<double>();
            obj.y = fuel.at(i + 1).get<double>();
            obj.z = fuel.at(i + 2).get<double>();
            out.push_back(obj);
          }
        }

        {
          std::scoped_lock lock(mutex_);
          latest_ = std::move(out);
        }
      } catch (const std::exception&) {
        // Ignore malformed payloads and continue receiving.
      }
    }

    CloseSocket(conn);
  }

  CloseSocket(server);

#if defined(_WIN32)
  WSACleanup();
#endif
}

}  // namespace repulsor3d
