#include "hound_core/ntrip_runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sstream>
#include <utility>

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

namespace hound_core
{
namespace
{

constexpr uint16_t kDefaultPort = 2101;
constexpr size_t kMaxAcc = 8192;
constexpr int kConnectTimeoutMs = 5000;
constexpr int kPollMs = 500;

uint32_t crc24q(const uint8_t * data, size_t len)
{
  uint32_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int b = 0; b < 8; ++b) {
      crc <<= 1;
      if (crc & 0x1000000u) {
        crc ^= 0x1864CFBu;
      }
    }
  }
  return crc & 0xFFFFFFu;
}

bool rtcm3_ok(const uint8_t * frame, size_t n)
{
  if (n < 6 || frame[0] != 0xD3) {
    return false;
  }
  const uint16_t payload = static_cast<uint16_t>(((frame[1] & 0x03) << 8) | frame[2]);
  if (n != static_cast<size_t>(3 + payload + 3)) {
    return false;
  }
  const uint32_t got = (static_cast<uint32_t>(frame[n - 3]) << 16) |
    (static_cast<uint32_t>(frame[n - 2]) << 8) |
    static_cast<uint32_t>(frame[n - 1]);
  return got == crc24q(frame, n - 3);
}

std::string b64(const std::string & in)
{
  static constexpr char kTbl[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((in.size() + 2) / 3) * 4);
  int val = 0;
  int valb = -6;
  for (unsigned char c : in) {
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      out.push_back(kTbl[(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }
  if (valb > -6) {
    out.push_back(kTbl[((val << 8) >> (valb + 8)) & 0x3F]);
  }
  while (out.size() % 4) {
    out.push_back('=');
  }
  return out;
}

void split_server(const std::string & server, std::string & host, std::string & port)
{
  host = server;
  port = std::to_string(kDefaultPort);
  const auto colon = server.rfind(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 >= server.size()) {
    return;
  }
  // Skip IPv6 literals without brackets (no extra colon besides port).
  if (server.find(':') != colon) {
    return;
  }
  host = server.substr(0, colon);
  port = server.substr(colon + 1);
}

std::string nmea_checksum(const std::string & body)
{
  uint8_t cs = 0;
  for (unsigned char c : body) {
    cs ^= c;
  }
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%02X", cs);
  return buf;
}

int nmea_quality(uint8_t fix_type)
{
  // MAVLink GPS_FIX_TYPE → NMEA GGA quality.
  switch (fix_type) {
    case 4: return 2;   // DGPS
    case 5: return 5;   // RTK float
    case 6: return 4;   // RTK fixed
    default: return (fix_type >= 2) ? 1 : 0;
  }
}

void deg_to_nmea(double deg, bool is_lat, int & dd, double & mm, char & hemi)
{
  hemi = is_lat ? (deg >= 0.0 ? 'N' : 'S') : (deg >= 0.0 ? 'E' : 'W');
  deg = std::fabs(deg);
  dd = static_cast<int>(deg);
  mm = (deg - static_cast<double>(dd)) * 60.0;
}

int connect_tcp(const std::string & host, const std::string & port, std::string & err)
{
  addrinfo hints{};
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_family = AF_UNSPEC;
  addrinfo * res = nullptr;
  const int rc = getaddrinfo(host.c_str(), port.c_str(), &hints, &res);
  if (rc != 0) {
    err = gai_strerror(rc);
    return -1;
  }

  int fd = -1;
  for (addrinfo * ai = res; ai != nullptr; ai = ai->ai_next) {
    fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (fd < 0) {
      continue;
    }
    const int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    timeval tv{};
    tv.tv_sec = kConnectTimeoutMs / 1000;
    tv.tv_usec = (kConnectTimeoutMs % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) {
      timeval none{};
      setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &none, sizeof(none));
      setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &none, sizeof(none));
      break;
    }
    ::close(fd);
    fd = -1;
  }
  freeaddrinfo(res);
  if (fd < 0) {
    err = "connect failed";
  }
  return fd;
}

bool write_all(int fd, const char * p, size_t n)
{
  size_t off = 0;
  while (off < n) {
    const ssize_t w = ::send(fd, p + off, n - off, MSG_NOSIGNAL);
    if (w <= 0) {
      return false;
    }
    off += static_cast<size_t>(w);
  }
  return true;
}

}  // namespace

NtripRunner::NtripRunner(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

NtripRunner::~NtripRunner()
{
  stop();
}

void NtripRunner::start(FcuBus & bus, const Config & config)
{
  stop();
  cfg_ = config;
  bus_ = &bus;
  split_server(cfg_.server, host_, port_);
  if (host_.empty()) {
    RCLCPP_WARN(logger_, "NTRIP enabled but server empty — not starting");
    bus_ = nullptr;
    return;
  }
  if (cfg_.mountpoint.empty()) {
    RCLCPP_WARN(logger_, "NTRIP mountpoint empty — caster will send sourcetable");
  }
  acc_.clear();
  frames_ = 0;
  bytes_ = 0;
  running_.store(true);
  thread_ = std::thread([this]() {loop();});
  RCLCPP_INFO(
    logger_, "NTRIP client: %s:%s /%s gga=%s",
    host_.c_str(), port_.c_str(), cfg_.mountpoint.c_str(), cfg_.gga.c_str());
}

void NtripRunner::stop()
{
  running_.store(false);
  close_fd();
  if (thread_.joinable()) {
    thread_.join();
  }
  bus_ = nullptr;
  acc_.clear();
}

void NtripRunner::close_fd()
{
  const int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
}

void NtripRunner::loop()
{
  double backoff = std::max(0.5, cfg_.reconnect_s);
  while (running_.load(std::memory_order_relaxed)) {
    if (run_session()) {
      backoff = std::max(0.5, cfg_.reconnect_s);
    }
    if (!running_.load(std::memory_order_relaxed)) {
      break;
    }
    RCLCPP_WARN(
      logger_, "NTRIP reconnect in %.1fs (frames=%llu bytes=%llu dropped=%llu)",
      backoff, static_cast<unsigned long long>(frames_),
      static_cast<unsigned long long>(bytes_),
      static_cast<unsigned long long>(bus_ ? bus_->rtcm.dropped() : 0));
    const auto until = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(backoff);
    while (running_.load(std::memory_order_relaxed) &&
      std::chrono::steady_clock::now() < until)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    backoff = std::min(backoff * 2.0, 30.0);
  }
}

bool NtripRunner::run_session()
{
  acc_.clear();
  std::string err;
  const int fd = connect_tcp(host_, port_, err);
  if (fd < 0) {
    RCLCPP_WARN(logger_, "NTRIP connect %s:%s: %s", host_.c_str(), port_.c_str(), err.c_str());
    return false;
  }
  fd_.store(fd);

  std::string mount = cfg_.mountpoint;
  if (!mount.empty() && mount.front() == '/') {
    mount.erase(mount.begin());
  }

  std::ostringstream req;
  req << "GET /" << mount << " HTTP/1.0\r\n"
      << "Host: " << host_ << ":" << port_ << "\r\n"
      << "Ntrip-Version: Ntrip/2.0\r\n"
      << "User-Agent: NTRIP hound_core\r\n";
  if (!cfg_.user.empty()) {
    req << "Authorization: Basic " << b64(cfg_.user + ":" + cfg_.password) << "\r\n";
  }
  req << "\r\n";
  const std::string header = req.str();
  if (!write_all(fd, header.data(), header.size())) {
    RCLCPP_WARN(logger_, "NTRIP request send failed");
    close_fd();
    return false;
  }

  auto recv_byte = [&](char & out) -> bool {
      while (running_.load(std::memory_order_relaxed)) {
        pollfd pfd{fd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, kPollMs);
        if (pr == 0) {
          continue;
        }
        if (pr < 0 || !(pfd.revents & POLLIN)) {
          return false;
        }
        const ssize_t n = ::recv(fd, &out, 1, 0);
        return n == 1;
      }
      return false;
    };

  auto recv_line = [&](std::string & line) -> bool {
      line.clear();
      char c = 0;
      while (line.size() < 512) {
        if (!recv_byte(c)) {
          return false;
        }
        line.push_back(c);
        if (line.size() >= 2 && line.compare(line.size() - 2, 2, "\r\n") == 0) {
          return true;
        }
      }
      return false;
    };

  std::string line;
  if (!recv_line(line)) {
    close_fd();
    return false;
  }
  if (line.find("SOURCETABLE") != std::string::npos) {
    RCLCPP_WARN(logger_, "NTRIP got sourcetable (check mountpoint)");
    close_fd();
    return false;
  }
  const bool icy = line.compare(0, 11, "ICY 200 OK") == 0;
  const bool http_ok = line.find(" 200 ") != std::string::npos;
  if (!icy && !http_ok) {
    RCLCPP_WARN(logger_, "NTRIP caster rejected: %s", line.c_str());
    close_fd();
    return false;
  }
  // HTTP/1.x has more headers; NTRIP v1 ICY often starts RTCM on the next byte.
  if (!icy) {
    std::string head = line;
    while (head.size() < 4096) {
      if (!recv_line(line)) {
        close_fd();
        return false;
      }
      head += line;
      if (line == "\r\n") {
        break;
      }
    }
    if (head.find("chunked") != std::string::npos) {
      RCLCPP_WARN(logger_, "NTRIP Transfer-Encoding: chunked is not supported");
      close_fd();
      return false;
    }
  }
  RCLCPP_INFO(logger_, "NTRIP connected %s:%s /%s", host_.c_str(), port_.c_str(), mount.c_str());

  last_gga_ = std::chrono::steady_clock::time_point{};
  if (cfg_.gga != "none") {
    send_gga(fd);
  }

  uint8_t buf[1024];
  while (running_.load(std::memory_order_relaxed)) {
    if (cfg_.gga != "none" && cfg_.gga_period_s > 0.0) {
      const auto now = std::chrono::steady_clock::now();
      if (last_gga_.time_since_epoch().count() == 0 ||
        (now - last_gga_) >= std::chrono::duration<double>(cfg_.gga_period_s))
      {
        if (!send_gga(fd)) {
          break;
        }
      }
    }

    pollfd pfd{fd, POLLIN, 0};
    const int pr = ::poll(&pfd, 1, kPollMs);
    if (pr == 0) {
      continue;
    }
    if (pr < 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      break;
    }
    const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) {
      break;
    }
    feed_bytes(buf, static_cast<size_t>(n));
  }

  close_fd();
  return frames_ > 0;
}

bool NtripRunner::send_gga(int fd)
{
  const std::string gga = build_gga();
  if (gga.empty()) {
    return true;
  }
  if (!write_all(fd, gga.data(), gga.size())) {
    return false;
  }
  last_gga_ = std::chrono::steady_clock::now();
  return true;
}

std::string NtripRunner::build_gga() const
{
  if (cfg_.gga == "none") {
    return {};
  }
  if (cfg_.gga == "static" && !cfg_.static_gga.empty()) {
    std::string s = cfg_.static_gga;
    if (s.back() != '\n') {
      s += "\r\n";
    }
    return s;
  }

  double lat = cfg_.origin_lat;
  double lon = cfg_.origin_lon;
  double alt = cfg_.origin_alt;
  int qual = 1;
  int sats = 0;
  double hdop = 1.0;
  if (bus_) {
    GpsSample gps;
    if (bus_->gps.copy_latest(gps) && gps.fix_type >= 2 &&
      (gps.lat_deg != 0.0 || gps.lon_deg != 0.0))
    {
      lat = gps.lat_deg;
      lon = gps.lon_deg;
      alt = gps.alt_m;
      qual = nmea_quality(gps.fix_type);
      sats = gps.satellites_visible;
      if (gps.eph_m > 0.0f) {
        hdop = std::max(0.5, static_cast<double>(gps.eph_m));
      }
    }
  }

  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&tt, &tm);

  int lat_dd = 0, lon_dd = 0;
  double lat_mm = 0, lon_mm = 0;
  char ns = 'N', ew = 'E';
  deg_to_nmea(lat, true, lat_dd, lat_mm, ns);
  deg_to_nmea(lon, false, lon_dd, lon_mm, ew);

  char body[160];
  std::snprintf(
    body, sizeof(body),
    "GPGGA,%02d%02d%02d.00,%02d%010.7f,%c,%03d%010.7f,%c,%d,%02d,%.1f,%.1f,M,0.0,M,,",
    tm.tm_hour, tm.tm_min, tm.tm_sec,
    lat_dd, lat_mm, ns, lon_dd, lon_mm, ew,
    qual, sats, hdop, alt);
  return std::string("$") + body + "*" + nmea_checksum(body) + "\r\n";
}

void NtripRunner::feed_bytes(const uint8_t * data, size_t n)
{
  if (bus_ == nullptr || data == nullptr || n == 0) {
    return;
  }
  acc_.insert(acc_.end(), data, data + n);
  if (acc_.size() > kMaxAcc) {
    acc_.erase(acc_.begin(), acc_.begin() + static_cast<std::ptrdiff_t>(acc_.size() - kMaxAcc));
  }

  while (true) {
    auto it = std::find(acc_.begin(), acc_.end(), static_cast<uint8_t>(0xD3));
    if (it != acc_.begin()) {
      acc_.erase(acc_.begin(), it);
    }
    if (acc_.size() < 3) {
      return;
    }
    if ((acc_[1] & 0xFC) != 0) {
      acc_.erase(acc_.begin());
      continue;
    }
    const uint16_t payload =
      static_cast<uint16_t>(((acc_[1] & 0x03) << 8) | acc_[2]);
    const size_t need = 3u + static_cast<size_t>(payload) + 3u;
    if (acc_.size() < need) {
      return;
    }
    if (!rtcm3_ok(acc_.data(), need)) {
      acc_.erase(acc_.begin());
      continue;
    }
    std::vector<uint8_t> frame(acc_.begin(), acc_.begin() + static_cast<std::ptrdiff_t>(need));
    acc_.erase(acc_.begin(), acc_.begin() + static_cast<std::ptrdiff_t>(need));
    bytes_ += frame.size();
    ++frames_;
    bus_->rtcm.push(std::move(frame));
    if (frames_ == 1) {
      RCLCPP_INFO(logger_, "NTRIP first RTCM3 frame (%u B)", static_cast<unsigned>(need));
    }
  }
}

}  // namespace hound_core
