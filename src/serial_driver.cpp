#include "armor_track_ros2/serial_driver.hpp"

#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

#include <termios.h>
#include <sys/ioctl.h>

#include <iostream>

namespace serial_driver
{

SerialDriver::~SerialDriver()
{
  close();
}

bool SerialDriver::open(const std::string & port, int baud)
{
  close();

  fd_ = openPort(port.c_str());
  if (fd_ < 0) {
    std::cerr << "[serial] 打开串口失败: " << port << " (" << strerror(errno) << ")"
              << std::endl;
    return false;
  }

  configurePort(fd_, baud);
  return true;
}

void SerialDriver::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialDriver::send(const SendPacket & packet)
{
  if (fd_ < 0) return false;

  auto bytes = reinterpret_cast<const uint8_t *>(&packet);
  ssize_t n = ::write(fd_, bytes, sizeof(SendPacket));
  return n == static_cast<ssize_t>(sizeof(SendPacket));
}

bool SerialDriver::poll()
{
  if (fd_ < 0) return false;

  int available = 0;
  if (ioctl(fd_, FIONREAD, &available) != 0) return false;
  if (available <= 0) return false;

  rx_buf_.resize(static_cast<size_t>(available));
  ssize_t n = ::read(fd_, rx_buf_.data(), available);
  return n > 0;
}

int SerialDriver::openPort(const char * port)
{
  int fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) return -1;
  // 设为非阻塞读
  fcntl(fd, F_SETFL, FNDELAY);
  return fd;
}

int SerialDriver::configurePort(int fd, int baud)
{
  struct termios port_settings;
  memset(&port_settings, 0, sizeof(port_settings));

  speed_t speed = B460800;
  switch (baud) {
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default: speed = B460800; break;
  }

  cfsetispeed(&port_settings, speed);
  cfsetospeed(&port_settings, speed);

  port_settings.c_cflag |= (CLOCAL | CREAD);
  port_settings.c_cflag &= ~PARENB;
  port_settings.c_cflag &= ~PARODD;
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;

  port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
  port_settings.c_iflag &= ~(INLCR | IGNCR | ICRNL);
  port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  port_settings.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP);
  port_settings.c_oflag &= ~OPOST;

  port_settings.c_cc[VTIME] = 0;
  port_settings.c_cc[VMIN] = 0;

  tcsetattr(fd, TCSANOW, &port_settings);
  return fd;
}

}  // namespace serial_driver
