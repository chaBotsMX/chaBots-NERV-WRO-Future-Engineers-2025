#include "SerialPort.h"

bool SerialPort::open_port() {
  close_port();
  puerto_ = ::open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (puerto_ < 0) {
    last_error_ = std::string("open() failed: ") + std::strerror(errno);
    return false;
  }

  struct termios tio{};
  if (tcgetattr(puerto_, &tio) != 0) {
    last_error_ = std::string("tcgetattr failed: ") + std::strerror(errno);
    close_port();
    return false;
  }

  cfmakeraw(&tio);
  // Usa un baud soportado por tu sistema. B2000000 (2 Mbps) suele estar disponible.
  cfsetispeed(&tio, B2000000);
  cfsetospeed(&tio, B2000000);

  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB; // 8N1
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(puerto_, TCSANOW, &tio) != 0) {
    last_error_ = std::string("tcsetattr failed: ") + std::strerror(errno);
    close_port();
    return false;
  }
  tcflush(puerto_, TCIOFLUSH);
  return true;
}

bool SerialPort::write_bytes() {
  size_t total = 0;

  while (total < len) {
    ssize_t n = ::write(puerto_, data + total, len - total);

    if (n < 0 && errno != EAGAIN && errno != WOULDBLOCK)
      return false;

    total += static_cast<size_t>(n);
  }

  return true;
}
