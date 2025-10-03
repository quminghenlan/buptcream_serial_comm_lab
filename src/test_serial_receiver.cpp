#include "serial_comm/serial_receiver.hpp"
#include <memory>

int main() {
  auto serial_receiver = std::make_unique<SerialReceiver>("/dev/ttyUSB1");
  serial_receiver->start();
  return 0;
}