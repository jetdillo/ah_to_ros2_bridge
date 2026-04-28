// amazinghand_hardware/src/scs0009_driver.cpp
//
// SCS0009 serial bus servo driver implementation.
//
// Protocol reference: Feetech SCS series serial bus protocol
//   https://gitee.com/ftservo/fddcode (official SDK in C)
//   Python reference: https://github.com/pollen-robotics/AmazingHand/tree/main/PythonExample
//
// The SCS0009 uses a half-duplex UART bus at 1Mbps.
// All multi-byte values are big-endian.
//
// Packet format (write):
//   0xFF 0xFF  ID  LENGTH  INSTRUCTION  PARAMS...  CHECKSUM
//   CHECKSUM = ~(ID + LENGTH + INSTRUCTION + sum(PARAMS)) & 0xFF
//
// Packet format (read response):
//   0xFF 0xFF  ID  LENGTH  ERROR  PARAMS...  CHECKSUM
//
// Key instructions:
//   0x01  PING
//   0x02  READ_DATA
//   0x03  WRITE_DATA
//   0x83  SYNC_WRITE   (broadcast, no response)
//
// Key register addresses:
//   0x2A (42)  Goal Position    2 bytes  R/W
//   0x38 (56)  Present Position 2 bytes  R
//   0x3A (58)  Present Speed    2 bytes  R
//   0x3C (60)  Present Load     2 bytes  R  (signed, bit15=direction)
//   0x3F (63)  Present Temp     1 byte   R
//   0x28 (40)  Torque Enable    1 byte   R/W  (0=off, 1=on)
//
// TODO: Replace stub implementations with real serial I/O.
//       Recommended approach: use the existing Feetech C SDK as a backend,
//       wrapped via a thin POSIX serial layer (open/read/write/ioctl).
//       The Pollen Robotics Python demo is the best reference for
//       sync-write and bulk-read patterns on this specific hardware.

#include "amazinghand_hardware/scs0009_driver.hpp"

#include <stdexcept>
#include <cstring>

// Silence unused-parameter warnings in stubs
#define STUB_UNUSED(x) (void)(x)

namespace amazinghand {

// ---------------------------------------------------------------------------
// Pimpl — holds platform-specific serial state
// ---------------------------------------------------------------------------
class SCS0009Driver::Impl {
public:
  std::string port;
  int         baud_rate;
  int         fd = -1;   // file descriptor (POSIX) or HANDLE (Windows)
  bool        open = false;

  // TODO: add platform serial I/O here
  //   Linux: termios / open() / read() / write()
  //   Windows: CreateFile / ReadFile / WriteFile
};

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
SCS0009Driver::SCS0009Driver(const std::string& port, int baud_rate)
  : impl_(std::make_unique<Impl>())
{
  impl_->port      = port;
  impl_->baud_rate = baud_rate;
}

SCS0009Driver::~SCS0009Driver()
{
  close();
}

// ---------------------------------------------------------------------------
// open
// ---------------------------------------------------------------------------
bool SCS0009Driver::open()
{
  // TODO: open serial port
  //   impl_->fd = ::open(impl_->port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  //   configure termios for baud_rate, 8N1, half-duplex
  //   set RTS/DTR for bus direction control if required by adapter

  // STUB: pretend success
  impl_->open = true;
  return true;
}

void SCS0009Driver::close()
{
  if (impl_->open) {
    // TODO: ::close(impl_->fd);
    impl_->open = false;
    impl_->fd   = -1;
  }
}

bool SCS0009Driver::is_open() const
{
  return impl_->open;
}

// ---------------------------------------------------------------------------
// write_position  (single servo)
// ---------------------------------------------------------------------------
bool SCS0009Driver::write_position(uint8_t servo_id, uint16_t raw_pos)
{
  STUB_UNUSED(servo_id);
  STUB_UNUSED(raw_pos);

  // TODO: build WRITE_DATA packet targeting register 0x2A (Goal Position)
  //   uint8_t params[4] = {
  //     0x2A,                        // start register
  //     0x02,                        // data length
  //     static_cast<uint8_t>(raw_pos >> 8),
  //     static_cast<uint8_t>(raw_pos & 0xFF),
  //   };
  //   send_packet(servo_id, 0x03, params, 4);

  return true;
}

// ---------------------------------------------------------------------------
// sync_write_positions  (all servos, single bus packet — most efficient)
// ---------------------------------------------------------------------------
bool SCS0009Driver::sync_write_positions(
  const std::vector<uint8_t>&  ids,
  const std::vector<uint16_t>& raw_positions)
{
  if (ids.size() != raw_positions.size()) return false;
  STUB_UNUSED(ids);
  STUB_UNUSED(raw_positions);

  // TODO: build SYNC_WRITE packet (instruction 0x83)
  //   Header: 0xFF 0xFF 0xFE LENGTH 0x83
  //   Payload per servo: ID HI(pos) LO(pos)
  //   This writes Goal Position (0x2A, 2 bytes) to all servos simultaneously.
  //   No response expected from servos on SYNC_WRITE.

  return true;
}

// ---------------------------------------------------------------------------
// read_feedback  (single servo)
// ---------------------------------------------------------------------------
std::optional<ServoFeedback> SCS0009Driver::read_feedback(uint8_t servo_id)
{
  STUB_UNUSED(servo_id);

  // TODO: send READ_DATA request for registers 0x38–0x3F (8 bytes):
  //   Present Position (2), Present Speed (2), Present Load (2), Temp (1), Voltage (1)
  //   Parse response into ServoFeedback struct.

  ServoFeedback fb;
  fb.id          = servo_id;
  fb.position    = SCS0009_CENTER_POS;  // neutral
  fb.velocity    = 0;
  fb.load        = 0;
  fb.temperature = 25;
  fb.valid       = false;  // mark invalid until real comms implemented
  return fb;
}

// ---------------------------------------------------------------------------
// bulk_read_feedback  (all servos)
// ---------------------------------------------------------------------------
std::vector<ServoFeedback>
SCS0009Driver::bulk_read_feedback(const std::vector<uint8_t>& ids)
{
  // TODO: the SCS protocol does not have a true "bulk read" command like
  //   Dynamixel.  Poll each servo sequentially with read_feedback(), or
  //   implement a batched read using the STATUS_RETURN_LEVEL register to
  //   minimise turnaround overhead.
  //   The Pollen Python demo uses sequential polling at 50Hz successfully.

  std::vector<ServoFeedback> results;
  results.reserve(ids.size());
  for (uint8_t id : ids) {
    auto fb = read_feedback(id);
    if (fb.has_value()) {
      results.push_back(*fb);
    } else {
      ServoFeedback empty;
      empty.id    = id;
      empty.valid = false;
      results.push_back(empty);
    }
  }
  return results;
}

// ---------------------------------------------------------------------------
// set_torque
// ---------------------------------------------------------------------------
bool SCS0009Driver::set_torque(uint8_t servo_id, bool enable)
{
  STUB_UNUSED(servo_id);
  STUB_UNUSED(enable);

  // TODO: WRITE_DATA to register 0x28 (Torque Enable), 1 byte, value 0 or 1

  return true;
}

bool SCS0009Driver::set_torque_all(const std::vector<uint8_t>& ids, bool enable)
{
  for (uint8_t id : ids) {
    if (!set_torque(id, enable)) return false;
  }
  return true;
}

}  // namespace amazinghand
