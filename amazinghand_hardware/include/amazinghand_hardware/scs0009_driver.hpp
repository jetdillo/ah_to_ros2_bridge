#pragma once

// amazinghand_hardware/include/amazinghand_hardware/scs0009_driver.hpp
//
// Thin C++ wrapper around the Feetech SCS0009 serial bus protocol.
// This layer speaks raw servo — position in raw units (0-1023), velocity,
// load, temperature.  Unit conversion (raw <-> radians) lives here.
//
// The driver is intentionally decoupled from ROS so it can be unit-tested
// without a live serial port.  Swap the backend by subclassing SerialBackend.

#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <optional>

namespace amazinghand {

// ---------------------------------------------------------------------------
// Raw servo feedback packet
// ---------------------------------------------------------------------------
struct ServoFeedback {
  uint8_t  id;
  uint16_t position;   // 0-1023
  uint16_t velocity;   // raw
  int16_t  load;       // signed, raw
  uint8_t  temperature;
  bool     valid = false;
};

// ---------------------------------------------------------------------------
// SCS0009 constants
// ---------------------------------------------------------------------------
constexpr uint16_t SCS0009_CENTER_POS   = 512;
constexpr uint16_t SCS0009_MAX_POS      = 1023;
constexpr double   SCS0009_RAD_PER_UNIT = (2.0 * 3.14159265358979323846) / 1023.0;

// ---------------------------------------------------------------------------
// Unit conversion helpers
// ---------------------------------------------------------------------------
inline uint16_t rad_to_raw(double rad)
{
  int raw = static_cast<int>(SCS0009_CENTER_POS + rad / SCS0009_RAD_PER_UNIT);
  if (raw < 0)    raw = 0;
  if (raw > 1023) raw = 1023;
  return static_cast<uint16_t>(raw);
}

inline double raw_to_rad(uint16_t raw)
{
  return (static_cast<double>(raw) - SCS0009_CENTER_POS) * SCS0009_RAD_PER_UNIT;
}

// ---------------------------------------------------------------------------
// SCS0009Driver
// ---------------------------------------------------------------------------
class SCS0009Driver {
public:
  explicit SCS0009Driver(const std::string& port, int baud_rate);
  ~SCS0009Driver();

  // Opens the serial port.  Returns false on failure.
  bool open();
  void close();
  bool is_open() const;

  // Write a goal position to a single servo (non-blocking).
  bool write_position(uint8_t servo_id, uint16_t raw_pos);

  // Sync-write goal positions to multiple servos in a single bus packet.
  // ids and positions must be the same length.
  bool sync_write_positions(const std::vector<uint8_t>& ids,
                             const std::vector<uint16_t>& raw_positions);

  // Read back full feedback from a single servo.
  std::optional<ServoFeedback> read_feedback(uint8_t servo_id);

  // Bulk-read feedback from multiple servos.
  // Returns one entry per ID; entry.valid = false if the servo didn't respond.
  std::vector<ServoFeedback> bulk_read_feedback(const std::vector<uint8_t>& ids);

  // Torque enable / disable
  bool set_torque(uint8_t servo_id, bool enable);
  bool set_torque_all(const std::vector<uint8_t>& ids, bool enable);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace amazinghand
