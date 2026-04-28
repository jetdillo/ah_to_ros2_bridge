#pragma once

// amazinghand_hardware/include/amazinghand_hardware/finger_types.hpp
//
// Core data types shared between the hardware interface and controller.
// A FingerConfig describes the physical wiring of one finger.
// A FingerState holds live position/velocity/effort for one finger,
// expressed in *semantic* axes (flexion, abduction) — not raw servo angles.

#include <string>
#include <cstdint>

namespace amazinghand {

// ---------------------------------------------------------------------------
// Chirality
// Determines the sign of the abduction mixing term.
// For a right hand, positive abduction spreads the finger away from the thumb.
// For a left hand, the same physical motion requires the opposite sign.
// ---------------------------------------------------------------------------
enum class Chirality { RIGHT, LEFT };

// ---------------------------------------------------------------------------
// FingerConfig
// Populated from YAML at startup.  Read-only at runtime.
// ---------------------------------------------------------------------------
struct FingerConfig {
  int         id;           // 0-based finger index within the hand
  std::string name;         // colloquial name: "index", "middle", "ring", "thumb"
  Chirality   chirality;    // RIGHT or LEFT
  uint8_t     leader_id;    // SCS0009 servo bus ID — leader motor
  uint8_t     follower_id;  // SCS0009 servo bus ID — follower motor

  // Derived joint names (built once at init, cached here for convenience)
  std::string flexion_joint_name;    // e.g. "right_hand_index_flexion"
  std::string abduction_joint_name;  // e.g. "right_hand_index_abduction"
};

// ---------------------------------------------------------------------------
// FingerState
// Live state in semantic axes.  Updated every control cycle.
// ---------------------------------------------------------------------------
struct FingerState {
  // Commanded (written by controller)
  double cmd_flexion    = 0.0;  // rad, + = curl
  double cmd_abduction  = 0.0;  // rad, + = spread (chirality-adjusted)

  // Feedback (read from servos)
  double pos_flexion    = 0.0;
  double pos_abduction  = 0.0;
  double vel_flexion    = 0.0;
  double vel_abduction  = 0.0;
  double eff_flexion    = 0.0;  // estimated from servo load feedback
  double eff_abduction  = 0.0;
};

// ---------------------------------------------------------------------------
// MotorCommand
// The result of applying the mixing matrix to a FingerState command.
// These are the raw position targets written to the two servos.
// ---------------------------------------------------------------------------
struct MotorCommand {
  double leader_pos_rad   = 0.0;
  double follower_pos_rad = 0.0;
};

// ---------------------------------------------------------------------------
// Mixing matrix
//
//   motor_leader   =  flexion + (chirality * abduction)
//   motor_follower = -flexion + (chirality * abduction)
//
// where chirality = +1.0 for RIGHT, -1.0 for LEFT.
//
// Inverse (for state feedback):
//   flexion    = ( leader - follower) / 2
//   abduction  = ( leader + follower) / 2  * chirality
// ---------------------------------------------------------------------------
inline MotorCommand mix(const FingerConfig& cfg, double flexion_rad, double abduction_rad)
{
  const double chirality_sign = (cfg.chirality == Chirality::RIGHT) ? 1.0 : -1.0;
  MotorCommand cmd;
  cmd.leader_pos_rad   =  flexion_rad + chirality_sign * abduction_rad;
  cmd.follower_pos_rad = -flexion_rad + chirality_sign * abduction_rad;
  return cmd;
}

inline void unmix(const FingerConfig& cfg,
                  double leader_rad, double follower_rad,
                  double& flexion_rad, double& abduction_rad)
{
  const double chirality_sign = (cfg.chirality == Chirality::RIGHT) ? 1.0 : -1.0;
  flexion_rad   = (leader_rad - follower_rad) / 2.0;
  abduction_rad = (leader_rad + follower_rad) / 2.0 * chirality_sign;
}

}  // namespace amazinghand
