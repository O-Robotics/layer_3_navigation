#pragma once
#include "fusioncore/state.hpp"
#include <memory>
#include <map>
#include <string>

namespace fusioncore {

// Abstract base for UKF motion models.
// Implement predict() to define how a single sigma point propagates over dt seconds.
// The default (ConstantVelocityAcceleration) matches the original behavior.
class MotionModelBase {
public:
  virtual StateVector predict(const StateVector& x, double dt) const = 0;
  virtual ~MotionModelBase() = default;
};

// Original model: position integrates body-frame velocity, velocity integrates
// body-frame acceleration. No kinematic constraints. Suitable for all platforms
// as a generic baseline and required for aerial vehicles.
class ConstantVelocityAcceleration : public MotionModelBase {
public:
  StateVector predict(const StateVector& x, double dt) const override;
};

// Non-holonomic constraint for differential drive, skid-steer, and tracked vehicles.
// Zeros lateral velocity (VY) and lateral acceleration (AY) during prediction so
// the filter does not accumulate false lateral drift between encoder updates.
// The measurement model can still observe non-zero VY during wheel slip.
class DifferentialDrive : public MotionModelBase {
public:
  StateVector predict(const StateVector& x, double dt) const override;
};

// Ackermann (car-like) steering: same lateral constraint as DifferentialDrive.
// wheelbase is stored for future minimum-turning-radius extensions.
class AckermannDrive : public MotionModelBase {
public:
  explicit AckermannDrive(double wheelbase = 0.55) : wheelbase_(wheelbase) {}
  StateVector predict(const StateVector& x, double dt) const override;
private:
  double wheelbase_;
};

// Factory: instantiate a motion model by name with optional params.
// Recognized names:
//   "ConstantVelocityAcceleration", "CVA", "" -> ConstantVelocityAcceleration
//   "DifferentialDrive", "diff_drive"         -> DifferentialDrive
//   "Ackermann", "AckermannDrive", "ackermann" -> AckermannDrive (params: wheelbase)
// Throws std::runtime_error on unknown name.
std::shared_ptr<MotionModelBase> create_motion_model(
  const std::string& name,
  const std::map<std::string, double>& params = {});

} // namespace fusioncore
