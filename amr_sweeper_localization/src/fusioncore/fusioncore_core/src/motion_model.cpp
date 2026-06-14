#include "fusioncore/motion_model.hpp"
#include <cmath>
#include <stdexcept>

namespace fusioncore {

// Shared kinematics: position + orientation integration, common to all models.
// vy_body overrides the lateral velocity used in position integration.
// Pass x[VY] for the default (no constraint) or 0.0 for non-holonomic models.
static StateVector kinematic_predict(const StateVector& x, double dt, double vy_body)
{
  StateVector x_new = x;

  // Normalize quaternion before use: sigma points may not be exactly unit length.
  double qnorm = std::sqrt(x[QW]*x[QW] + x[QX]*x[QX] + x[QY]*x[QY] + x[QZ]*x[QZ]);
  double qw = x[QW]/qnorm, qx = x[QX]/qnorm, qy = x[QY]/qnorm, qz = x[QZ]/qnorm;
  double vx = x[VX], vz = x[VZ];
  double vy = vy_body;

  // Position: p += R(q) * v * dt
  double R00 = 1 - 2*(qy*qy + qz*qz);
  double R01 =     2*(qx*qy - qw*qz);
  double R02 =     2*(qx*qz + qw*qy);
  double R10 =     2*(qx*qy + qw*qz);
  double R11 = 1 - 2*(qx*qx + qz*qz);
  double R12 =     2*(qy*qz - qw*qx);
  double R20 =     2*(qx*qz - qw*qy);
  double R21 =     2*(qy*qz + qw*qx);
  double R22 = 1 - 2*(qx*qx + qy*qy);

  x_new[X] += dt * (R00*vx + R01*vy + R02*vz);
  x_new[Y] += dt * (R10*vx + R11*vy + R12*vz);
  x_new[Z] += dt * (R20*vx + R21*vy + R22*vz);

  // Orientation: exact quaternion kinematics. q_new = q * exp(omega * dt / 2).
  // WX/WY/WZ are true angular rates (bias subtracted at the measurement level).
  double wx = x[WX], wy = x[WY], wz = x[WZ];
  double omega_mag = std::sqrt(wx*wx + wy*wy + wz*wz);
  double dqw, dqx, dqy, dqz;
  if (omega_mag > 1e-10) {
    double half = 0.5 * omega_mag * dt;
    double s = std::sin(half) / omega_mag;
    dqw = std::cos(half);
    dqx = s * wx; dqy = s * wy; dqz = s * wz;
  } else {
    dqw = 1.0;
    dqx = 0.5*wx*dt; dqy = 0.5*wy*dt; dqz = 0.5*wz*dt;
  }
  // Scale by qnorm so process_model acts as a linear map near identity:
  // this keeps the weighted mean on S³ accurate for the high-rate IMU case.
  x_new[QW] = (qw*dqw - qx*dqx - qy*dqy - qz*dqz) * qnorm;
  x_new[QX] = (qw*dqx + qx*dqw + qy*dqz - qz*dqy) * qnorm;
  x_new[QY] = (qw*dqy - qx*dqz + qy*dqw + qz*dqx) * qnorm;
  x_new[QZ] = (qw*dqz + qx*dqy - qy*dqx + qz*dqw) * qnorm;

  return x_new;
}

StateVector ConstantVelocityAcceleration::predict(const StateVector& x, double dt) const
{
  StateVector x_new = kinematic_predict(x, dt, x[VY]);
  x_new[VX] += dt * x[AX];
  x_new[VY] += dt * x[AY];
  x_new[VZ] += dt * x[AZ];
  return x_new;
}

StateVector DifferentialDrive::predict(const StateVector& x, double dt) const
{
  StateVector x_new = kinematic_predict(x, dt, 0.0);  // no lateral position drift
  x_new[VX] += dt * x[AX];
  x_new[VY]  = 0.0;  // lateral velocity does not grow: reset each predict step
  x_new[AY]  = 0.0;  // lateral acceleration: same
  x_new[VZ] += dt * x[AZ];
  return x_new;
}

StateVector AckermannDrive::predict(const StateVector& x, double dt) const
{
  StateVector x_new = kinematic_predict(x, dt, 0.0);
  x_new[VX] += dt * x[AX];
  x_new[VY]  = 0.0;
  x_new[AY]  = 0.0;
  x_new[VZ] += dt * x[AZ];
  return x_new;
}

std::shared_ptr<MotionModelBase> create_motion_model(
  const std::string& name,
  const std::map<std::string, double>& params)
{
  if (name.empty() || name == "ConstantVelocityAcceleration" || name == "CVA") {
    return std::make_shared<ConstantVelocityAcceleration>();
  }
  if (name == "DifferentialDrive" || name == "diff_drive") {
    return std::make_shared<DifferentialDrive>();
  }
  if (name == "Ackermann" || name == "AckermannDrive" || name == "ackermann") {
    double wb = 0.55;
    auto it = params.find("wheelbase");
    if (it != params.end()) wb = it->second;
    return std::make_shared<AckermannDrive>(wb);
  }
  throw std::runtime_error("FusionCore: unknown motion_model '" + name +
    "'. Valid: ConstantVelocityAcceleration, DifferentialDrive, Ackermann");
}

} // namespace fusioncore
