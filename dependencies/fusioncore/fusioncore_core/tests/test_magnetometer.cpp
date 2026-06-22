#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/magnetometer.hpp"
#include "fusioncore/state.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

// Helper: create and initialize a FusionCore instance with magnetometer enabled
static FusionCore make_fc_with_mag() {
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;  // disable gating so every update fires
  cfg.mag.noise_rad     = 0.05;
  cfg.mag.chi2_threshold = 9.21;
  cfg.mag.declination_rad = 0.0;

  FusionCore fc(cfg);
  State initial;
  initial.P(X, X) = 1.0;
  initial.P(Y, Y) = 1.0;
  fc.init(initial, 0.0);
  return fc;
}

// ─── Test 1: mag_yaw_from_field flat, no tilt, pointing east ─────────────────

TEST(MagnetometerTest, FlatPointingEast) {
  MagParams p;
  // Flat, no roll/pitch, pointing east (yaw=0).
  // Magnetic north is ENU +y direction. With yaw=0 (robot pointing east),
  // m_body = [0, Bh, 0].
  double yaw = mag_yaw_from_field(0.0, 1.0, 0.0, p, 0.0, 0.0);
  EXPECT_NEAR(yaw, 0.0, 1e-9);
}

// ─── Test 2: mag_yaw_from_field flat, no tilt, pointing north ────────────────

TEST(MagnetometerTest, FlatPointingNorth) {
  MagParams p;
  // Pointing north (yaw=pi/2). m_body = [Bh, 0, 0].
  double yaw = mag_yaw_from_field(1.0, 0.0, 0.0, p, 0.0, 0.0);
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-9);
}

// ─── Test 3: mag_yaw_from_field flat, pointing west ──────────────────────────

TEST(MagnetometerTest, FlatPointingWest) {
  MagParams p;
  // Pointing west (yaw=pi). m_body = [0, -Bh, 0].
  double yaw = mag_yaw_from_field(0.0, -1.0, 0.0, p, 0.0, 0.0);
  EXPECT_NEAR(std::abs(yaw), M_PI, 1e-9);
}

// ─── Test 4: Declination offset applied correctly ────────────────────────────

TEST(MagnetometerTest, DeclinationOffset) {
  MagParams p;
  p.declination_rad = 0.1;  // 0.1 rad positive declination
  // Pointing east (yaw=0 true north reference). m_body = [0, Bh, 0].
  double yaw = mag_yaw_from_field(0.0, 1.0, 0.0, p, 0.0, 0.0);
  EXPECT_NEAR(yaw, 0.1, 1e-9);
}

// ─── Test 5: Hard iron correction ────────────────────────────────────────────

TEST(MagnetometerTest, HardIronCorrection) {
  MagParams p;
  p.hard_iron = Eigen::Vector3d(0.5, 0.3, 0.1);
  // Raw reading: true field [0, 1, 0] + hard iron bias
  double mx = 0.0 + 0.5;
  double my = 1.0 + 0.3;
  double mz = 0.0 + 0.1;
  double yaw = mag_yaw_from_field(mx, my, mz, p, 0.0, 0.0);
  EXPECT_NEAR(yaw, 0.0, 1e-9);
}

// ─── Test 6: Tilt compensation (pitched forward) preserves yaw ───────────────

TEST(MagnetometerTest, TiltCompensationPitch) {
  MagParams p;
  // Robot pointing east (yaw=0), pitched forward 20 deg.
  // m_world = [0, Bh=1, -Bv=0] (horizontal only for simplicity).
  // m_body = Ry(-pitch) * [0, 1, 0] = [sin(pitch)*0 + cos(pitch)*0, 1, 0] ... let me compute:
  // R^T = Rz(0)^T * Ry(-pitch) = Ry(-pitch) for yaw=0
  // Ry(-p)*[0,1,0] = [sin(p)*0+cos(p)*0 + ... ]
  // Actually: Ry(-p) = [[cos(p),0,-sin(p)],[0,1,0],[sin(p),0,cos(p)]]
  // m_body = [0*cos(p)+1*0+0*(-sin(p)), 0*0+1*1+0*0, 0*sin(p)+1*0+0*cos(p)]
  //        = [0, 1, 0] -- no change since m_world is purely y
  // So tilt compensation should also give yaw = 0
  double pitch = 20.0 * M_PI / 180.0;
  double yaw = mag_yaw_from_field(0.0, 1.0, 0.0, p, 0.0, pitch);
  EXPECT_NEAR(yaw, 0.0, 1e-6);
}

// ─── Test 7: update_magnetometer moves yaw toward measurement ────────────────

TEST(MagnetometerTest, UpdateMovesYawTowardMeasurement) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection = false;
  cfg.mag.noise_rad     = 0.05;

  FusionCore fc(cfg);
  State initial;
  // Initialize with large yaw uncertainty so the filter accepts magnetometer corrections.
  // Default P[QW/QZ] is 1e-8, which would prevent convergence in any reasonable iteration count.
  initial.P(QW, QW) = 0.5;
  initial.P(QZ, QZ) = 0.5;
  fc.init(initial, 0.0);

  for (int i = 0; i < 20; ++i) {
    fc.update_imu(0.01 * i, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);
  }

  // yaw=pi/4 (northeast): mx=sin(pi/4), my=cos(pi/4) -> atan2(mx,my)=pi/4
  double target_yaw = M_PI / 4.0;
  double mx = std::sin(target_yaw);
  double my = std::cos(target_yaw);

  for (int i = 0; i < 300; ++i) {
    fc.update_magnetometer(0.2 + 0.01 * i, mx, my, 0.0);
  }

  const State& s = fc.get_state();
  double roll, pitch, yaw;
  quat_to_euler(s.x[QW], s.x[QX], s.x[QY], s.x[QZ], roll, pitch, yaw);
  // Tolerance is 3x noise_rad: UKF residual after many updates without
  // interleaved IMU updates is bounded by the measurement noise floor.
  EXPECT_NEAR(yaw, target_yaw, 0.15);
}

// ─── Test 8: update_magnetometer returns true on acceptance ──────────────────

TEST(MagnetometerTest, UpdateReturnsTrueOnAcceptance) {
  FusionCore fc = make_fc_with_mag();
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  bool accepted = fc.update_magnetometer(0.01, 0.0, 1.0, 0.0);
  EXPECT_TRUE(accepted);
}

// ─── Test 9: Chi-squared gate rejects outlier ────────────────────────────────

TEST(MagnetometerTest, Chi2GateRejectsOutlier) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection  = true;
  cfg.mag.noise_rad      = 0.001;   // very tight: any deviation is an outlier
  cfg.mag.chi2_threshold = 9.21;

  FusionCore fc(cfg);
  State initial;
  fc.init(initial, 0.0);
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  // Filter is at yaw=0. Feed a pi/2 reading with noise_rad=0.001: huge outlier.
  bool accepted = fc.update_magnetometer(0.01, 1.0, 0.0, 0.0);  // yaw=pi/2
  EXPECT_FALSE(accepted);
}

// ─── Test 10: heading_source set to MAGNETOMETER after first update ───────────

TEST(MagnetometerTest, HeadingSourceSetAfterUpdate) {
  FusionCore fc = make_fc_with_mag();
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  EXPECT_FALSE(fc.is_heading_valid());
  EXPECT_EQ(fc.heading_source(), HeadingSource::NONE);

  fc.update_magnetometer(0.01, 0.0, 1.0, 0.0);

  EXPECT_TRUE(fc.is_heading_valid());
  EXPECT_EQ(fc.heading_source(), HeadingSource::MAGNETOMETER);
}

// ─── Test 11: DUAL_ANTENNA heading not overridden by magnetometer ─────────────

TEST(MagnetometerTest, DualAntennaNotOverriddenByMag) {
  FusionCore fc = make_fc_with_mag();
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  // Set heading source to DUAL_ANTENNA manually via update_gnss_heading
  sensors::GnssHeading hdg;
  hdg.heading_rad  = 0.0;
  hdg.accuracy_rad = 0.01;
  hdg.valid        = true;
  fc.update_gnss_heading(0.01, hdg);

  EXPECT_EQ(fc.heading_source(), HeadingSource::DUAL_ANTENNA);

  // Now fuse a magnetometer update
  fc.update_magnetometer(0.02, 0.0, 1.0, 0.0);

  // Source must remain DUAL_ANTENNA
  EXPECT_EQ(fc.heading_source(), HeadingSource::DUAL_ANTENNA);
}

// ─── Test 12: mag_outliers incremented on rejection ───────────────────────────

TEST(MagnetometerTest, OutlierCounterIncremented) {
  FusionCoreConfig cfg;
  cfg.outlier_rejection  = true;
  cfg.mag.noise_rad      = 0.001;
  cfg.mag.chi2_threshold = 9.21;

  FusionCore fc(cfg);
  State initial;
  fc.init(initial, 0.0);
  fc.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665);

  fc.update_magnetometer(0.01, 1.0, 0.0, 0.0);  // outlier

  auto status = fc.get_status();
  EXPECT_GT(status.mag_outliers, 0);
}
