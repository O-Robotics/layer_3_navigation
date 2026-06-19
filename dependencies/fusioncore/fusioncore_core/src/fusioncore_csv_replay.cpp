#include "fusioncore/fusioncore.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using fusioncore::FusionCore;
using fusioncore::FusionCoreConfig;
using fusioncore::State;
using fusioncore::StateMatrix;
using fusioncore::sensors::ECEFPoint;
using fusioncore::sensors::GnssFix;
using fusioncore::sensors::GnssFixType;
using fusioncore::sensors::LLAPoint;
using fusioncore::sensors::ecef_to_enu;

struct CsvRow {
  std::string type;
  double receive_time = 0.0;
  double stamp = 0.0;
  double values[19] = {0.0};
};

struct PoseSample {
  double stamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double pxx = 0.0;
  double pyy = 0.0;
  double pyaw = 0.0;
  std::string source;
};

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

ECEFPoint lla_to_ecef(double lat_rad, double lon_rad, double alt_m) {
  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  constexpr double e2 = f * (2.0 - f);

  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  const double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

  ECEFPoint p;
  p.x = (N + alt_m) * cos_lat * cos_lon;
  p.y = (N + alt_m) * cos_lat * sin_lon;
  p.z = (N * (1.0 - e2) + alt_m) * sin_lat;
  return p;
}

CsvRow parse_row(const std::string& line) {
  CsvRow row;
  std::stringstream ss(line);
  std::string field;
  std::vector<std::string> cols;
  while (std::getline(ss, field, ',')) cols.push_back(field);
  if (cols.size() < 22) {
    throw std::runtime_error("Malformed CSV row: " + line);
  }
  row.type = cols[0];
  row.receive_time = std::stod(cols[1]);
  row.stamp = std::stod(cols[2]);
  for (int i = 0; i < 19; ++i) {
    row.values[i] = std::stod(cols[3 + i]);
  }
  return row;
}

class ReplayHarness {
public:
  ReplayHarness(const FusionCoreConfig& config, bool use_transformed_init_window)
  : config_(config), fc_(config), use_transformed_init_window_(use_transformed_init_window) {}

  void process(const CsvRow& row) {
    if (row.type == "imu") {
      process_imu(row);
    } else if (row.type == "encoder") {
      process_encoder(row);
    } else if (row.type == "gnss") {
      process_gnss(row);
    }
  }

  const std::vector<PoseSample>& samples() const { return samples_; }

private:
  FusionCoreConfig config_;
  FusionCore fc_;
  bool pending_init_ = true;
  double filter_init_time_ = 0.0;
  bool gnss_ref_set_ = false;
  LLAPoint gnss_ref_lla_{};
  ECEFPoint gnss_ref_ecef_{};

  double last_primary_encoder_time_ = 0.0;
  double last_primary_encoder_speed_ = 0.0;
  double last_primary_encoder_wz_ = 0.0;
  double last_primary_imu_time_ = 0.0;
  double last_primary_imu_angular_speed_ = 0.0;
  double last_primary_imu_dynamic_accel_norm_ = 0.0;

  bool stationary_transition_initialized_ = false;
  bool last_stationary_context_ = false;
  double stationary_transition_start_time_ = 0.0;
  double stationary_transition_start_scale_ = 1.0;

  double init_window_start_ = 0.0;
  bool init_window_collecting_ = false;
  bool use_transformed_init_window_ = false;
  int init_win_n_ = 0;
  int init_win_orient_n_ = 0;
  double init_win_wx_ = 0.0;
  double init_win_wy_ = 0.0;
  double init_win_wz_ = 0.0;
  double init_win_ax_ = 0.0;
  double init_win_ay_ = 0.0;
  double init_win_az_ = 0.0;
  double init_win_qw_ = 0.0;
  double init_win_qx_ = 0.0;
  double init_win_qy_ = 0.0;
  double init_win_qz_ = 0.0;

  std::vector<PoseSample> samples_;

  void process_imu(const CsvRow& row) {
    const double t = row.stamp;
    const double wx = row.values[0];
    const double wy = row.values[1];
    const double wz = row.values[2];
    const double ax = row.values[3];
    const double ay = row.values[4];
    const double az = row.values[5];
    const double qw = row.values[6];
    const double qx = row.values[7];
    const double qy = row.values[8];
    const double qz = row.values[9];

    last_primary_imu_time_ = t;

    if (pending_init_) {
      collect_init_window(row);
      return;
    }

    fc_.update_imu(t, wx, wy, wz, ax, ay, az);
    remember_primary_imu_motion_sample(t, wx, wy, wz, ax, ay, az);

    double orientation_cov[9] = {
      row.values[10], 0.0, 0.0,
      0.0, row.values[11], 0.0,
      0.0, 0.0, row.values[12]
    };
    double roll, pitch, yaw;
    fusioncore::quat_to_euler(qw, qx, qy, qz, roll, pitch, yaw);
    fc_.update_imu_orientation(t, roll, pitch, yaw, orientation_cov);
    record_sample(t, "imu");
  }

  void process_encoder(const CsvRow& row) {
    if (pending_init_ || !fc_.is_initialized()) return;

    const double t = row.stamp;
    const double vx = row.values[0];
    const double vy = row.values[1];
    const double wz = row.values[2];

    last_primary_encoder_time_ = t;
    last_primary_encoder_speed_ = std::sqrt(vx * vx + vy * vy);
    last_primary_encoder_wz_ = wz;

    fc_.update_encoder(t, vx, vy, wz, row.values[3], row.values[4], row.values[5]);
    fc_.update_ground_constraint(t);
    if (last_primary_encoder_speed_ < config_.zupt_velocity_threshold &&
        std::abs(last_primary_encoder_wz_) < config_.zupt_angular_threshold) {
      fc_.update_zupt(t, config_.zupt_noise_sigma);
    }
    record_sample(t, "encoder");
  }

  void process_gnss(const CsvRow& row) {
    if (pending_init_ || !fc_.is_initialized()) return;

    const double t = row.stamp;
    const double lat_rad = deg2rad(row.values[0]);
    const double lon_rad = deg2rad(row.values[1]);
    const double alt_m = row.values[2];

    LLAPoint lla{lat_rad, lon_rad, alt_m};
    ECEFPoint ecef = lla_to_ecef(lat_rad, lon_rad, alt_m);
    if (!gnss_ref_set_) {
      gnss_ref_set_ = true;
      gnss_ref_lla_ = lla;
      gnss_ref_ecef_ = ecef;
    }
    const auto enu = ecef_to_enu(ecef, gnss_ref_ecef_, gnss_ref_lla_);

    GnssFix fix;
    fix.x = enu[0];
    fix.y = enu[1];
    fix.z = enu[2];
    fix.fix_type = static_cast<int>(row.values[14]) >= 2 ? GnssFixType::RTK_FIXED :
      (static_cast<int>(row.values[14]) == 1 ? GnssFixType::DGPS_FIX : GnssFixType::GPS_FIX);
    fix.source_id = 0;
    double covariance_scale = 400.0;

    const bool stationary_context = is_gnss_stationary_context(t);
    covariance_scale *= stationary_gnss_covariance_scale_for_time(t, stationary_context);

    if (static_cast<int>(row.values[13]) >= 1) {
      fix.hdop = std::sqrt(std::max(0.25, (row.values[3] + row.values[7]) * 0.5));
      fix.vdop = std::sqrt(std::max(1.0, row.values[11]));
      fix.satellites = 4;
      if (static_cast<int>(row.values[13]) == 3) {
        fix.has_full_covariance = true;
        fix.full_covariance <<
          std::max(0.25, row.values[3]), row.values[4], row.values[5],
          row.values[6], std::max(0.25, row.values[7]), row.values[8],
          row.values[9], row.values[10], std::max(1.0, row.values[11]);
      }
    } else {
      fix.hdop = 1.5;
      fix.vdop = 2.0;
      fix.satellites = 4;
    }

    if (filter_init_time_ > 0.0) {
      const double since_init = std::max(0.0, t - filter_init_time_);
      if (since_init < 5.0) return;
      if (since_init < 20.0) {
        const double alpha = std::clamp((since_init - 5.0) / 15.0, 0.0, 1.0);
        covariance_scale *= 25.0 + (1.0 - 25.0) * alpha;
      }
    }

    if (fix.has_full_covariance) {
      fix.full_covariance *= covariance_scale;
    } else {
      const double sigma_scale = std::sqrt(covariance_scale);
      fix.hdop *= sigma_scale;
      fix.vdop *= sigma_scale;
    }

    fc_.update_gnss(t, fix);
    record_sample(t, "gnss");
  }

  void collect_init_window(const CsvRow& row) {
    const double t = row.stamp;
    if (!init_window_collecting_) {
      init_window_collecting_ = true;
      init_window_start_ = t;
    }

    const int gyro_idx = use_transformed_init_window_ ? 0 : 13;
    const int accel_idx = use_transformed_init_window_ ? 3 : 16;
    init_win_wx_ += row.values[gyro_idx + 0];
    init_win_wy_ += row.values[gyro_idx + 1];
    init_win_wz_ += row.values[gyro_idx + 2];
    init_win_ax_ += row.values[accel_idx + 0];
    init_win_ay_ += row.values[accel_idx + 1];
    init_win_az_ += row.values[accel_idx + 2];
    ++init_win_n_;

    init_win_qw_ += row.values[6];
    init_win_qx_ += row.values[7];
    init_win_qy_ += row.values[8];
    init_win_qz_ += row.values[9];
    ++init_win_orient_n_;

    if ((t - init_window_start_) < 2.0) return;

    State initial;
    initial.P = StateMatrix::Identity() * 0.1;
    initial.P(0, 0) = 1000.0;
    initial.P(1, 1) = 1000.0;
    initial.P(2, 2) = 1000.0;

    const double n = static_cast<double>(std::max(1, init_win_n_));
    initial.x[fusioncore::B_GX] = init_win_wx_ / n;
    initial.x[fusioncore::B_GY] = init_win_wy_ / n;
    initial.x[fusioncore::B_GZ] = init_win_wz_ / n;

    double qw = init_win_qw_ / init_win_orient_n_;
    double qx = init_win_qx_ / init_win_orient_n_;
    double qy = init_win_qy_ / init_win_orient_n_;
    double qz = init_win_qz_ / init_win_orient_n_;
    const double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw /= qnorm;
    qx /= qnorm;
    qy /= qnorm;
    qz /= qnorm;
    initial.x[fusioncore::QW] = qw;
    initial.x[fusioncore::QX] = qx;
    initial.x[fusioncore::QY] = qy;
    initial.x[fusioncore::QZ] = qz;

    constexpr double g = 9.80665;
    const double gx = 2.0 * (qx * qz - qy * qw) * g;
    const double gy = 2.0 * (qy * qz + qx * qw) * g;
    const double gz = (1.0 - 2.0 * (qx * qx + qy * qy)) * g;
    initial.x[fusioncore::B_AX] = init_win_ax_ / n - gx;
    initial.x[fusioncore::B_AY] = init_win_ay_ / n - gy;
    initial.x[fusioncore::B_AZ] = init_win_az_ / n - gz;

    fc_.init(initial, t);
    filter_init_time_ = t;
    pending_init_ = false;
  }

  void remember_primary_imu_motion_sample(
    double timestamp_seconds,
    double wx,
    double wy,
    double wz,
    double ax,
    double ay,
    double az)
  {
    last_primary_imu_time_ = timestamp_seconds;
    last_primary_imu_angular_speed_ = std::sqrt(wx * wx + wy * wy + wz * wz);

    const auto& s = fc_.get_state();
    double R[3][3];
    fusioncore::quat_to_rotation_matrix(
      s.x[fusioncore::QW], s.x[fusioncore::QX], s.x[fusioncore::QY], s.x[fusioncore::QZ], R);
    const double g_world[3] = {0.0, 0.0, 9.80665};
    const double gx = R[0][0] * g_world[0] + R[1][0] * g_world[1] + R[2][0] * g_world[2];
    const double gy = R[0][1] * g_world[0] + R[1][1] * g_world[1] + R[2][1] * g_world[2];
    const double gz = R[0][2] * g_world[0] + R[1][2] * g_world[1] + R[2][2] * g_world[2];

    const double dax = ax - gx;
    const double day = ay - gy;
    const double daz = az - gz;
    last_primary_imu_dynamic_accel_norm_ = std::sqrt(dax * dax + day * day + daz * daz);
  }

  bool is_gnss_stationary_context(double timestamp_seconds) const {
    if (last_primary_encoder_time_ <= 0.0) return false;
    if (std::abs(timestamp_seconds - last_primary_encoder_time_) > 0.5) return false;
    if (last_primary_encoder_speed_ >= config_.zupt_velocity_threshold ||
        std::abs(last_primary_encoder_wz_) >= config_.zupt_angular_threshold) {
      return false;
    }
    if (last_primary_imu_time_ > 0.0 && std::abs(timestamp_seconds - last_primary_imu_time_) <= 0.5) {
      return last_primary_imu_angular_speed_ < 0.05 &&
             last_primary_imu_dynamic_accel_norm_ < 0.2;
    }
    return true;
  }

  double stationary_gnss_covariance_scale_for_time(double timestamp_seconds, bool stationary_context) {
    const double target_scale = stationary_context ? 100.0 : 1.0;
    if (!stationary_transition_initialized_) {
      stationary_transition_initialized_ = true;
      last_stationary_context_ = stationary_context;
      stationary_transition_start_time_ = timestamp_seconds;
      stationary_transition_start_scale_ = target_scale;
      return target_scale;
    }
    if (stationary_context != last_stationary_context_) {
      const double current = stationary_gnss_covariance_scale_for_time(
        timestamp_seconds, last_stationary_context_);
      last_stationary_context_ = stationary_context;
      stationary_transition_start_time_ = timestamp_seconds;
      stationary_transition_start_scale_ = current;
    }
    const double elapsed = std::max(0.0, timestamp_seconds - stationary_transition_start_time_);
    const double alpha = std::clamp(elapsed / 3.0, 0.0, 1.0);
    const double scale =
      stationary_transition_start_scale_ +
      (target_scale - stationary_transition_start_scale_) * alpha;
    if (alpha >= 1.0) {
      stationary_transition_start_scale_ = target_scale;
      stationary_transition_start_time_ = timestamp_seconds;
    }
    return scale;
  }

  void record_sample(double stamp, const std::string& source) {
    if (!fc_.is_initialized()) return;
    const auto& state = fc_.get_state();
    PoseSample sample;
    sample.stamp = stamp;
    sample.source = source;
    sample.x = state.x[fusioncore::X];
    sample.y = state.x[fusioncore::Y];
    sample.yaw = state.yaw();
    sample.vx = state.x[fusioncore::VX];
    sample.pxx = state.P(fusioncore::X, fusioncore::X);
    sample.pyy = state.P(fusioncore::Y, fusioncore::Y);
    sample.pyaw = state.P(fusioncore::QZ, fusioncore::QZ);
    samples_.push_back(sample);
  }
};

void print_summary(const std::vector<PoseSample>& samples) {
  double worst_xy = 0.0;
  double worst_yaw = 0.0;
  const PoseSample* worst_xy_prev = nullptr;
  const PoseSample* worst_xy_cur = nullptr;
  const PoseSample* worst_yaw_prev = nullptr;
  const PoseSample* worst_yaw_cur = nullptr;

  for (std::size_t i = 1; i < samples.size(); ++i) {
    const PoseSample& a = samples[i - 1];
    const PoseSample& b = samples[i];
    const double dxy = std::hypot(b.x - a.x, b.y - a.y);
    double dyaw = b.yaw - a.yaw;
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
    if (dxy > worst_xy) {
      worst_xy = dxy;
      worst_xy_prev = &a;
      worst_xy_cur = &b;
    }
    if (std::abs(dyaw) > std::abs(worst_yaw)) {
      worst_yaw = dyaw;
      worst_yaw_prev = &a;
      worst_yaw_cur = &b;
    }
  }

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "samples=" << samples.size() << "\n";
  if (worst_xy_cur != nullptr) {
    std::cout << "worst_xy_jump=" << worst_xy
              << " stamp=" << worst_xy_cur->stamp
              << " source=" << worst_xy_cur->source
              << " pre=(" << worst_xy_prev->x << "," << worst_xy_prev->y << "," << worst_xy_prev->yaw << "," << worst_xy_prev->vx << ")"
              << " post=(" << worst_xy_cur->x << "," << worst_xy_cur->y << "," << worst_xy_cur->yaw << "," << worst_xy_cur->vx << ")"
              << " cov_post=(" << worst_xy_cur->pxx << "," << worst_xy_cur->pyy << "," << worst_xy_cur->pyaw << ")\n";
  }
  if (worst_yaw_cur != nullptr) {
    std::cout << "worst_yaw_jump=" << worst_yaw
              << " stamp=" << worst_yaw_cur->stamp
              << " source=" << worst_yaw_cur->source
              << " pre=(" << worst_yaw_prev->x << "," << worst_yaw_prev->y << "," << worst_yaw_prev->yaw << "," << worst_yaw_prev->vx << ")"
              << " post=(" << worst_yaw_cur->x << "," << worst_yaw_cur->y << "," << worst_yaw_cur->yaw << "," << worst_yaw_cur->vx << ")"
              << " cov_post=(" << worst_yaw_cur->pxx << "," << worst_yaw_cur->pyy << "," << worst_yaw_cur->pyaw << ")\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) {
    std::cerr << "Usage: fusioncore_csv_replay <events.csv> [current|patched]\n";
    return 2;
  }

  const bool use_transformed_init_window = (argc == 3 && std::string(argv[2]) == "patched");
  if (argc == 3 && std::string(argv[2]) != "current" && std::string(argv[2]) != "patched") {
    std::cerr << "Mode must be 'current' or 'patched'\n";
    return 2;
  }

  FusionCoreConfig config;
  config.max_measurement_delay = 2.0;
  config.snapshot_buffer_size = 300;
  config.imu_buffer_size = 300;
  config.imu_has_magnetometer = true;
  config.imu.gyro_noise_x = 0.005;
  config.imu.gyro_noise_y = 0.005;
  config.imu.gyro_noise_z = 0.005;
  config.imu.accel_noise_x = 0.25;
  config.imu.accel_noise_y = 0.25;
  config.imu.accel_noise_z = 0.25;
  config.gnss.base_noise_xy = 1.5;
  config.gnss.base_noise_z = 3.0;
  config.gnss.max_hdop = 4.0;
  config.gnss.max_vdop = 6.0;
  config.gnss.min_satellites = 4;
  config.gnss.min_fix_type = GnssFixType::GPS_FIX;
  config.ukf.q_position = 0.01;
  config.ukf.q_orientation = 1.0e-9;
  config.ukf.q_velocity = 0.01;
  config.ukf.q_angular_vel = 0.01;
  config.ukf.q_acceleration = 0.2;
  config.ukf.q_gyro_bias = 1.0e-5;
  config.ukf.q_accel_bias = 1.0e-4;
  config.zupt_velocity_threshold = 0.05;
  config.zupt_angular_threshold = 0.05;
  config.zupt_noise_sigma = 0.01;
  config.motion_model = fusioncore::create_motion_model("DifferentialDrive");

  ReplayHarness harness(config, use_transformed_init_window);

  std::ifstream input(argv[1]);
  if (!input) {
    std::cerr << "Cannot open " << argv[1] << "\n";
    return 1;
  }

  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    harness.process(parse_row(line));
  }

  print_summary(harness.samples());
  return 0;
}
