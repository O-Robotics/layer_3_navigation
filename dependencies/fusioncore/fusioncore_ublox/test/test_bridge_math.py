"""
Pure-Python unit tests for the gnss_doppler_bridge conversion logic.
No ROS, no ublox_msgs required. Run with: python3 test_bridge_math.py
or via colcon test.

Tests the three things the bridge does:
  1. NED mm/s -> ENU m/s
  2. s_acc -> variance
  3. Acceptance filter (fix_type + gnssFixOK flag + min speed)
"""
import math
import pytest


# --- Replicated from gnss_doppler_bridge.cpp (the logic under test) ---

def ned_to_enu(vel_n_mm: int, vel_e_mm: int, vel_d_mm: int):
    return vel_e_mm / 1000.0, vel_n_mm / 1000.0, -vel_d_mm / 1000.0


def s_acc_to_variance(s_acc_mm: int) -> float:
    sigma = s_acc_mm / 1000.0
    return sigma * sigma


def fix_accepted(fix_type: int, flags: int) -> bool:
    return fix_type >= 3 and bool(flags & 0x01)


def speed_above_min(vel_n_mm: int, vel_e_mm: int, min_mps: float = 0.05) -> bool:
    east  = vel_e_mm / 1000.0
    north = vel_n_mm / 1000.0
    return math.hypot(east, north) >= min_mps


# --- NED to ENU conversion ---

def test_pure_east():
    east, north, up = ned_to_enu(vel_n_mm=0, vel_e_mm=1000, vel_d_mm=0)
    assert east  == pytest.approx(1.0)
    assert north == pytest.approx(0.0)
    assert up    == pytest.approx(0.0)


def test_pure_north():
    east, north, up = ned_to_enu(vel_n_mm=2000, vel_e_mm=0, vel_d_mm=0)
    assert east  == pytest.approx(0.0)
    assert north == pytest.approx(2.0)
    assert up    == pytest.approx(0.0)


def test_down_inverted_to_up():
    # NED down=500 mm/s becomes ENU up=-0.5 m/s
    east, north, up = ned_to_enu(vel_n_mm=0, vel_e_mm=0, vel_d_mm=500)
    assert up == pytest.approx(-0.5)


def test_upward_motion():
    # NED down=-300 mm/s (climbing) becomes ENU up=+0.3 m/s
    east, north, up = ned_to_enu(vel_n_mm=0, vel_e_mm=0, vel_d_mm=-300)
    assert up == pytest.approx(0.3)


def test_combined_velocity():
    east, north, up = ned_to_enu(vel_n_mm=1000, vel_e_mm=2000, vel_d_mm=3000)
    assert east  == pytest.approx(2.0)
    assert north == pytest.approx(1.0)
    assert up    == pytest.approx(-3.0)


def test_mm_to_m_scaling():
    # 1 mm/s = 0.001 m/s
    east, north, up = ned_to_enu(vel_n_mm=1, vel_e_mm=1, vel_d_mm=1)
    assert east  == pytest.approx(0.001)
    assert north == pytest.approx(0.001)
    assert up    == pytest.approx(-0.001)


# --- Covariance from s_acc ---

def test_variance_typical():
    # s_acc=500 mm/s (0.5 m/s 1-sigma) -> variance=0.25
    assert s_acc_to_variance(500) == pytest.approx(0.25)


def test_variance_tight():
    # s_acc=100 mm/s (0.1 m/s) -> variance=0.01
    assert s_acc_to_variance(100) == pytest.approx(0.01)


def test_variance_zero():
    assert s_acc_to_variance(0) == pytest.approx(0.0)


def test_variance_is_sigma_squared():
    for s_acc in [50, 200, 1000, 3000]:
        sigma = s_acc / 1000.0
        assert s_acc_to_variance(s_acc) == pytest.approx(sigma ** 2)


# --- Fix acceptance filter ---

def test_3d_fix_accepted():
    assert fix_accepted(fix_type=3, flags=0x01)


def test_gnss_dr_combined_accepted():
    assert fix_accepted(fix_type=4, flags=0x01)


def test_time_only_with_ok_flag_accepted():
    assert fix_accepted(fix_type=5, flags=0x03)


def test_2d_fix_rejected():
    assert not fix_accepted(fix_type=2, flags=0x01)


def test_dead_reckoning_rejected():
    assert not fix_accepted(fix_type=1, flags=0x01)


def test_no_fix_rejected():
    assert not fix_accepted(fix_type=0, flags=0x01)


def test_gnssFixOK_not_set_rejected():
    # fix_type=3 but bit 0 of flags is not set
    assert not fix_accepted(fix_type=3, flags=0x00)
    assert not fix_accepted(fix_type=3, flags=0x02)   # diff soln bit set but not gnssFixOK
    assert not fix_accepted(fix_type=3, flags=0xFE)   # all bits except bit 0


def test_gnssFixOK_set_regardless_of_other_flags():
    # bit 0 set alongside other flags: should be accepted
    assert fix_accepted(fix_type=3, flags=0xFF)
    assert fix_accepted(fix_type=3, flags=0x03)


# --- Minimum speed filter ---

def test_moving_east_passes():
    assert speed_above_min(vel_n_mm=0, vel_e_mm=1000)     # 1.0 m/s


def test_moving_north_passes():
    assert speed_above_min(vel_n_mm=1000, vel_e_mm=0)     # 1.0 m/s


def test_diagonal_motion_passes():
    # sqrt(0.04^2 + 0.03^2) = 0.05 m/s: exactly at threshold, should pass
    assert speed_above_min(vel_n_mm=30, vel_e_mm=40)


def test_near_stationary_blocked():
    # 0.03 m/s: below 0.05 default threshold
    assert not speed_above_min(vel_n_mm=0, vel_e_mm=30)


def test_very_slow_blocked():
    assert not speed_above_min(vel_n_mm=10, vel_e_mm=10)  # ~0.014 m/s


def test_exactly_zero_blocked():
    assert not speed_above_min(vel_n_mm=0, vel_e_mm=0)


if __name__ == "__main__":
    # Run directly without pytest: useful on embedded systems without pytest
    import sys
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
        except Exception as e:
            print(f"  FAIL  {t.__name__}: {e}")
            failed += 1
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
