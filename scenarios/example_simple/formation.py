# formation.py
import math
from typing import Tuple

from geometry_msgs.msg import PoseStamped


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Quaternion -> yaw (Z-axis rotation)."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_to_xyyaw(msg: PoseStamped) -> Tuple[float, float, float]:
    p = msg.pose.position
    q = msg.pose.orientation
    yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
    return p.x, p.y, yaw


def rotate_2d(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    """Rotate vector (dx, dy) by yaw in 2D."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * dx - s * dy, s * dx + c * dy)


def compute_target_xy(
    leader_xyyaw: Tuple[float, float, float],
    offset_xy: Tuple[float, float],
) -> Tuple[float, float]:
    """
    Leader 기준 offset(dx, dy)을 leader yaw로 회전해 world 좌표 목표점으로 변환.
    offset_xy는 leader local frame(leader heading 기준)으로 해석.
    """
    lx, ly, lyaw = leader_xyyaw
    dx, dy = offset_xy
    rdx, rdy = rotate_2d(dx, dy, lyaw)
    return lx + rdx, ly + rdy


def in_formation(
    follower_xy: Tuple[float, float],
    target_xy: Tuple[float, float],
    tolerance_xy: float,
) -> bool:
    fx, fy = follower_xy
    tx, ty = target_xy
    dist = math.hypot(tx - fx, ty - fy)
    return dist <= tolerance_xy


def compute_cmd_vel_to_target(
    follower_xyyaw: Tuple[float, float, float],
    target_xy: Tuple[float, float],
    tolerance_xy: float,
    *,
    v_max: float = 0.6,
    w_max: float = 1.2,
    k_v: float = 0.8,
    k_w: float = 2.0,
    slow_radius: float = 2.0,
) -> Tuple[float, float, bool]:
    """
    (x,y,yaw) -> (v,w) P-control.
    - 목표점에 tolerance 이내면 done=True 반환.
    - heading error가 클수록 선속을 줄여 벽에 박는 것 완화.
    """
    x, y, yaw = follower_xyyaw
    tx, ty = target_xy

    ex = tx - x
    ey = ty - y
    dist = math.hypot(ex, ey)

    if dist <= tolerance_xy:
        return 0.0, 0.0, True

    desired = math.atan2(ey, ex)
    err_yaw = normalize_angle(desired - yaw)

    # 각속도
    w = clamp(k_w * err_yaw, -w_max, w_max)

    # 선속도: 멀수록 빠르게, 하지만 방향이 틀리면(각오차) 줄이기
    # - cos(err_yaw)로 “목표 방향 성분”만 반영
    # - slow_radius 안에서는 더 부드럽게 감속
    v_scale = clamp(dist / max(1e-6, slow_radius), 0.2, 1.0)
    v = k_v * dist * math.cos(err_yaw) * v_scale
    v = clamp(v, 0.0, v_max)

    return v, w, False