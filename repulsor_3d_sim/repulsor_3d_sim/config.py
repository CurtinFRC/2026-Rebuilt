from __future__ import annotations
from dataclasses import dataclass
import os

def _f(name: str, default: float) -> float:
    v = os.environ.get(name, "")
    if not v:
        return float(default)
    try:
        return float(v)
    except Exception:
        return float(default)

def _s(name: str, default: str) -> str:
    v = os.environ.get(name, "")
    return v if v else default

def _i(name: str, default: int) -> int:
    v = os.environ.get(name, "")
    if not v:
        return int(default)
    try:
        return int(v)
    except Exception:
        return int(default)

def _b(name: str, default: bool) -> bool:
    v = os.environ.get(name, "")
    if not v:
        return bool(default)
    vv = v.strip().lower()
    if vv in ("1", "true", "yes", "y", "on"):
        return True
    if vv in ("0", "false", "no", "n", "off"):
        return False
    return bool(default)

@dataclass(frozen=True)
class ViewerConfig:
    nt_server: str
    nt_client_name: str
    fieldvision_path: str
    repulsorvision_path: str
    pose_base_path: str
    pose_struct_key: str
    truth_socket_host: str
    truth_socket_port: int
    truth_socket_enabled: bool
    field_image_path: str
    show_field_image: bool
    field_image_alpha: float
    field_image_flip_x: bool
    field_image_flip_y: bool
    collect_age_decay: float
    resource_hard_max_age_s: float
    show_age_filtered_fuel: bool
    window_w: int
    window_h: int
    fps: int
    field_length_m: float
    field_width_m: float
    field_z_m: float
    ball_radius_m: float
    obs_box_side_m: float
    robot_box_l_m: float
    robot_box_w_m: float
    robot_box_h_m: float
    pose_box_side_m: float
    pose_box_h_m: float
    camera_distance_m: float
    camera_pitch_deg: float
    camera_yaw_deg: float
    follow_robot: bool
    show_camera_debug: bool
    show_truth_fuel: bool
    show_camera_debug: bool

def load_config() -> ViewerConfig:
    return ViewerConfig(
        nt_server=_s("NT_SERVER", "127.0.0.1"),
        nt_client_name=_s("NT_CLIENT_NAME", "repulsor_3d_sim"),
        fieldvision_path=_s("NT_FIELDVISION_PATH", "FieldVision/main"),
        repulsorvision_path=_s("NT_REPULSORVISION_PATH", "RepulsorVision"),
        pose_base_path=_s("NT_POSE_BASE_PATH", "AdvantageKit/RealOutputs/Odometry"),
        pose_struct_key=_s("NT_POSE_STRUCT_KEY", "Robot"),
        truth_socket_host=_s("TRUTH_SOCKET_HOST", "127.0.0.1"),
        truth_socket_port=_i("TRUTH_SOCKET_PORT", 5809),
        truth_socket_enabled=_b("TRUTH_SOCKET_ENABLED", True),
        field_image_path=_s("FIELD_IMAGE_PATH", "field.png"),
        show_field_image=_b("SHOW_FIELD_IMAGE", True),
        field_image_alpha=_f("FIELD_IMAGE_ALPHA", 0.92),
        # Default flip aligns field texture orientation with blue-origin field coordinates.
        field_image_flip_x=_b("FIELD_IMAGE_FLIP_X", True),
        field_image_flip_y=_b("FIELD_IMAGE_FLIP_Y", False),
        collect_age_decay=_f("COLLECT_AGE_DECAY", 1.25),
        resource_hard_max_age_s=_f("RESOURCE_HARD_MAX_AGE_S", 0.95),
        show_age_filtered_fuel=_b("SHOW_AGE_FILTERED_FUEL", False),
        window_w=_i("WINDOW_W", 1280),
        window_h=_i("WINDOW_H", 720),
        fps=_i("FPS", 60),
        field_length_m=_f("FIELD_LENGTH_M", 16.540988),
        field_width_m=_f("FIELD_WIDTH_M", 8.21055),
        field_z_m=_f("FIELD_Z_M", 0.0),
        ball_radius_m=_f("BALL_RADIUS_M", 0.075),
        obs_box_side_m=_f("OBS_BOX_SIDE_M", 0.6),
        robot_box_l_m=_f("ROBOT_BOX_L_M", 0.85),
        robot_box_w_m=_f("ROBOT_BOX_W_M", 0.85),
        robot_box_h_m=_f("ROBOT_BOX_H_M", 0.35),
        pose_box_side_m=_f("POSE_BOX_SIDE_M", 0.35),
        pose_box_h_m=_f("POSE_BOX_H_M", 0.25),
        camera_distance_m=_f("CAMERA_DISTANCE_M", 12.0),
        camera_pitch_deg=_f("CAMERA_PITCH_DEG", 35.0),
        camera_yaw_deg=_f("CAMERA_YAW_DEG", 135.0),
        follow_robot=_b("FOLLOW_ROBOT", True),
        show_camera_debug=_b("SHOW_CAMERA_DEBUG", True),
        show_truth_fuel=_b("SHOW_TRUTH_FUEL", True),
    )
