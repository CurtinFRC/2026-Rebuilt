# repulsor_sim/config.py
from dataclasses import dataclass
import os

@dataclass(frozen=True)
class Config:
    nt_server: str
    fieldvision_name: str
    fps: float
    seed: int

    field_length_m: float
    field_width_m: float

    grid_cell_m: float
    grid_region_half_x_m: float
    grid_region_half_y_m: float
    fuel_z_m: float

    max_objects: int
    max_obstacles: int

    pose_base_path: str
    pose_struct_key: str

def load_config() -> Config:
    nt_server = os.getenv("NT_SERVER", "localhost")
    fieldvision_name = os.getenv("FIELDVISION_NAME", "main")
    fps = float(os.getenv("FPS", "20"))
    seed = int(os.getenv("SEED", "1337"))

    field_length_m = float(os.getenv("FIELD_LENGTH_M", "16.54"))
    field_width_m = float(os.getenv("FIELD_WIDTH_M", "8.21"))

    grid_cell_m = float(os.getenv("GRID_CELL_M", "0.25"))
    grid_region_half_x_m = float(os.getenv("GRID_REGION_HALF_X_M", "2.0"))
    grid_region_half_y_m = float(os.getenv("GRID_REGION_HALF_Y_M", "2.0"))
    fuel_z_m = float(os.getenv("FUEL_Z_M", "0.10"))

    max_objects = int(os.getenv("MAX_OBJECTS", "500"))
    max_obstacles = int(os.getenv("MAX_OBSTACLES", "0"))

    return Config(
        nt_server=nt_server,
        fieldvision_name=fieldvision_name,
        fps=fps,
        seed=seed,
        field_length_m=field_length_m,
        field_width_m=field_width_m,
        grid_cell_m=grid_cell_m,
        grid_region_half_x_m=grid_region_half_x_m,
        grid_region_half_y_m=grid_region_half_y_m,
        fuel_z_m=fuel_z_m,
        max_objects=max_objects,
        max_obstacles=max_obstacles,
        pose_base_path= "AdvantageKit/RealOutputs/Odometry",
        pose_struct_key= "Robot",
    )
