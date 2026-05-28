import os
from ruamel.yaml import YAML


class Config:
    """LiDAR-only training / inference configuration loaded from traj_opt.yaml."""

    def __init__(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self._data = YAML().load(open(os.path.join(base_dir, "traj_opt.yaml"), 'r'))
        self._data["train"] = True
        self._data["goal_length"] = 2.0 * self._data['radio_range']
        self._data["sgm_time"] = 2 * self._data["radio_range"] / self._data["vel_max_train"]

        self._data["horizon_num"] = (
            self._data["lidar_horizon_num_per_segment"] * self._data["lidar_lattice_segments"]
        )
        self._data["vertical_num"] = self._data["lidar_vertical_num"]
        self._data["horizon_camera_fov"] = self._data["lidar_horizon_fov"]
        self._data["vertical_camera_fov"] = self._data.get(
            "lidar_lattice_vertical_fov", self._data["lidar_vertical_fov"]
        )
        self._data["horizon_anchor_fov"] = self._data["lidar_horizon_anchor_fov"]
        self._data["vertical_anchor_fov"] = self._data["lidar_vertical_anchor_fov"]
        self._data["image_height"] = self._data["range_image_height"]
        self._data["image_width"] = self._data["range_image_width"]
        self._data["traj_num"] = self._data['horizon_num'] * self._data['vertical_num'] * self._data["radio_num"]

    def __getitem__(self, key):
        return self._data[key]

    def __setitem__(self, key, value):
        self._data[key] = value

    def get(self, key, default=None):
        return self._data.get(key, default)

    def is_lidar_mode(self):
        """Always LiDAR-only in this repository."""
        return True


cfg = Config()
