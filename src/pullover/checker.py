import math
import numpy as np
import carla
from typing import cast
from .utils import RansacPlaneFit

# Assume RansacPlaneFit, fit_plane_ransac, point_plane_distance
# are available in your environment


class SafePulloverChecker:
    """
    Decide if pulling over is safe using a single radar sensor and RANSAC plane fitting.

    Workflow:
      - Collect radar points from the sensor.
      - Use RANSAC to fit a plane (guardrail or road boundary).
      - Decide safety based on inlier/outlier statistics:
          * Require a minimum number of inliers.
          * Require an inlier ratio above a threshold.
    """

    def __init__(
        self,
        radar_sensor: carla.Sensor,
        inlier_dist_thresh: float = 0.15,
        min_inlier_ratio: float = 0.8,
        min_inliers: int = 20,
        ransac_max_trials: int = 400,
        debug: bool = False,
    ):
        self.inlier_dist_thresh = float(inlier_dist_thresh)
        self.min_inlier_ratio = float(min_inlier_ratio)
        self.min_inliers = int(min_inliers)
        self.ransac_max_trials = int(ransac_max_trials)
        self.debug = bool(debug)

        # buffer for latest radar points
        self._latest_points = np.zeros((0, 3), dtype=np.float32)

        radar_sensor.listen(lambda data: self.radar_callback(data))


    def _debug(self, msg):
        if self.debug:
            print(msg)

    def radar_callback(self, data):
        arr = self._to_cartesian_coords(cast(carla.RadarMeasurement, data))
        self._latest_points = arr

    def _to_cartesian_coords(self, radar_measurement: carla.RadarMeasurement) -> np.ndarray:
        """Convert CARLA radar detections to Nx3 numpy array and store them."""
        pts = []
        for detection in radar_measurement:
            # conversion from polar to cartesian coordinates
            azi = math.degrees(detection.azimuth)
            alt = math.degrees(detection.altitude)
            fw_vec = carla.Vector3D(x=detection.depth)
            current_rot = radar_measurement.transform.rotation
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)
            
            x = radar_measurement.transform.location.x + fw_vec.x
            y = radar_measurement.transform.location.y + fw_vec.y
            z = radar_measurement.transform.location.z + fw_vec.z
            pts.append((x, y, z))
        return np.asarray(pts, dtype=np.float32)

    # ------------------------------
    # Main decision
    # ------------------------------
    def is_pullover_safe(self) -> bool:
        """Return True if pullover is considered safe, False otherwise."""
        pts = self._latest_points.copy()
        self._debug(pts.shape)

        if pts.shape[0] < self.min_inliers:
            self._debug("Not enough radar points.")
            return False

        # fit plane using RANSAC
        ransac = RansacPlaneFit(pts, self.inlier_dist_thresh, self.ransac_max_trials)

        if not ransac.has_found_plane:
            self._debug("Plane fitting failed.")
            return False

        inlier_ratio = ransac.num_inliers / float(ransac.num_points)
        self._debug(f"Inliers: {ransac.num_inliers}/{ransac.num_points} "
                  f"({inlier_ratio:.2f}), Distance={ransac.distance:.2f}")

        # check thresholds
        if ransac.num_inliers < self.min_inliers:
            self._debug("Not enough inliers.")
            return False

        if inlier_ratio < self.min_inlier_ratio:
            self._debug("Inlier ratio too low.")
            return False

        self._debug("Safe to pull over.")
        return True
