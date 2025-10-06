import math
import numpy as np
import carla
from typing import cast
from .utils import RansacPlaneFit, to_cartesian_coords

# Assume RansacPlaneFit, fit_plane_ransac, point_plane_distance
# are available in your environment


class SafePulloverChecker:
    """
    Decide if pulling over is safe using a single radar sensor.

    Workflow:
      - Collect radar points from the sensor.
      - Check only the points scanning the right lane area.
      - Decide safety based on inlier/outlier statistics:
          * Require a minimum number of inliers.
          * Require an inlier ratio above a threshold.
    """

    def __init__(
        self,
        radar_sensor: carla.Sensor,
        vehicle: carla.Vehicle,
        scanned_area_width: float,
        scanned_area_x_offset: float,
        min_inliers: int = 2,
        scanned_area_depth: float = 50, # in meters
        debug: bool = False,
    ):
        self.scanned_area_width = float(scanned_area_width)
        self.scanned_area_depth = float(scanned_area_depth)
        self.scanned_area_x_offset = float(scanned_area_x_offset)
        self.min_inliers = int(min_inliers)
        self.debug = bool(debug)

        # buffer for latest radar points
        self._latest_points = np.zeros((0, 3), dtype=np.float32)

        self.vehicle = vehicle

        self.radar_sensor = radar_sensor
        self.radar_sensor.listen(lambda data: self.radar_callback(data))
        

    def _debug(self, msg):
        if self.debug:
            print(msg)

    def radar_callback(self, data):
        arr = to_cartesian_coords(cast(carla.RadarMeasurement, data))
        self._latest_points = arr

    # ------------------------------
    # Main decision
    # ------------------------------
    def is_pullover_safe(self, depth: float = -1) -> bool:
        """
        Return True if pullover is considered safe, False otherwise.
        
        Parameters
        ----------
        depth: float
            The distance to cover during the scan. If it's a negative number, then uses the default value of the checker.
            By default `depth` is -1.
        """
        if depth < 0:
            depth = self.scanned_area_depth

        pts = self._latest_points.copy()
        self._debug(pts.shape)

        # compute relative positions
        vehicle_tr = self.vehicle.get_transform()
        for i in range(pts.shape[0]):
            vec = carla.Vector3D(float(pts[i, 0]), float(pts[i, 1]), float(pts[i, 2]))
            vehicle_tr.inverse_transform(vec)
            pts[i, 0] = vec.x 
            pts[i, 1] = vec.y
            pts[i, 2] = vec.z

        # get points inside the scanned area
        # x is parallel to car direction
        scan_pts = (pts[:, 1] >= self.scanned_area_x_offset) & (pts[:, 1] < self.scanned_area_x_offset + self.scanned_area_width) & \
                   (pts[:, 0] < depth)
        
        pts = pts[scan_pts]

        if self.debug:
            for i in range(pts.shape[0]):
                self.radar_sensor.get_world().debug.draw_point(
                    vehicle_tr.transform(carla.Vector3D(float(pts[i, 0]), float(pts[i, 1]), float(pts[i, 2]))),
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(255, 0, 0))

        if pts.shape[0] < self.min_inliers:
            self._debug("No obstacles.")
            return True

        self._debug("Obstacles detected.")
        return False
