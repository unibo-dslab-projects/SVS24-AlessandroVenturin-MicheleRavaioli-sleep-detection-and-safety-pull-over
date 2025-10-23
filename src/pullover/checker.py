import time
import numpy as np
import carla
from typing import cast
from .utils import to_cartesian_coords

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
        scanned_area_x_offset: float, # in meters
        min_inliers: int = 1,
        safety_delay: float = 0.8, # in seconds: time that should pass before stating its safe
        debug: bool = False,
    ):
        self.scanned_area_x_offset = float(scanned_area_x_offset)
        self.min_inliers = int(min_inliers)
        self.safety_delay = float(safety_delay)
        self.debug = bool(debug)

        # buffer for latest radar points
        self._latest_points = np.zeros((0, 3), dtype=np.float32)
        # last pullover check
        self._safety_time = time.time() + safety_delay

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

    def _is_pullover_safe_no_delay(self, depth: float, scan_width: float, rotation: float = 0) -> bool:

        pts = self._latest_points.copy().reshape((-1, 3))
        self._debug(pts.shape)

        # compute relative positions
        vehicle_tr = self.vehicle.get_transform()
        rotated_tr = carla.Transform(
            vehicle_tr.location,
            carla.Rotation(pitch=vehicle_tr.rotation.pitch, roll=vehicle_tr.rotation.roll, yaw=vehicle_tr.rotation.yaw + rotation)
        )

        thr = depth / self.scanned_area_x_offset # used to select the correct points
        is_relevant = np.zeros(pts.shape[0], dtype=np.bool)
        for i in range(pts.shape[0]):
            vec = carla.Vector3D(float(pts[i, 0]), float(pts[i, 1]), float(pts[i, 2]))
            rotated_tr.inverse_transform(vec)
            pts[i, 0] = vec.x 
            pts[i, 1] = vec.y
            pts[i, 2] = vec.z
            is_relevant[i] = (vec.x / vec.y) < thr

        # 1) get points inside the scanned area (from 0 to width + offset)
        # 2) ignore the points that do not obscure the scan (like cars in front of you)
        # x is parallel to car direction
        in_scanned_area = (pts[:, 1] < self.scanned_area_x_offset + scan_width) & \
                          (pts[:, 0] < depth)
        
        pts_inlier = pts[in_scanned_area & is_relevant]
        pts_outlier = pts[~(in_scanned_area & is_relevant)]

        if self.debug:
            for i in range(pts_inlier.shape[0]):
                self.radar_sensor.get_world().debug.draw_point(
                    rotated_tr.transform(carla.Vector3D(float(pts_inlier[i, 0]), float(pts_inlier[i, 1]), float(pts_inlier[i, 2]))),
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(255, 0, 0))
            for i in range(pts_outlier.shape[0]):
                self.radar_sensor.get_world().debug.draw_point(
                    rotated_tr.transform(carla.Vector3D(float(pts_outlier[i, 0]), float(pts_outlier[i, 1]), float(pts_outlier[i, 2]))),
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(255, 255, 255))
                
        if pts_inlier.shape[0] < self.min_inliers:
            self._debug("No obstacles.")
            return True

        self._debug("Obstacles detected.")
        return False

    def is_pullover_safe(self, depth: float, scan_width: float, rotation: float = 0) -> bool:
        """
        Return True if pullover is considered safe, False otherwise.
        
        Parameters
        ----------
        depth: float
            The distance to cover during the scan.
        scan_width: float
            The lateral width of the scan area.
        rotation: float
            The alignment in degrees of the scan. 0 means it goes stright ahead, any other value rotates the area around the
            top-right corner of the vehicle.
            By default `rotation` is 0.
        """
        is_safe = self._is_pullover_safe_no_delay(depth, scan_width, rotation)

        if not is_safe:
            # when not safe, reset timer for safety
            self._safety_time = time.time() + self.safety_delay

        # is safe only if the safety timer has expired
        return is_safe if self._safety_time <= time.time() else False


