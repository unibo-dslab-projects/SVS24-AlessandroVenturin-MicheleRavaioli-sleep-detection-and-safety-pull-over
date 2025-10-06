from typing import Optional, Tuple
import numpy as np
import carla
import math

def to_cartesian_coords(radar_measurement: carla.RadarMeasurement) -> list[tuple[float, float, float]]:
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
        return pts

# ==============================
# RANSAC plane fitting utilities
# ==============================

class RansacPlaneFit():
    def __init__(self, points: np.ndarray, thresh: float, max_trials: int = 400, origin = (0, 0, 0)):
        self.points = points
        self.num_points = points.shape[0]
        self.threshold = thresh
        self.max_trials = max_trials

        plane, mask = fit_plane_ransac(points, thresh, max_trials)

        if plane is not None and mask is not None:
            # plane found: compute right values
            self.has_found_plane = True
            self.num_inliers = int(mask.sum())
            self.num_outliers = self.num_points - self.num_inliers
            self.distance = point_plane_distance(plane, origin)
        else:
            # plane not found: no inliers
            self.has_found_plane = False
            self.num_inliers = 0
            self.num_outliers = self.num_points
            self.distance = 0.0
            


def fit_plane_ransac(
    points: np.ndarray,
    thresh: float,
    max_iterations: int,
    min_inliers: int = 3,
    random_seed: Optional[int] = None
) -> Tuple[Optional[Tuple[float, float, float, float]], Optional[np.ndarray]]:
    """
    Fit a plane to 3D points using RANSAC.

    Parameters
    ----------
    points : (N,3) ndarray
        Input 3D points.
    thresh : float
        Distance threshold (meters). A point is an inlier if distance_to_plane <= thresh.
    max_iterations : int
        Maximum number of RANSAC iterations.
    min_inliers : int, optional
        Minimum number of inliers required to accept a model.
    random_seed : Optional[int]
        RNG seed for reproducibility.

    Returns
    -------
    (a,b,c,d), inlier_mask or (None, None)
        Plane parameters a,b,c,d such that a*x + b*y + c*z + d = 0 and
        (a,b,c) is a unit vector. inlier_mask is boolean array shape (N,).
        If no valid plane found, returns (None, None).
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must be an (N,3) array")
    N = pts.shape[0]
    if N < 3:
        return (None, None)

    rng = np.random.default_rng(random_seed)

    best_mask = None
    best_count = 0

    for _ in range(int(max_iterations)):
        # sample 3 distinct points
        try:
            idx = rng.choice(N, size=3, replace=False)
        except Exception:
            # in case N < 3 (shouldn't happen because we checked), skip
            continue
        p1, p2, p3 = pts[idx]
        # candidate plane normal = cross(p2-p1, p3-p1)
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            # degenerate triple (collinear) -> skip
            continue
        normal_unit = normal / norm
        d = -float(np.dot(normal_unit, p1))
        # compute perpendicular distances |n·x + d|
        dists = np.abs(np.dot(pts, normal_unit) + d)  # shape (N,)
        mask = dists <= float(thresh)
        cnt = int(mask.sum())
        if cnt > best_count:
            best_count = cnt
            best_mask = mask
            # early exit if almost all points are inliers
            if best_count > 0.995 * N:
                break

    if best_mask is None or best_count < int(min_inliers):
        # no model found
        return (None, None)

    # refine the model using all inliers by total-least-squares (SVD)
    inlier_pts = pts[best_mask]
    centroid = np.mean(inlier_pts, axis=0)
    # Use SVD on centered inliers: last singular vector is normal direction
    # U, S, Vt = svd(inlier_pts - centroid)
    # normal_refined = Vt[-1]
    try:
        _, _, Vt = np.linalg.svd(inlier_pts - centroid)
    except np.linalg.LinAlgError:
        # numerical issue
        return (None, None)

    normal_refined = Vt[-1, :]
    normr = np.linalg.norm(normal_refined)
    if normr < 1e-12:
        return (None, None)
    normal_refined = normal_refined / normr
    d_refined = -float(np.dot(normal_refined, centroid))

    # recompute final inliers with refined plane
    dists_final = np.abs(np.dot(pts, normal_refined) + d_refined)
    final_mask = dists_final <= float(thresh)

    a, b, c = float(normal_refined[0]), float(normal_refined[1]), float(normal_refined[2])
    d = float(d_refined)
    return ( (a, b, c, d), final_mask )


def point_plane_distance(plane: Tuple[float, float, float, float], point: Tuple[float, float, float]) -> float:
    """
    Compute perpendicular distance from a 3D point to plane ax + by + cz + d = 0.

    Parameters
    ----------
    plane : tuple (a,b,c,d)
        Plane coefficients. (a,b,c) need not be normalized; denominator accounts for that.
    point : (x,y,z)
        3D point coordinates.

    Returns
    -------
    float
        Perpendicular distance (>=0).
    """
    a, b, c, d = plane
    x, y, z = point
    denom = math.sqrt(a*a + b*b + c*c)
    if denom == 0.0:
        return float('inf')
    return abs(a*x + b*y + c*z + d) / denom
