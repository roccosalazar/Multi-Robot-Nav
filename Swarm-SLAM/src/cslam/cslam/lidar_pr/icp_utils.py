import cslam.utils.point_cloud2 as pc2_utils
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

import multiprocessing as mp
import numpy as np
import os
import teaserpp_python
import open3d
import platform
import queue
import signal
import sys
from scipy.spatial import cKDTree
import rclpy
import rclpy.logging

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

_warned_icp_refinement_disabled = False

# Partially adapted from https://github.com/MIT-SPARK/TEASER-plusplus/tree/master/examples/teaser_python_fpfh_icp


def pcd2xyz(pcd):
    return np.asarray(pcd.points).T


def identity_transform_msg():
    transform = Transform()
    transform.rotation.w = 1.0
    return transform


def open3d_version_tuple():
    version = getattr(open3d, "__version__", "0")
    parts = []
    for part in version.split("."):
        digits = "".join(ch for ch in part if ch.isdigit())
        if digits == "":
            break
        parts.append(int(digits))
    return tuple(parts)


def should_use_icp_refinement():
    machine = platform.machine().lower()
    version = open3d_version_tuple()
    if machine in ("aarch64", "arm64") and version < (0, 19, 0):
        return False
    return True


def log_icp_refinement_disabled_once():
    global _warned_icp_refinement_disabled
    if _warned_icp_refinement_disabled:
        return
    _warned_icp_refinement_disabled = True
    rclpy.logging.get_logger('cslam').warn(
        'Open3D ICP refinement disabled for loop closure registration '
        f'on {platform.machine()} with Open3D {getattr(open3d, "__version__", "unknown")}; '
        'using TEASER++ plus NumPy fitness/RMSE verification.')


def transform_is_finite(translation, rotation):
    return (
        np.isfinite(np.asarray(translation)).all()
        and np.isfinite(np.asarray(rotation)).all()
    )


def count_correspondence_inliers(src_corr, dst_corr, translation, rotation,
                                 noise_bound):
    transformed = np.asarray(rotation) @ src_corr + np.asarray(translation).reshape(3, 1)
    residuals = np.linalg.norm(transformed - dst_corr, axis=0)
    return int(np.count_nonzero(residuals <= noise_bound))


def evaluate_transform_numpy(src, dst, translation, rotation,
                             max_correspondence_distance):
    src_points = np.asarray(src.points)
    dst_points = np.asarray(dst.points)
    if len(src_points) == 0 or len(dst_points) == 0:
        return 0.0, float("inf")

    transformed = (np.asarray(rotation) @ src_points.T).T + np.asarray(translation)
    nearest_neighbor_tree = cKDTree(dst_points)
    distances, _ = nearest_neighbor_tree.query(transformed, k=1, workers=-1)
    inliers = distances <= max_correspondence_distance
    inlier_count = int(np.count_nonzero(inliers))
    if inlier_count == 0:
        return 0.0, float("inf")

    fitness = inlier_count / float(len(src_points))
    rmse = float(np.sqrt(np.mean(distances[inliers] ** 2)))
    return fitness, rmse


def points_to_open3d(points):
    cloud = open3d.geometry.PointCloud()
    cloud.points = open3d.utility.Vector3dVector(
        np.ascontiguousarray(points, dtype=np.float64))
    return cloud


def format_process_exitcode(exitcode):
    if exitcode is None:
        return "unknown"
    if exitcode < 0:
        try:
            signame = signal.Signals(-exitcode).name
        except ValueError:
            signame = f"signal {-exitcode}"
        return f"{exitcode} ({signame})"
    return str(exitcode)


def registration_multiprocessing_context():
    main_module = sys.modules.get("__main__")
    main_file = getattr(main_module, "__file__", None)
    if main_file and os.path.exists(main_file):
        return mp.get_context("spawn")
    return mp.get_context("fork")


def extract_fpfh(pcd, voxel_size):
    radius_normal = voxel_size * 2
    pcd.estimate_normals(
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal,
                                                max_nn=30))

    radius_feature = voxel_size * 5
    fpfh = open3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature,
                                                max_nn=100))
    return np.array(fpfh.data).T


def find_knn_cpu(feat0, feat1, knn=1, return_distance=False):
    feat1tree = cKDTree(feat1)
    dists, nn_inds = feat1tree.query(feat0, k=knn, workers=-1)
    if return_distance:
        return nn_inds, dists
    else:
        return nn_inds


def find_correspondences(feats0, feats1, mutual_filter=True):
    nns01 = find_knn_cpu(feats0, feats1, knn=1, return_distance=False)
    corres01_idx0 = np.arange(len(nns01))
    corres01_idx1 = nns01

    if not mutual_filter:
        return corres01_idx0, corres01_idx1

    nns10 = find_knn_cpu(feats1, feats0, knn=1, return_distance=False)
    corres10_idx1 = np.arange(len(nns10))
    corres10_idx0 = nns10

    mutual_filter = (corres10_idx0[corres01_idx1] == corres01_idx0)
    corres_idx0 = corres01_idx0[mutual_filter]
    corres_idx1 = corres01_idx1[mutual_filter]

    return corres_idx0, corres_idx1


def get_teaser_solver(noise_bound):
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1.0
    solver_params.noise_bound = noise_bound
    solver_params.estimate_scaling = False
    solver_params.inlier_selection_mode = \
        teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT
    solver_params.rotation_tim_graph = \
        teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
    solver_params.rotation_estimation_algorithm = \
        teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    solver_params.rotation_gnc_factor = 1.4
    solver_params.rotation_max_iterations = 10000
    solver_params.rotation_cost_threshold = 1e-16
    solver = teaserpp_python.RobustRegistrationSolver(solver_params)
    return solver


def Rt2T(R, t):
    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def downsample(points, voxel_size):

    mask = np.isfinite(points).all(axis=1)
    filtered_points = np.ascontiguousarray(points[mask], dtype=np.float64)

    open3d_cloud = open3d.geometry.PointCloud()
    open3d_cloud.points = open3d.utility.Vector3dVector(filtered_points)
    return open3d_cloud.voxel_down_sample(voxel_size=voxel_size)


def solve_teaser(src, dst, voxel_size, min_inliers, min_fitness=0.0,
                 max_rmse=float("inf"), use_icp_refinement=None):
    if len(src.points) == 0 or len(dst.points) == 0:
        rclpy.logging.get_logger('cslam').info(
            'Failed to compute loop closure. Empty point cloud received.')
        return False, None, None

    if use_icp_refinement is None:
        use_icp_refinement = should_use_icp_refinement()

    src_feats = extract_fpfh(src, voxel_size)
    dst_feats = extract_fpfh(dst, voxel_size)

    corrs_src, corrs_dst = find_correspondences(src_feats,
                                                dst_feats,
                                                mutual_filter=True)

    if len(corrs_src) == 0 or len(corrs_dst) == 0:
        rclpy.logging.get_logger('cslam').info(
            'Failed to compute loop closure. No feature correspondences found.')
        return False, None, None

    src_xyz = pcd2xyz(src)  # np array of size 3 by N
    dst_xyz = pcd2xyz(dst)  # np array of size 3 by M
    src_corr = src_xyz[:, corrs_src]  # np array of size 3 by num_corrs
    dst_corr = dst_xyz[:, corrs_dst]  # np array of size 3 by num_corrs

    solver = get_teaser_solver(voxel_size)
    # TEASER++ estimates the rigid transform that maps source correspondences
    # onto target correspondences, i.e. src -> dst.
    solver.solve(src_corr, dst_corr)

    solution = solver.getSolution()
    if not transform_is_finite(solution.translation, solution.rotation):
        rclpy.logging.get_logger('cslam').info(
            'Failed to compute loop closure. Non-finite TEASER++ transform.')
        return False, None, None

    inlier_count = count_correspondence_inliers(
        src_corr, dst_corr, solution.translation, solution.rotation, voxel_size)
    valid = inlier_count > min_inliers

    if valid:
        if use_icp_refinement:
            # ICP refinement
            T_teaser = Rt2T(solution.rotation, solution.translation)
            icp_sol = open3d.pipelines.registration.registration_icp(
                src, dst, voxel_size, T_teaser,
                open3d.pipelines.registration.TransformationEstimationPointToPoint(
                ),
                open3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=100))
            fitness = icp_sol.fitness
            rmse = icp_sol.inlier_rmse
            T_icp = icp_sol.transformation
            solution.translation = T_icp[:3, 3]
            solution.rotation = T_icp[:3, :3]
        else:
            log_icp_refinement_disabled_once()
            fitness, rmse = evaluate_transform_numpy(
                src, dst, solution.translation, solution.rotation, voxel_size)

        if fitness < min_fitness:
            rclpy.logging.get_logger('cslam').info(
                'Failed to compute loop closure. Registration fitness too low '
                f'( {fitness:.4f} < {min_fitness:.4f} )')
            return False, None, None
        if rmse > max_rmse:
            rclpy.logging.get_logger('cslam').info(
                'Failed to compute loop closure. Registration RMSE too high '
                f'( {rmse:.4f} > {max_rmse:.4f} )')
            return False, None, None
    else:
        rclpy.logging.get_logger('cslam').info(
            'Failed to compute loop closure. Number of inliers ( {} / {} )'.
            format(inlier_count, min_inliers))
        return False, None, None
    return True, solution.translation, solution.rotation


def to_transform_msg(translation, rotation):
    T = Transform()
    T.translation.x = translation[0]
    T.translation.y = translation[1]
    T.translation.z = translation[2]
    rotation_matrix = R.from_matrix(np.array(rotation))
    q = rotation_matrix.as_quat()
    T.rotation.x = q[0]
    T.rotation.y = q[1]
    T.rotation.z = q[2]
    T.rotation.w = q[3]
    return T


def open3d_to_ros(open3d_cloud):
    header = Header()
    fields = FIELDS_XYZ
    points = np.asarray(open3d_cloud.points)
    return pc2_utils.create_cloud(header, fields, points)


def ros_to_open3d(msg):
    points = ros_pointcloud_to_points(msg)
    open3d_cloud = open3d.geometry.PointCloud()
    open3d_cloud.points = open3d.utility.Vector3dVector(points)
    return open3d_cloud


def ros_pointcloud_to_points(pc_msg):
    return pc2_utils.read_points_numpy_filtered(pc_msg)[:, :3]


def downsample_ros_pointcloud(pc_msg, voxel_size):
    points = ros_pointcloud_to_points(pc_msg)
    return downsample(points, voxel_size)

def compute_transform(src, dst, voxel_size, min_inliers, min_fitness=0.0,
                      max_rmse=float("inf"), use_icp_refinement=None):
    """Computes a 3D transform between 2 point clouds using TEASER++.

    Returns the transform that maps ``src`` onto ``dst``.
    TEASER++ and the Open3D ICP refinement both use the same source-to-target
    convention, so the result is suitable for a BetweenFactor(src, dst, ...).

    Args:
        src (Open3D point cloud): pointcloud from
        dst (Open3D point cloud): pointcloud to
        voxel_size (int): Voxel size
        min_inliers (int): Minimum number of inlier points correspondance to consider the registration a success

    Returns:
        (Transform, bool): transform and success flag
    """
    valid, translation, rotation = solve_teaser(
        src, dst, voxel_size, min_inliers, min_fitness, max_rmse,
        use_icp_refinement)
    if not valid or translation is None or rotation is None:
        return identity_transform_msg(), False

    transform = to_transform_msg(translation, rotation)
    return transform, True


def compute_transform_worker(result_queue, src_points, dst_points, voxel_size,
                             min_inliers, min_fitness, max_rmse,
                             use_icp_refinement):
    try:
        src = points_to_open3d(src_points)
        dst = points_to_open3d(dst_points)
        valid, translation, rotation = solve_teaser(
            src, dst, voxel_size, min_inliers, min_fitness, max_rmse,
            use_icp_refinement)
        if not valid or translation is None or rotation is None:
            result_queue.put((False, None, None, None))
            return
        result_queue.put((
            True,
            np.asarray(translation, dtype=float).tolist(),
            np.asarray(rotation, dtype=float).tolist(),
            None,
        ))
    except BaseException as exc:
        result_queue.put((False, None, None, repr(exc)))


def compute_transform_crash_safe(src, dst, voxel_size, min_inliers,
                                 min_fitness=0.0, max_rmse=float("inf"),
                                 timeout_s=30.0,
                                 use_icp_refinement=None):
    """Run loop-closure registration in a child process.

    TEASER++ and Open3D execute native code. If that code segfaults, Python
    cannot catch it in-process; isolating the registration keeps the ROS map
    manager alive and reports the loop closure as failed instead.
    """
    src_points = np.ascontiguousarray(np.asarray(src.points), dtype=np.float64)
    dst_points = np.ascontiguousarray(np.asarray(dst.points), dtype=np.float64)

    context = registration_multiprocessing_context()
    result_queue = context.Queue(maxsize=1)
    process = context.Process(
        target=compute_transform_worker,
        args=(
            result_queue,
            src_points,
            dst_points,
            voxel_size,
            min_inliers,
            min_fitness,
            max_rmse,
            use_icp_refinement,
        ),
    )
    process.start()
    process.join(timeout_s)

    logger = rclpy.logging.get_logger('cslam')
    if process.is_alive():
        process.terminate()
        process.join()
        logger.error(
            'Loop closure registration timed out '
            f'after {timeout_s:.1f}s; rejecting candidate.')
        return identity_transform_msg(), False

    if process.exitcode != 0:
        logger.error(
            'Loop closure registration worker crashed with exit code '
            f'{format_process_exitcode(process.exitcode)}; rejecting candidate '
            'without stopping the map manager.')
        return identity_transform_msg(), False

    try:
        success, translation, rotation, error = result_queue.get(timeout=1.0)
    except queue.Empty:
        logger.error(
            'Loop closure registration worker exited without a result; '
            'rejecting candidate.')
        return identity_transform_msg(), False

    if error is not None:
        logger.error(
            'Loop closure registration failed with exception in worker: '
            f'{error}')
        return identity_transform_msg(), False

    if not success:
        return identity_transform_msg(), False

    return to_transform_msg(np.asarray(translation), np.asarray(rotation)), True
