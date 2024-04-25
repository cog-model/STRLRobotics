import numpy as np
import cv2
import open3d as o3d


class ObjectPoseEstimation:
    def __init__(self, gt_point_cloud, voxel_size,
            max_correspondence_distances,
            fitness_threshold=0.95):
        self.voxel_size = voxel_size
        self.max_correspondence_distances = max_correspondence_distances
        self.fitness_threshold = fitness_threshold

        if isinstance(gt_point_cloud, np.ndarray):
            self.gt_pc = o3d.geometry.PointCloud()
            self.gt_pc.points = o3d.utility.Vector3dVector(gt_point_cloud)
        else:
            self.gt_pc = o3d.geometry.PointCloud(gt_point_cloud)
        self.gt_pc_down, self.gt_fpfh = self._preprocess_pc(self.gt_pc, no_noise_allowed=True)

        self.pc = None
        self.pc_down = None
        self.global_reg = None
        self.reg = None
        self.fitness = None

        self.reason = None

    def estimate_pose(self, point_cloud):
        self.pc = None
        self.pc_down = None
        self.global_reg = None
        self.reg = None
        self.fitness = None

        self.reason = None

        if len(point_cloud) < 1000:
            self.reason = \
                f"Too few points in extracted point cloud " \
                f"({len(point_cloud)} points)"
            return None

        self.pc = o3d.geometry.PointCloud()
        self.pc.points = o3d.utility.Vector3dVector(point_cloud)
        self.pc_down, fpfh = self._preprocess_pc(self.pc)
        if self.pc_down is None:
            return None

        try:
            self.global_reg = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                self.gt_pc_down, self.pc_down, self.gt_fpfh, fpfh,
                mutual_filter=True, max_correspondence_distance=self.voxel_size * 1.5,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                ransac_n=3,
                checkers=[
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(self.voxel_size * 1.5)],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        except:
            self.reason = "Global registration failed"
            return None

        pose = self.global_reg.transformation
        for max_correspondence_distance in self.max_correspondence_distances:
            self.reg = o3d.pipelines.registration.registration_icp(
                self.gt_pc_down, self.pc_down,
                max_correspondence_distance, init=pose,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
            pose = self.reg.transformation

        result = o3d.pipelines.registration.evaluate_registration(
            self.pc_down, self.gt_pc_down, self.voxel_size * 1.5,
            transformation=np.linalg.inv(pose))
        self.fitness = result.fitness
        if self.fitness < self.fitness_threshold:
            self.reason = f"Low fitness ({self.fitness})"
            return None

        return pose

    def _preprocess_pc(self, pc, no_noise_allowed=False):
        pc_down = pc.voxel_down_sample(self.voxel_size)

        pc_clusters_labels = pc_down.cluster_dbscan(self.voxel_size * 2, 0)
        pc_clusters_labels = np.array(pc_clusters_labels)
        unique_labels, counts = np.unique(pc_clusters_labels, return_counts=True)
        if no_noise_allowed:
            assert -1 not in unique_labels, \
                "There are noise points after clustering " \
                "and no_noise_allowed is True"
        label = unique_labels[np.argmax(counts)]
        if label == -1:
            self.reason = "Too many noise points after clustering"
            return None, None
        indices = np.where(pc_clusters_labels == label)[0]
        pc_down = pc_down.select_by_index(indices)

        pc_down.estimate_normals(search_param=
            o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(pc_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 5, max_nn=100))
        return pc_down, fpfh


def get_box_point_cloud(edges_sizes, points_per_cm, shift=(0, 0, 0), skip=tuple()):
    # 'shift' shifts coordinate system, not points
    # example of using 'skip': skip=('x-', 'y+')

    def _get_box_face(edges_sizes, points_per_cm, axis_index, displacement):
        face_axes_indices = np.delete(np.array([0, 1, 2]), axis_index)
        face_edges_sizes = edges_sizes[face_axes_indices]
        face = np.mgrid[
            -face_edges_sizes[0] / 2 : face_edges_sizes[0] / 2 : int(face_edges_sizes[0] * 100 * points_per_cm) * 1j,
            -face_edges_sizes[1] / 2 : face_edges_sizes[1] / 2 : int(face_edges_sizes[1] * 100 * points_per_cm) * 1j]
        face = face.reshape(2, -1).swapaxes(0, 1)
        face = np.hstack((face, np.full((len(face), 1), displacement)))
        axes_order = np.hstack((face_axes_indices, axis_index))
        face[:, axes_order] = face[:, [0, 1, 2]]
        return face

    faces = list()
    edges_sizes = np.array(edges_sizes)
    for axis_index in (0, 1, 2):
        edge_size = edges_sizes[axis_index]
        for di, displacement in enumerate((-edge_size / 2, edge_size / 2)):
            current_face = ('x', 'y', 'z')[axis_index] + ('-', '+')[di]
            if current_face in skip:
                continue
            face = _get_box_face(edges_sizes, points_per_cm, axis_index, displacement)
            faces.append(face)
    points = np.vstack(faces)

    points -= shift
    return points


def load_point_cloud(stl_file):
    mesh = o3d.io.read_triangle_mesh(stl_file)
    mesh.scale(0.001, np.array([0., 0., 0.]))
    pc = mesh.sample_points_poisson_disk(number_of_points=10000)
    return pc


def align_poses(ref_pose, pose):
    pose = pose.copy()
    ref_z = ref_pose[:3, 2]
    z = pose[:3, 2]
    if np.dot(ref_z, z) < 0:
        correction, _ = cv2.Rodrigues(np.array([np.pi, 0, 0]))
        pose[:3, :3] = np.matmul(pose[:3, :3], correction)

    ref_x = ref_pose[:3, 0]
    x = pose[:3, 0]
    if np.dot(ref_x, x) < 0:
        correction, _ = cv2.Rodrigues(np.array([0, 0, np.pi]))
        pose[:3, :3] = np.matmul(pose[:3, :3], correction)

    return pose


def align_poses_90(ref_pose, pose):
    pose = pose.copy()
    ref_z = ref_pose[:3, 2]
    z = pose[:3, 2]
    if np.dot(ref_z, z) < -0.5:  # 180 deg
        correction, _ = cv2.Rodrigues(np.array([np.pi, 0, 0]))
        pose[:3, :3] = np.matmul(pose[:3, :3], correction)
    elif np.dot(ref_z, z) < 0.5:  # 90 deg
        rot_axis = np.cross(z, ref_z)
        x = pose[:3, 0]
        y = pose[:3, 1]
        if abs(np.dot(x, rot_axis)) > abs(np.dot(y, rot_axis)):
            # x axis rotation
            correction, _ = cv2.Rodrigues(
                np.array([np.pi / 2 * np.sign(np.dot(x, rot_axis)), 0, 0]))
            pose[:3, :3] = np.matmul(pose[:3, :3], correction)
        else:
            # y axis rotation
            correction, _ = cv2.Rodrigues(
                np.array([0, np.pi / 2 * np.sign(np.dot(y, rot_axis)), 0]))
            pose[:3, :3] = np.matmul(pose[:3, :3], correction)

    ref_x = ref_pose[:3, 0]
    ref_y = ref_pose[:3, 1]
    x = pose[:3, 0]
    z_correction_angle = None
    if np.dot(ref_x, x) < -0.5:
        z_correction_angle = np.pi
    elif np.dot(ref_x, x) < 0.5:
        if np.dot(ref_y, x) < -0.5:
            z_correction_angle = np.pi / 2
        else:
            z_correction_angle = -np.pi / 2
    if z_correction_angle is not None:
        correction, _ = cv2.Rodrigues(np.array([0, 0, z_correction_angle]))
        pose[:3, :3] = np.matmul(pose[:3, :3], correction)

    return pose
