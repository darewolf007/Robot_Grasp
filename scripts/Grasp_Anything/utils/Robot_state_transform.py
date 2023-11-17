import numpy as np
from transforms3d.euler import euler2quat, quat2euler


#TODO add uv shape is 2
def transform_uv_to_xy(R, t, K, uv, depth):
    xy_reshape = np.array(uv).reshape((3, 1))
    t_reshape = np.array(t).reshape((3, 1))
    k_inv = np.linalg.inv(K)
    t_diff = depth * np.dot(k_inv, xy_reshape)
    world_xyz = np.dot(R, t_diff) + t_reshape
    return world_xyz.flatten()

def transfrom_angle_to_ri(angle_in_dgree):
    quat = np.array([0., 0, 1, 0])
    euler = np.asarray(quat2euler(quat, 'sxyz'))
    euler[2] += angle_in_dgree * np.pi / 180.
    while euler[2] < -np.pi:
        euler[2] += np.pi * 2
    while euler[2] > np.pi:
        euler[2] -= np.pi * 2
    quat = euler2quat(euler[0], euler[1], euler[2], 'sxyz')
    return (quat[1], quat[2], quat[3], quat[0])

def translation_matrix(translation):
    return np.array([
        [1, 0, 0, translation[0]],
        [0, 1, 0, translation[1]],
        [0, 0, 1, translation[2]],
        [0, 0, 0, 1]
    ])

def transform_Rt_to_T(R, t):
    T_rotation = np.zeros((4, 4))
    T_rotation[:3, :3] = R
    T_translation = translation_matrix(t)
    T = np.dot(T_translation, T_rotation)
    return T

def transform_xyz_to_uv(R_W_to_C, t_W_to_C, K, xyz, reverse=False):
    xyz_array = np.array(xyz).reshape(-1, 3)
    num_points = xyz_array.shape[0]
    T = transform_Rt_to_T(R_W_to_C, t_W_to_C)
    if reverse:
        T = np.linalg.inv(T)
    ones_column = np.ones((num_points, 1), dtype=xyz_array.dtype)
    xyz_homogeneous = np.hstack((xyz_array, ones_column))
    transformed_xyz_homogeneous = np.dot(T, xyz_homogeneous.T).T
    transformed_xyz_3d = transformed_xyz_homogeneous[:, :3] / transformed_xyz_homogeneous[:, 3, None]
    uvw_homogeneous = np.dot(K, transformed_xyz_3d.T).T
    uv = uvw_homogeneous[:, :2] / uvw_homogeneous[:, 2, None]
    return uv