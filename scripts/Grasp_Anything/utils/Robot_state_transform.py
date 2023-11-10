import numpy as np
from transforms3d.euler import euler2quat, quat2euler


#TODO add uv shape is 2
def transform_uv_to_xy(R, T, K, uv, depth):
    xy_reshape = np.array(uv).reshape((3, 1))
    t_reshape = np.array(T).reshape((3, 1))
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