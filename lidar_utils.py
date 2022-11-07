import numpy as np
import numba as nb

# -------------------------------------------- NOTE: To Be Overwritten: START
@nb.njit(fastmath=True, cache=True)
def get_limits(manual=True):
    if manual:
        x_limits = np.array([ 0.65,1.7])
        y_limits = np.array([ -6,2.5])
        z_limits = np.array([ -2,-1.5])
        return x_limits, y_limits, z_limits


@nb.njit(fastmath=True, cache=True)
def get_rotation_angles(manual=True):
    if manual:
        x_rotation_angle = -1.75
        y_rotation_angle = -52.5
        z_rotation_angle = -2
        return x_rotation_angle, y_rotation_angle, z_rotation_angle  
# -------------------------------------------- NOTE: To Be Overwritten: END


# -------------------------------------------- Preprocessing Functions: START
@nb.njit(fastmath=True, cache=True)
def get_3d_rotation_matrix(x_rotation_angle, y_rotation_angle, z_rotation_angle):
    theta_xyz_degrees = np.array([x_rotation_angle, y_rotation_angle, z_rotation_angle])
    theta = theta_xyz_degrees * (np.pi/180)
    c, s = np.cos(theta), np.sin(theta)
    Rx = np.array([[1.0, 0.0, 0.0], 
        [0.0, c[0], -s[0]], 
        [0.0, s[0], c[0]]], dtype=np.float32)
    Ry = np.array([[c[1], 0.0, s[1]], 
        [0.0, 1.0, 0.0], 
        [-s[1], 0.0, c[1]]], dtype=np.float32)
    Rz = np.array([[c[2], -s[2], 0.0], 
        [s[2], c[2], 0.0], 
        [0.0, 0.0, 1.0]], dtype=np.float32)
    return Rx @ Ry @ Rz


@nb.njit(fastmath=True, cache=True)
def assign_velodyne_variables(pcd):
        pcd = np.stack((pcd['x'], pcd['y'], pcd['z'], pcd['intensity'], pcd['ring'])).T
        return pcd


@nb.njit(fastmath=True, cache=True)
def rotate(pcd, rotation_matrix):
    xyz = np.ascontiguousarray(pcd[:,0:3])
    pcd[:,0:3] =  xyz @ rotation_matrix
    return pcd


@nb.njit(fastmath=True, cache=True)
def trim(pcd, x_limits, y_limits, z_limits, intensity_threshold_min=0, intensity_threshold_max=150, threshold_on_intensity=True):
    cx = ((max(x_limits) >= pcd[:,0]) & (min(x_limits) <= pcd[:,0]))
    cy = ((max(y_limits) >= pcd[:,1]) & (min(y_limits) <= pcd[:,1]))
    cz = ((max(z_limits) >= pcd[:,2]) & (min(z_limits) <= pcd[:,2]))

    if threshold_on_intensity:
        ci = (pcd[:,3] > intensity_threshold_min) & (pcd[:,3] < intensity_threshold_max)
        mask = np.where(cx & cy & cz & ci)
    else:
        mask = np.where(cx & cy & cz)
    pcd = pcd[mask]
    return pcd


@nb.njit(fastmath=True, cache=True)
def sort_by_lateral_distance(pcd):
    lateral_distance_index = 1
    return pcd[np.argsort(pcd[:, lateral_distance_index])]


@nb.njit(fastmath=True, cache=True)
def preprocess_pcd(pcd, limits, rotation_angles):
    x_limits, y_limits, z_limits = limits
    x_rotation_angle, y_rotation_angle, z_rotation_angle = rotation_angles
    rotation_matrix = get_3d_rotation_matrix(x_rotation_angle, y_rotation_angle, z_rotation_angle)
    pcd = assign_velodyne_variables(pcd)
    pcd = rotate(pcd, rotation_matrix)
    pcd = trim(pcd, x_limits, y_limits, z_limits)
    pcd = sort_by_lateral_distance(pcd)
    return pcd 
# -------------------------------------------- Preprocessing Functions: END
