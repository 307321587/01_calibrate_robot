import numpy as np
from scipy.spatial.transform import Rotation as R

def pbvs_straight(cur_wcT, tar_wcT):
    """PBVS2: goes straight and shortest path"""
    ctT = np.linalg.inv(cur_wcT) @ tar_wcT
    u = R.from_matrix(ctT[:3, :3]).as_rotvec()

    v = ctT[:3, 3]
    w = u
    vel = np.concatenate([w, v])

    return vel

