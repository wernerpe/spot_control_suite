
from spot_control_suite.util.terrain_utils import get_terrain_urdf

import numpy as np

resolution = [20, 20]
height = np.random.rand(*resolution)*0.1
name = 'test_trimesh'
length = 1
width = 1

path_urdf, points, triangles = get_terrain_urdf(length, width, height, name)
