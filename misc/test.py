
from spot_control_suite.util.terrain_utils import get_terrain_urdf, get_ig_terrain_urdf
from spot_control_suite.util.asset_utils import get_asset_path, AddSpotRemote

import numpy as np
from pydrake.all import(Meldis, 
                        RobotDiagramBuilder, 
                        AddDefaultVisualization, 
                        Rgba, 
                        Sphere, 
                        RigidTransform,
                        RotationMatrix)
from functools import partial

from spot_control_suite.util.isaac_gym_terrain_generation import terrain_types
#path_terrain_urdf, points, triangles = get_terrain_urdf(length, width, height, name)
path_terrain_urdf, points, triangles = get_ig_terrain_urdf('test', terrain_types[0], xlen=8, ylen=8)

