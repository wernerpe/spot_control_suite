from spot_control_suite.util.terrain_utils import get_terrain_urdf, get_ig_terrain_sdf
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
path_terrain_sdf, points, triangles, origin_height = get_ig_terrain_sdf('test', 
                                                          'random', 
                                                          xlen=64., 
                                                          ylen=64.,
                                                          zscale=0.01,
                                                          resolution_x=128,
                                                          subterrain_splits=3,
                                                          centered=True)

terrain_name = path_terrain_sdf.split('/')[-1][:-5]
#meshcat = StartMeshcat()
meldis = Meldis()
meshcat = meldis.meshcat
builder = RobotDiagramBuilder()
plant = builder.plant()
scene_graph = builder.scene_graph()
parser = builder.parser()
AddSpotRemote(parser)
model = parser.AddModels(path_terrain_sdf)
spot_urdf = get_asset_path() + '/spot/spot_with_arm_and_floating_base_actuators.urdf'
model2 = parser.AddModels(spot_urdf)
frame = plant.GetFrameByName(f"origin", model[0])
plant.WeldFrames(plant.world_frame(), frame)
plant.Finalize()
visualizer = AddDefaultVisualization(builder.builder(), meshcat)
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = plant.GetMyContextFromRoot(diagram_context)
diagram.ForcedPublish(diagram_context)

zero_pos = np.zeros(plant.GetPositionLowerLimits().shape)
zero_pos[2] = origin_height + 0.52
plant.SetPositions(plant_context, zero_pos)
diagram.ForcedPublish(diagram_context)

print('pause')