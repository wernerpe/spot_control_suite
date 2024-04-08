import numpy as np
import os
import imageio
from scipy.spatial import Delaunay
from .isaac_gym_terrain_generation import *
import pyvista as pv
import tetgen
import pymeshlab as ml
import time
    
path_tmp_dir = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
path_tmp_dir += "/tmp"

def save_obj(name, vertices, triangles):
    path_file = path_tmp_dir + "/" + name + ".obj"
    with open(path_file, "w") as obj_file:
        obj_file.write("# terrain\n")

        # Write vertices
        for vertex in vertices:
            obj_file.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")

        # Write faces
        for face in triangles:
            obj_file.write(f"f {' '.join(map(str, [idx+1 for idx in face]))}\n")
    return path_file

def save_tet_vtk(name, vertices, triangles, debug_viz = False):
    #fix non manifold vertices
    ms = ml.MeshSet()
    # Add vertices and faces to the MeshSet
    ms.add_mesh(ml.Mesh(vertices, triangles))
     # Apply filters to fix non-manifold mesh errors.
    ms.apply_filter('meshing_repair_non_manifold_edges')
    #ms.apply_filter('meshing_repair_non_manifold_vertices')
    mesh_fixed = ms.current_mesh()
    tet = tetgen.TetGen(mesh_fixed.vertex_matrix(), mesh_fixed.face_matrix())
    t1 = time.time()
    tet.tetrahedralize(quality=True, nobisect=True, nomergefacet=True,
                    nomergevertex=True, vtksurfview=True, vtkview=True,
                    verbose=True)
    t2 = time.time()
    path_vtk = path_tmp_dir + "/" + name + ".vtk"
    tet.write(path_vtk, binary=False)
    print(f"time {t2-t1}")

    #debugging visualization of tetmesh
    if debug_viz:
        grid = tet.grid
        grid.plot(show_edges = True)

    return path_vtk

def get_terrain_urdf_from_obj(obj_name):
    urdf_string = f"""<?xml version="1.0"?>
<robot name="terrain">
<material name="terrain_color">
    <color rgba="0.7 0.7 0.7 1"/> 
</material>
    <link name="{obj_name}">
    <visual name="{obj_name}_visual">
        <geometry>
            <mesh filename="{obj_name}.obj"/>
        </geometry>
        <material name="terrain_color"/>
    </visual>
    <collision name="{obj_name}_collision">
        <geometry>
            <mesh filename="{obj_name}.obj"/>
        </geometry>
    </collision>
    </link>
</robot>"""
    return urdf_string

def get_terrain_sdf_from_vtk(vtk_name):
    sdf_string = f"""<?xml version="1.0"?>
<sdf version="1.7" xmlns:drake="drake.mit.edu">
  <model name="{vtk_name}">
    <link name="{vtk_name}">
    <visual name="visual">
        <geometry>
          <mesh>
            <uri>{vtk_name}.obj</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
    </visual>
    <collision name="{vtk_name}_collision">
        <geometry>
          <mesh>
            <uri>{vtk_name}.vtk</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <drake:proximity_properties>
          <drake:compliant_hydroelastic/>
          <drake:hydroelastic_modulus>5.0e4</drake:hydroelastic_modulus>
          <!-- Most shapes (capsule, cylinder, ellipsoid, sphere) need
            drake:mesh_resolution_hint, but the resolution hint is no-op
            for the mesh geometry. That's why we do not set it here. -->
          <drake:hunt_crossley_dissipation>10</drake:hunt_crossley_dissipation>
          <!-- Both mu_dynamic and mu_static are used in Continuous system.
            Only mu_dynamic is used in Discrete system.  -->
          <drake:mu_dynamic>0.5</drake:mu_dynamic>
          <drake:mu_static>1.0</drake:mu_static>
        </drake:proximity_properties>
    </collision>
    </link>
    <frame name="origin">
      <pose relative_to="{vtk_name}">0 0 0 0 0 0</pose>
    </frame>
  </model>
</sdf>"""
    return sdf_string

class IsaacTerrain:
    def __init__(self, 
                 width = 100, 
                 length= 100, 
                 horizontal_scale = 0.1, 
                 vertical_scale = 0.1,
                 subterrain_splits_per_side = 1):
        assert subterrain_splits_per_side>=1
        self.height_field_raw = np.zeros((width, length))
        self.horizontal_scale = horizontal_scale
        self.vertical_scale = vertical_scale
        self.width = width
        self.length = length
        self.subterrain_splits_per_side = subterrain_splits_per_side
        self.sub_width = int(self.width/self.subterrain_splits_per_side)
        self.sub_length = int(self.length/self.subterrain_splits_per_side)

    def get_subterrain(self,):
        return IsaacTerrain(self.sub_width, self.sub_length, self.horizontal_scale, self.vertical_scale)
        
    def set_subterrain(self, height_field_subterrain, idx_x, idx_y):
        assert idx_x >=0 and idx_x< self.subterrain_splits_per_side
        assert idx_y >=0 and idx_y< self.subterrain_splits_per_side
        self.height_field_raw[idx_x*self.sub_width : (idx_x+1)*self.sub_width,idx_y*self.sub_length : (idx_y+1)*self.sub_length] = height_field_subterrain

def fill_terrain_type(terrain: IsaacTerrain, terrain_type: str, zscale =0.1):
    if terrain_type == 'random':
        terr_type = terrain_types[np.random.choice(len(terrain_types))]#
    else:
        terr_type = terrain_type
    if terr_type == 'random_uniform':
        terrain = random_uniform_terrain(terrain, 
                                         min_height=-1*zscale, 
                                         max_height=1*zscale,
                                         step=0.1, 
                                         downsampled_scale= 0.3)
    elif terr_type == 'sloped':
        terrain = sloped_terrain(terrain, slope = 0.1)
    elif terr_type == 'pyramid_sloped':
        terrain = pyramid_sloped_terrain(terrain, slope = 0.5, platform_size = 1)
    elif terr_type == 'discrete_obstacles':
        terrain = discrete_obstacles_terrain(terrain, 
                                             max_height = 0.2, 
                                             min_size = 1.3, 
                                             max_size= 3.6, 
                                             num_rects =50, 
                                             platform_size=(terrain.sub_width*terrain.horizontal_scale)/10.)
    elif terr_type == 'wave':
        terrain = wave_terrain(terrain, num_waves=4, amplitude=0.1)
    elif terr_type == 'stairs':
        terrain = stairs_terrain(terrain, step_height=0.2, step_width=terrain.width*terrain.horizontal_scale/20)
    elif terr_type == 'pyramid_stairs':
        terrain = pyramid_stairs_terrain(terrain, step_height=0.2, step_width=terrain.width*terrain.horizontal_scale/20.0, platform_size=terrain.width*terrain.horizontal_scale/4)
    elif terr_type == 'stepping_stones':
        len_sc = terrain.width*terrain.horizontal_scale
        terrain = stepping_stones_terrain(terrain, len_sc/10, len_sc/35, 0.2, platform_size=1., depth=-0.8)

    
    # elif terrain_type == 'stepping_stones':
    return terrain

def get_ig_terrain_sdf(name, 
                        terrain_type, 
                        xlen= 8.0, 
                        ylen = 8.0, 
                        zscale=0.1, 
                        subterrain_splits=1, 
                        resolution_x = 256,
                        centered=True):
    np.random.seed(1)
    assert terrain_type in terrain_types+['random']
    horizontal_scale = xlen/resolution_x
    length = int(ylen/horizontal_scale)
    terrain = IsaacTerrain(width= resolution_x, 
                           length=length, 
                           horizontal_scale=horizontal_scale, 
                           vertical_scale=zscale,
                           subterrain_splits_per_side=subterrain_splits)
    sub_terrain_idx = [[i, j] for i in range(subterrain_splits) for j in range(subterrain_splits)]
    subterrain = terrain.get_subterrain()
    for pair in sub_terrain_idx:
        subterrain.height_field_raw[:] = 0
        subterrain = fill_terrain_type(subterrain, terrain_type)
        terrain.set_subterrain(subterrain.height_field_raw, pair[0], pair[1])
    
    verts, triangles= convert_heightfield_to_trimesh(terrain.height_field_raw, 
                                   terrain.horizontal_scale, 
                                   terrain.vertical_scale,
                                   slope_threshold=None)
    terrain_name = name+'_'+terrain_type
    if centered:
        verts[:,0] -= xlen/2.0
        verts[:,1] -= ylen/2.0
    path_file = save_obj(terrain_name, verts, triangles)
    path_vtk_file = save_tet_vtk(terrain_name, 
                                 verts, 
                                 triangles, 
                                 debug_viz=True)
    sdf_string = get_terrain_sdf_from_vtk(terrain_name)
    path_sdf = path_tmp_dir + "/" + terrain_name + ".sdf"
    with open(path_sdf, "w") as f:
        f.write(sdf_string)
        f.flush()

    return path_sdf, verts, triangles, zscale*terrain.height_field_raw[int(resolution_x/2), int(length/2)]


def terrain_to_obj(length: float, width: float, heightmap: np.ndarray, name: str):

    assert len(heightmap.shape) == 2

    resolution = list(heightmap.shape)
    # Generate a grid of height points
    x, y = np.meshgrid(
        np.linspace(0, length, resolution[0]) - length / 2,
        np.linspace(0, width, resolution[1]) - width / 2,
    )
    z = heightmap
    
    # Triangulate 
    points = np.column_stack((x.flatten(), y.flatten(), z.flatten()))
    tri = Delaunay(points[:, :2])
    path_file = save_obj(name, points, tri.simplices)
    return path_file, points, [s for s in tri.simplices]


def get_terrain_urdf(length: float, width: float, heightmap: np.ndarray, name: str):

    path, verts, faces = terrain_to_obj(length, width, heightmap, name)
    urdf_string = get_terrain_urdf_from_obj(name)
    path_file = path_tmp_dir + "/" + name + ".urdf"
    with open(path_file, "w") as f:
        f.write(urdf_string)
        f.flush()

    return path_file, verts, faces


def get_terrain_asset_from_bitmap(path_bitmap, scale_x=0.1, scale_y=0.1, scale_z=0.1):
    image = imageio.imread(path_bitmap)
    name = (path_bitmap.split("/")[-1]).split(".")[0]
    # Convert the image to a NumPy array
    hmap = np.array(image)
    return get_terrain_urdf(
        length=scale_x * hmap.shape[0],
        width=scale_y * hmap.shape[1],
        heightmap=hmap * scale_z,
        name=name,
    )
