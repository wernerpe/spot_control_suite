import numpy as np
from scipy.spatial import Delaunay
import os

def terrain_to_trimesh(length: float, 
                       width: float, 
                       heightmap: np.ndarray, 
                       name :str):
    resolution = list(heightmap.shape)
    # Generate a grid of height points
    x, y = np.meshgrid(np.linspace(0, length, resolution[0])-length/2, np.linspace(0, width, resolution[1])-width/2)
    z = heightmap 
    # Flatten the arrays for Delaunay triangulation
    points = np.column_stack((x.flatten(), y.flatten(), z.flatten()))

    # Perform Delaunay triangulation
    tri = Delaunay(points[:, :2])

    with open(name +'.obj', 'w') as obj_file:
        obj_file.write("# terrain\n")

        # Write vertices
        for vertex in points:
            obj_file.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")

        # Write faces
        for face in tri.simplices:
            obj_file.write(f"f {' '.join(map(str, [idx+1 for idx in face]))}\n")

    path_file = os.path.dirname(os.path.abspath(__file__))+'/'+name +'.obj'
    return path_file, points, [s for s in tri.simplices]

def get_terrain_urdf(length: float, 
                    width: float, 
                    heightmap: np.ndarray, 
                    name :str):
    
    path, verts, faces = terrain_to_trimesh(length, width, heightmap, name)
    sdf_file = f"""<?xml version="1.0"?>
<robot name="terrain">
    <link name="{name}">
    <visual name="{name}_visual">
        <geometry>
            <mesh filename="{name}.obj"/>
        </geometry>
    </visual>
    <collision name="{name}_visual">
        <geometry>
            <mesh filename="{name}.obj"/>
        </geometry>
    </collision>
    </link>
</robot>"""
    with open(f'terrain_{name}.urdf', 'w') as f:
        f.write(sdf_file)
        f.flush()
    path_file = os.path.dirname(os.path.abspath(__file__))+'/terrain_'+name +'.urdf'
    return path_file, verts, faces