import numpy as np
import os
import imageio
from scipy.spatial import Delaunay

path_tmp_dir = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
path_tmp_dir += "/tmp"


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
    
    path_file = path_tmp_dir + "/" + name + ".obj"
    with open(path_file, "w") as obj_file:
        obj_file.write("# terrain\n")

        # Write vertices
        for vertex in points:
            obj_file.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")

        # Write faces
        for face in tri.simplices:
            obj_file.write(f"f {' '.join(map(str, [idx+1 for idx in face]))}\n")

    return path_file, points, [s for s in tri.simplices]


def get_terrain_urdf(length: float, width: float, heightmap: np.ndarray, name: str):

    path, verts, faces = terrain_to_obj(length, width, heightmap, name)
    urdf_file = f"""<?xml version="1.0"?>
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
    path_file = path_tmp_dir + "/" + name + ".urdf"
    with open(path_file, "w") as f:
        f.write(urdf_file)
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
