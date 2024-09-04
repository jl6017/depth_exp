import open3d as o3d
import numpy as np
import mcubes

import pyvista as pv
import pymeshfix


pcd = o3d.io.read_point_cloud("data/reconstruction.ply")
voxel_size = 0.015
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
o3d.visualization.draw_geometries([voxel_grid])

"""marching cubes"""

voxel_indices = np.asarray([voxel.grid_index for voxel in voxel_grid.get_voxels()])
print(voxel_indices.shape)

# Create an empty NumPy volume (3D array)
min_bound = voxel_grid.get_min_bound()
max_bound = voxel_grid.get_max_bound()

# Compute the size of the 3D array based on the voxel grid bounds and voxel size
volume_size = np.ceil((max_bound - min_bound) / voxel_size).astype(int)
volume = np.zeros(volume_size, dtype=bool)

volume = np.zeros(volume_size, dtype=np.uint8)

# Set occupied voxels to 1
for idx in voxel_indices:
    volume[idx[0], idx[1], idx[2]] = 1

threshold = 0.

# Apply marching cubes using PyMCubes
verts, faces = mcubes.marching_cubes(volume, threshold)

# scale the mesh
verts = verts * voxel_size + min_bound

# Create a mesh from the vertices and faces
mesh = o3d.geometry.TriangleMesh()

# Convert the vertices and faces to an Open3D mesh
mesh.vertices = o3d.utility.Vector3dVector(verts)
mesh.triangles = o3d.utility.Vector3iVector(faces)

# Visualize the mesh using Open3D
o3d.visualization.draw_geometries([mesh])

mesh_out = mesh.filter_smooth_simple(number_of_iterations=10)
# mesh_out = mesh.filter_smooth_laplacian(number_of_iterations=100)

mesh_out.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh_out])

# Convert the mesh to a PyVista mesh

vertices = np.asarray(mesh_out.vertices)
triangles = np.asarray(mesh_out.triangles)
# vclean, fclean = pymeshfix.clean_from_arrays(vertices, triangles)
meshfix = pymeshfix.MeshFix(vertices, triangles)
meshfix.plot()
meshfix.repair()
mesh_pv = meshfix.mesh

plotter = pv.Plotter()
plotter.add_mesh(mesh_pv, color='white')
plotter.show()