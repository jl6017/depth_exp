import numpy as np
import open3d as o3d

T1 = np.array([
    [ -0.45224653 , 0.70710678 , -0.54357435 , 481.26000000 ],
    [ -0.45224653 , -0.70710678 , -0.54357435 , 481.26000000 ],
    [ -0.76873022 , 0.00000000 , 0.63957318 , 886.20000000 ],
    [ 0.00000000 , 0.00000000 , 0.00000000 , 1.00000000 ],
])

T2 = np.array([
    [ 0.45224653 , 0.70710678 , 0.54357435 , -481.26000000 ],
    [ -0.45224653 , 0.70710678 , -0.54357435 , 481.26000000 ],
    [ -0.76873022 , 0.00000000 , 0.63957318 , 886.20000000 ],
    [ 0.00000000 , 0.00000000 , 0.00000000 , 1.00000000 ],
])

T3 = np.array([
    [ 0.45224653 , -0.70710678 , 0.54357435 , -481.26000000 ],
    [ 0.45224653 , 0.70710678 , 0.54357435 , -481.26000000 ],
    [ -0.76873022 , 0.00000000 , 0.63957318 , 886.20000000 ],
    [ 0.00000000 , 0.00000000 , 0.00000000 , 1.00000000 ],
])

T4 = np.array([
    [ -0.45224653 , -0.70710678 , -0.54357435 , 481.26000000 ],
    [ 0.45224653 , -0.70710678 , 0.54357435 , -481.26000000 ],
    [ -0.76873022 , 0.00000000 , 0.63957318 , 886.20000000 ],
    [ 0.00000000 , 0.00000000 , 0.00000000 , 1.00000000 ],
])


def depth2pcd(depth, normal):
    # og intrinsics
    fx = 906.80
    fy = 906.85
    cx = 644.53
    cy = 353.05

    # Intrinsic parameters (from your earlier scaled parameters)
    # fx = 604.49
    # fy = 604.57
    # cx = 429.34
    # cy = 235.36

    # Create meshgrid of pixel coordinates (u, v)
    u = np.arange(width)
    v = np.arange(height)
    u, v = np.meshgrid(u, v)

    # Convert pixel coordinates to camera coordinates (x, y, z)
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth

    # Stack the camera coordinates
    camera_coords = np.stack([x, y, z], axis=-1) # (width, height, 3)
    points = camera_coords.reshape(-1, 3) # (width*height, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    normals = normal.reshape(-1, 3)
    pcd.normals = o3d.utility.Vector3dVector(normals)

    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    filtered_pcd = pcd.select_by_index(ind)
    return filtered_pcd

depth0 = np.load('data/depth0.npy')
normal0 = np.load('data/normal0.npy')
# depth1 = np.load('data/depth1.npy')
# normal1 = np.load('data/normal1.npy')
height, width = depth0.shape


# crop distance
depth_threshold = 10
depth0 = np.clip(depth0, 0, depth_threshold)
# depth1 = np.clip(depth1, 0, depth_threshold)


pcd0 = depth2pcd(depth0, normal0)
# pcd1 = depth2pcd(depth1, normal1)

# color the point cloud
pcd0.paint_uniform_color([1, 0.706, 0])
# pcd1.paint_uniform_color([0, 0.651, 0.929])

# add coordinate frame
cf1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

# mm to m
T1[:3, 3] = 0

# transform coordinate frame
cf2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
cf2 = cf2.transform(T1)

o3d.visualization.draw_geometries([pcd0, cf1, cf2])

