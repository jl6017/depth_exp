import open3d as o3d

pcd = o3d.io.read_point_cloud("data/reconstruction.ply")
# estimate normals
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

# remove noise
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
filtered_pcd = pcd.select_by_index(ind)


o3d.visualization.draw_geometries([filtered_pcd])
