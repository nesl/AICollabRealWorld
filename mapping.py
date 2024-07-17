import numpy as np
import cv2
import open3d as o3d
import math

class mapping:
    def __init__(self, voxel_size):
        # Initialize the mapping class with an empty point cloud, placeholders for occupancy grids, and a specified voxel size.
        self.point_cloud = np.empty((0, 3))
        self.occupancy_grid_3d = None
        self.occupancy_grid_2d = None
        self.voxel_size = voxel_size
    
    def _depth_to_point_cloud(self, depth_image, intrinsic_matrix):
        # Convert a depth image to a 3D point cloud using the intrinsic matrix. Filter points based on distance thresholds.
        height, width = depth_image.shape
        i, j = np.indices((height, width))
        z = depth_image
        x = (j - intrinsic_matrix[0, 2]) * z / intrinsic_matrix[0, 0]
        y = (i - intrinsic_matrix[1, 2]) * z / intrinsic_matrix[1, 1]
        
        point_cloud = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        distances = np.linalg.norm(point_cloud, axis=1)
        print(np.max(distances), np.min(distances))
        upper_mask = distances <= 500
        lower_mask = distances >= 50
        mask = upper_mask & lower_mask
        filtered_point_cloud = point_cloud[mask]
        return filtered_point_cloud

    def _transform_point_cloud(self, point_cloud, extrinsic_matrix):
        # Transform the point cloud from the camera frame to the world frame using the extrinsic matrix.
        ones = np.ones((point_cloud.shape[0], 1))
        homogenous_points = np.hstack((point_cloud, ones))
        transformed_points = homogenous_points @ extrinsic_matrix.T
        return transformed_points[:, :3]

    def update_point_cloud(self, depth_image, intrinsic_matrix, extrinsic_matrix):
        # Update the point cloud by converting a new depth image and applying a transformation.
        pc = self._depth_to_point_cloud(depth_image, intrinsic_matrix)
        transformed_pc = self._transform_point_cloud(pc, extrinsic_matrix)
        self.point_cloud = np.vstack((self.point_cloud, transformed_pc))
    
    def update_occupancy_grid(self):
        # Create and update the 3D occupancy grid based on the current point cloud. Also generate a 2D slice of the grid.
        min_bound = np.min(self.point_cloud, axis=0)
        max_bound = np.max(self.point_cloud, axis=0)
        grid_shape = np.ceil((max_bound - min_bound) / self.voxel_size).astype(int)
        occupancy_grid = np.zeros(grid_shape, dtype=int)
        voxel_indices = ((self.point_cloud - min_bound) / self.voxel_size).astype(int)
        flat_indices = np.ravel_multi_index(voxel_indices.T, occupancy_grid.shape)
        counts = np.bincount(flat_indices, minlength=occupancy_grid.size)
        occupancy_grid = counts.reshape(occupancy_grid.shape)
        occupancy_grid = occupancy_grid > 200 # Apply a threshold to determine occupied voxels
        occupancy_grid = occupancy_grid.transpose(0, 2, 1)
        occupancy_grid = occupancy_grid[::-1, :, ::-1]
        self.occupancy_grid_3d = occupancy_grid
        self.occupancy_grid_2d = occupancy_grid[:, :, 5]
    
    def visualize_point_cloud(self):
        # Visualize the current point cloud using Open3D.
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_cloud)
        o3d.visualization.draw_geometries([pcd])
    
if __name__ == '__main__':
    import BEV
    import matplotlib.pyplot as plt
    orientation = [0, 0, 180, 180, 90, 90, 90, 0, 0, 180]
    position = [[0, 0], [0, 100], [0, 100], [0, 0], [0, 0], [100, 0], [200, 0], [200, 0], [200, 100], [200, 100]]
    intrin_matrix = np.array([[456.390625, 0, 314.60546875], [0, 456.85546875, 250.8203125], [0, 0, 1]])
    map = mapping(15)
    for i in range(10):
        depth_frame = np.loadtxt(f'depth_image{i + 1}.csv', delimiter=',')
        depth_image = np.array(depth_frame)
        extrin_matrix = BEV.get_extrinsic_matrix(0, math.radians(orientation[i]), 0, position[i][0], 30, position[i][1])
        map.update_point_cloud(depth_image, intrin_matrix, extrin_matrix)

    map.update_occupancy_grid()
    fig, ax = plt.subplots()
    cax = ax.imshow(map.occupancy_grid_2d, cmap='gray', origin='lower')

    # Adding grid lines
    ax.grid(True, which='both', color='k', linestyle='-', linewidth=0.5)
    ax.set_xticks(np.arange(0.5, map.occupancy_grid_2d.shape[1], 1))
    ax.set_yticks(np.arange(0.5, map.occupancy_grid_2d.shape[0], 1))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    plt.show()