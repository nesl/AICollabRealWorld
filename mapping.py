import numpy as np
import cv2
import open3d as o3d
import math

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    outlier_cloud.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

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
        occupancy_grid = occupancy_grid > 300 # Apply a threshold to determine occupied voxels
        occupancy_grid = occupancy_grid.transpose(0, 2, 1)
        occupancy_grid = occupancy_grid[::-1, :, ::-1]
        self.occupancy_grid_3d = occupancy_grid
        self.occupancy_grid_2d = occupancy_grid[:, :, 5]
    
    def visualize_point_cloud(self):
        # Visualize the current point cloud using Open3D.
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_cloud)
        o3d.visualization.draw_geometries([pcd])


class mapping_o3d:
    def __init__(self, voxel_size):
        self.point_cloud_o3d = None #type o3d pointcloud
        self.occupancy_grid_3d = None
        self.occupancy_grid_2d = None
        self.voxel_size = voxel_size / 1100
        self.min_height = -20 / 1100
        self.max_height = 270 / 1100
    
    def update_point_cloud(self, color_image, depth_image, intrinsic_matrix, extrinsic_matrix,
                           nb_neighbors=20, std_ratio=1.5, nb_points=20, radius=0.02):
        depth_image = depth_image.astype(np.float32)
        depth_height, depth_width = depth_image.shape
        color_image = cv2.resize(color_image, (depth_width, depth_height), interpolation=cv2.INTER_LINEAR)

        color_img = o3d.geometry.Image(color_image)
        depth_img = o3d.geometry.Image(depth_image)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_img, depth_img
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, 
            o3d.cpu.pybind.camera.PinholeCameraIntrinsic(
                depth_width, depth_height, intrinsic_matrix
            ), 
            extrinsic_matrix
        )
        pcd_downsampled = pcd.voxel_down_sample(voxel_size=0.005)
        stats_cl, stats_ind = pcd_downsampled.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        radius_cl, radius_ind = pcd_downsampled.remove_radius_outlier(nb_points=nb_points, radius=radius)
        intersection_ind = [value for value in stats_ind if value in radius_ind]
        if self.point_cloud_o3d is None:
            self.point_cloud_o3d = pcd_downsampled.select_by_index(intersection_ind)
        else:
            self.point_cloud_o3d += pcd_downsampled.select_by_index(intersection_ind)
        self.point_cloud_o3d = self.point_cloud_o3d.voxel_down_sample(voxel_size=0.005)
        
    def update_occupancy_map(self):
        point_cloud_array = np.asarray(self.point_cloud_o3d.points)
        min_bound = np.min(point_cloud_array, axis=0)
        max_bound = np.max(point_cloud_array, axis=0)
        grid_shape = np.ceil((max_bound - min_bound) / self.voxel_size).astype(int)
        occupancy_grid = np.zeros(grid_shape, dtype=int)
        voxel_indices = ((point_cloud_array - min_bound) / self.voxel_size).astype(int)
        flat_indices = np.ravel_multi_index(voxel_indices.T, occupancy_grid.shape)
        counts = np.bincount(flat_indices, minlength=occupancy_grid.size)
        occupancy_grid = counts.reshape(occupancy_grid.shape)
        occupancy_grid = occupancy_grid > 4 # Apply a threshold to determine occupied voxels
        occupancy_grid = occupancy_grid.transpose(0, 2, 1)
        occupancy_grid = occupancy_grid[::-1, :, ::-1]
        
        self.occupancy_grid_3d = occupancy_grid
        self.occupancy_grid_2d = occupancy_grid[:, :, 6]

    def get_point_cloud_as_numpy_array(self):
        return np.asarray(self.point_cloud_o3d.points)

    def visualize_point_cloud_o3d(self):
        o3d.visualization.draw_geometries([self.point_cloud_o3d])



if __name__ == '__main__':
    import BEV
    import matplotlib.pyplot as plt
    import os
    from PIL import Image
    directory_path = "test16"
    orientation = np.loadtxt(os.path.join(directory_path, "orientation.csv"))
    position = np.loadtxt(os.path.join(directory_path, "position.csv"), delimiter=',')
    intrin_matrix = np.array([[456.390625, 0, 314.60546875], [0, 456.85546875, 250.8203125], [0, 0, 1]])

    map = mapping_o3d(15)
    for i in range(orientation.shape[0]):
        depth_frame = np.loadtxt(os.path.join(directory_path, f'depth_image{i + 1}.csv'), delimiter=',')
        depth_image = np.array(depth_frame)
        x_displacement = np.sin(-math.radians(orientation[i])) * 7 + np.cos(-math.radians(orientation[i])) * 4
        y_displacement = np.cos(-math.radians(orientation[i])) * 7 + np.sin(-math.radians(orientation[i])) * 4
        extrin_matrix = BEV.get_extrinsic_matrix(0, -math.radians(orientation[i]), 0, (position[i][1] + x_displacement) / 1100, 30, (position[i][0] + y_displacement) / 1100)

        color_image = Image.open(os.path.join(directory_path, f"color_image{i+1}.png"))
        color_image_array = np.array(color_image)

        depth_image_array = np.loadtxt(os.path.join(directory_path, f"depth_image{i+1}.csv"), delimiter=',')
        map.update_point_cloud(color_image_array, depth_image_array, intrin_matrix, extrin_matrix)
        print(i, "/", orientation.shape[0])
    
    map.visualize_point_cloud_o3d()
    map.update_occupancy_map()
    fig, ax = plt.subplots()
    cax = ax.imshow(map.occupancy_grid_2d, cmap='gray', origin='lower')

    # Adding grid lines
    ax.grid(True, which='both', color='k', linestyle='-', linewidth=0.5)
    ax.set_xticks(np.arange(0.5, map.occupancy_grid_2d.shape[1], 1))
    ax.set_yticks(np.arange(0.5, map.occupancy_grid_2d.shape[0], 1))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    plt.show()



    '''
    map = mapping(15)
    for i in range(orientation.shape[0]):
        depth_frame = np.loadtxt(os.path.join(directory_path, f'depth_image{i + 1}.csv'), delimiter=',')
        depth_image = np.array(depth_frame)
        extrin_matrix = BEV.get_extrinsic_matrix_v2(0, math.radians(orientation[i]), 0, position[i][0], 30, position[i][1])
        map.update_point_cloud(depth_image, intrin_matrix, extrin_matrix)

    map.visualize_point_cloud()
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
    '''