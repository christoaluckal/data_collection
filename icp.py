# Script to perform global registration of two point clouds
import open3d as o3d
import numpy as np
import copy
import os
import time

def icp_register(src_cloud, dest_cloud, max_iter=10000, threshold=0.002, voxel_size = 0.05):

    start_time = time.time()
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_clouds(voxel_size,src_cloud, dest_cloud)

    # result_fast = execute_fast_global_registration(source_down, target_down,source_fpfh, target_fpfh, voxel_size)
    result_fast = execute_global_registration(source_down, target_down,source_fpfh, target_fpfh, voxel_size)

    trans_init = result_fast.transformation

    # Using identity matrix as initial transformation
    # trans_init = np.identity(4)

    registered_p2p = o3d.pipelines.registration.registration_icp(src_cloud, dest_cloud, threshold, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPoint())

    transformation = registered_p2p.transformation

    error = registered_p2p.inlier_rmse

    end_time = time.time()

    time_taken = end_time - start_time

    return trans_init, transformation, error, time_taken

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0, 0]) # Red
    target_temp.paint_uniform_color([0, 1, 0]) # Green
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])

def prepare_dataset(voxel_size, source_path, target_path):
    # Use with path to pcd files

    print(":: Load two point clouds and disturb initial pose.")

    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def prepare_clouds(voxel_size, source_cloud, target_cloud):
    # Use with open3d point clouds

    print(":: Load two point clouds and disturb initial pose.")

    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                          [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source_cloud.transform(trans_init)
    # draw_registration_result(source_cloud, target_cloud, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source_cloud, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target_cloud, voxel_size)
    return source_cloud, target_cloud, source_down, target_down, source_fpfh, target_fpfh

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

import sys

def main():
    voxel_size = 0.05  # means 5cm for this dataset
    source_path = sys.argv[1]
    target_path = sys.argv[2]


    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
        voxel_size, source_path, target_path)

    result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)

    # result_fast = execute_fast_global_registration(source_down, target_down,
    #                                            source_fpfh, target_fpfh,
    #                                            voxel_size)
    # print(result_fast)
    # print(result_fast.transformation)
    # draw_registration_result(source_down, target_down, result_fast.transformation)
    print(result_ransac)
    print(result_ransac.transformation)

if __name__ == "__main__":
    main()