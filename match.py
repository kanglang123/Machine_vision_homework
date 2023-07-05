import open3d as o3d
import lib
import numpy as np
import time
import cv2

voxel_size=0.007

#整合成RGBD图像
color_raw = o3d.io.read_image("image/1.jpg")
depth_raw = o3d.io.read_image("image/1.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(color_raw, depth_raw)

# 针眼相机的内参
width, height = 640, 480
cx = 312.83380126953125
cy = 241.61764526367188
fx = 622.0875244140625
fy = 622.0875854492188
intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
source_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
# o3d.visualization.draw_geometries([source_pcd],window_name='原始点云')

target = o3d.io.read_point_cloud("image/shoes.ply",format='xyzrgb') #读取模板
# o3d.visualization.draw_geometries([target],window_name='模板点云')

#下采样
source_down = source_pcd.voxel_down_sample(voxel_size)
target_down = target.voxel_down_sample(voxel_size)
# o3d.visualization.draw_geometries([source_down,target_down],window_name='下采样后点云')

#RANSAC平面分割
not_plane_pcd=lib.ransac_segment_plane(source_down,distance_threshold=0.008,ransac_n=20,num_iterations=10)

#提取特征
not_plane_pcd_fpfh = lib.get_feature(not_plane_pcd, voxel_size)
target_fpfh = lib.get_feature(target_down, voxel_size)

start_time=time.time()

#全局配准
result_ransac = lib.execute_global_registration(not_plane_pcd, target_down,
                                           not_plane_pcd_fpfh, target_fpfh,
                                           voxel_size)

# lib.draw_registration_result(not_plane_pcd, target_down, result_ransac.transformation) 

#计算icp配准矩阵
# T=lib.icp_registration(source_down,target_down,result_ransac.transformation,0.01)
T = lib.point_to_plane_icp(not_plane_pcd,target_down,0.01,result_ransac.transformation)

end_time=time.time()
print("配准用时：")
print(end_time-start_time)

#显示点云配准结果
lib.draw_registration_result(source_pcd, target, T.transformation)  