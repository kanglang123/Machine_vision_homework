
#######################################  ORB特征点提取和匹配  ###################################
# import cv2 as cv
# import numpy as np
# def ORB_Feature(img1, img2):
#     # 初始化ORB
#     orb = cv.ORB_create()
#     # 寻找关键点
#     kp1 = orb.detect(img1)
#     kp2 = orb.detect(img2)
#     # 计算描述符
#     kp1, des1 = orb.compute(img1, kp1)
#     kp2, des2 = orb.compute(img2, kp2)
#     # 画出关键点
#     outimg1 = cv.drawKeypoints(img1, keypoints=kp1, outImage=None)
#     outimg2 = cv.drawKeypoints(img2, keypoints=kp2, outImage=None)
#     # #显示关键点
#     # outimg3 = np.hstack([outimg1, outimg2])
#     # cv.imshow("Key Points", outimg3)
#     # cv.waitKey(0)
#     # 初始化 BFMatcher
#     bf = cv.BFMatcher(cv.NORM_HAMMING)
#     # 对描述子进行匹配
#     matches = bf.match(des1, des2)
#     # 计算最大距离和最小距离
#     min_distance = matches[0].distance
#     max_distance = matches[0].distance
#     for x in matches:
#         if x.distance < min_distance:
#             min_distance = x.distance
#         if x.distance > max_distance:
#             max_distance = x.distance
#     # 筛选匹配点
#     '''
#         当描述子之间的距离大于两倍的最小距离时，认为匹配有误。
#         但有时候最小距离会非常小，所以设置一个经验值30作为下限。
#     '''
#     good_match = []
#     for x in matches:
#         if x.distance <= max(2 * min_distance, 30):
#             good_match.append(x)
#     # 绘制匹配结果
#     draw_match(img1, img2, kp1, kp2, good_match)

# def draw_match(img1, img2, kp1, kp2, match):
#     outimage = cv.drawMatches(img1, kp1, img2, kp2, match, outImg=None)
#     cv.imshow("Match Result", outimage)
#     cv.waitKey(0)

# if __name__ == '__main__':
#     # 读取图片
#     image1 = cv.imread('image/1.jpg')
#     image2 = cv.imread('image/2.jpg')
#     ORB_Feature(image1, image2)
#######################################  ORB特征点提取和匹配  ###################################

#######################################  二维码识别  ###################################
# import numpy as np
# import cv2
# import cv2.aruco as aruco
# import glob
# import sys

# color = cv2.imread('image/1.jpg')
# cad = color.copy()
# gray = cv2.cvtColor(cad, cv2.COLOR_BGR2GRAY)
# aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
# parameters = aruco.DetectorParameters_create()

# # lists of ids and the corners beloning to each id
# corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

# font = cv2.FONT_HERSHEY_SIMPLEX

# if np.all(ids != None):
#     aruco.drawDetectedMarkers(cad, corners)  # Draw A square around the markers
#     # Display the resulting frame
#     cv2.imshow('Aruco detection on depth thresholded image', cad)
#     cv2.waitKey()
#######################################  二维码识别  ###################################

#######################################  open3d rgb2pointcloud/show point cloud/ICP  ###################################
import copy
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import cv2
import numpy as np

def draw_registration_result(source, target, transformation):
    '''
       显示点云配准结果
    '''

    source_temp = copy.deepcopy(source)#深复制，和原对象没有任何关联
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0, 0])#上色
    target_temp.paint_uniform_color([0, 1, 0])
    source_temp.transform(transformation)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size= 0.1, origin=[0, 0, 0])#建立坐标轴
    o3d.visualization.draw_geometries([source_temp, target_temp, mesh_frame],window_name='icp配准后点云')

# color_raw = o3d.io.read_image("image/1.jpg")
# depth_raw = o3d.io.read_image("image/1.png")
# rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(color_raw, depth_raw)#整合成RGBD图像

# # print(rgbd_image)

# # plt.subplot(1, 2, 1)
# # plt.title(' grayscale image')
# # plt.imshow(rgbd_image.color)
# # plt.subplot(1, 2, 2)
# # plt.title(' depth image')
# # plt.imshow(rgbd_image.depth)
# # plt.show()

# # #针眼相机的内参
# width, height = 640, 480
# cx = 312.83380126953125
# cy = 241.61764526367188
# fx = 622.0875244140625
# fy = 622.0875854492188
# intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
# pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
# # o3d.visualization.draw_geometries([pcd])
# o3d.io.write_point_cloud("source.pcd", pcd)

# #降采样
# # downpcd = pcd.voxel_down_sample(voxel_size=0.05)
# # o3d.visualization.draw_geometries([downpcd])
# good_pcd = o3d.io.read_point_cloud("data_2/data/pointcloud_extract_RGB_data/notplane/notplane1.pcd",format='auto')  
# mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
# target = o3d.io.read_point_cloud("image/shoes.ply",format='auto')  
# good_pcd = good_pcd.scale(0.00000002,(0,0,0))

# # print("good_pcd和target在frame下")
# # o3d.visualization.draw_geometries([target,good_pcd,mesh_frame])
# # print("配准初值")
# # draw_registration_result(good_pcd, target, np.identity(4))

# icp = o3d.pipelines.registration.registration_icp(
#         source=good_pcd,
#         target=target,
#         max_correspondence_distance=0.2,    # 距离阈值
#         estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
#     )
# print("配准后")
# print(icp.transformation)
# draw_registration_result(good_pcd, target, icp.transformation)   #需要进行平面滤波
# #######################################  open3d rgb2pointcloud/show point cloud/ICP  ###################################
