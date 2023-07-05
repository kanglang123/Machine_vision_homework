import copy
import open3d as o3d


def statistical_filter(source_pcd,num_neighbors = 10,std_ratio = 1.5):
   # ------------------------- 统计滤波 --------------------------
   '''
      num_neighbors  K邻域点的个数\n
      std_ratio      标准差乘数\n
   '''
   print("->正在进行统计滤波...")

   # 执行统计滤波，返回滤波后的点云sor_pcd和对应的索引ind
   sor_pcd, ind = source_pcd.remove_statistical_outlier(num_neighbors, std_ratio)
   sor_pcd.paint_uniform_color([0, 0, 1])
   print("统计滤波后的点云：", sor_pcd)
   sor_pcd.paint_uniform_color([0, 0, 1])
   # 提取噪声点云
   sor_noise_pcd = source_pcd.select_by_index(ind,invert = True)
   print("噪声点云：", sor_noise_pcd)
   sor_noise_pcd.paint_uniform_color([1, 0, 0])
   print("->正在可视化统计滤波点云")
   # o3d.visualization.draw_geometries([sor_pcd,sor_noise_pcd],window_name='统计滤波后点云')
   return sor_pcd

def ransac_segment_plane(source_pcd,distance_threshold = 0.008,ransac_n = 30,num_iterations = 100):
   # ------------------------- RANSAC平面分割 --------------------------
   '''
      distance_threshold  内点到平面模型的最大距离\n
      ransac_n = ransac_n 用于拟合平面的采样点数\n
      num_iterations = num_iterations  最大迭代次数
   '''
   print("->正在RANSAC平面分割...")
   # 返回模型系数plane_model和内点索引inliers，并赋值
   plane_model, inliers = source_pcd.segment_plane(distance_threshold, ransac_n, num_iterations)

   # 输出平面方程
   [a, b, c, d] = plane_model
   print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

   # 平面内点点云
   inlier_cloud = source_pcd.select_by_index(inliers)
   inlier_cloud.paint_uniform_color([0, 0, 1.0])
   print(inlier_cloud)

   # 平面外点点云 ---鞋垫点云
   outlier_cloud = source_pcd.select_by_index(inliers, invert=True)
   outlier_cloud.paint_uniform_color([1.0, 0, 0])
   print(outlier_cloud)

   print("->正在可视化RANSAC平面分割点云")
   # 可视化平面分割结果
   o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],window_name='RANSAC平面分割后点云')

   return outlier_cloud

def icp_registration(source_pcd,target_pcd,init1,max_correspondence_distance=0.1):
   '''
      max_correspondence_distance   距离阈值
   '''
   print("->正在进行icp 配准...")
   icp = o3d.pipelines.registration.registration_icp(
         source=source_pcd,
         target=target_pcd,
         max_correspondence_distance=max_correspondence_distance,    # 距离阈值
         init=init1,
         estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
      )

   print("配准结果矩阵:")
   print(icp.transformation)
   print("->正在可视化icp 配准结果点云")
   return icp.transformation


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


def get_feature(pcd_down, voxel_size = 0.007):
   '''
   提取几何特征
   '''   
   radius_normal = voxel_size * 2
   print(":: Estimate normal with search radius %.3f." % radius_normal)
   pcd_down.estimate_normals(
      o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

   radius_feature = voxel_size * 5
   print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
   pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
      pcd_down,
      o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
   return pcd_fpfh

def point_to_plane_icp(source, target, threshold, trans_init):
   print("Apply point-to-plane ICP")
   reg_p2l = o3d.pipelines.registration.registration_icp(
      source, target, threshold, trans_init,
      o3d.pipelines.registration.TransformationEstimationPointToPlane())
   print(reg_p2l)
   print("Transformation is:")
   print(reg_p2l.transformation, "\n")
   return reg_p2l

def execute_global_registration(source_down, target_down, source_fpfh,
                               target_fpfh, voxel_size):
   distance_threshold = voxel_size * 1.5
   print(":: RANSAC registration on downsampled point clouds.")
   print("   Since the downsampling voxel size is %.3f," % voxel_size)
   print("   we use a liberal distance threshold %.3f." % distance_threshold)
   result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
       source_down, target_down, source_fpfh, target_fpfh,True,distance_threshold,
       o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4, 
       [
           o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
           o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
               distance_threshold)
       ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 300))
   return result