#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

import numpy as np
# from scipy.spatial.transform import Rotation as R   # not support python2

from tf.transformations import quaternion_matrix
import tf

# from velo2cam_calibration.msg import CentersGT
from geometry_msgs.msg import Point

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# 
import roslib
# roslib.load_manifest('learning_tf')
# import rospy
# import tf

# from sensor_msgs.msg import PointCloud2

class GazeboLinkPose:
    link_name = ''
    link_pose = Pose()
    def __init__(self, link_name):
        self.link_name = link_name
        print(self.link_name)
        self.link_name_rectified = link_name.replace("::", "_")
        print(self.link_name_rectified )


        if not self.link_name:
            raise ValueError("'link_name' is an empty string")
        
        print("before subscriber!")
        self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        # self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name_rectified, Pose, queue_size = 10)
        # self.point_pub = rospy.Publisher("/gazebo/four_centers_gt", CentersGT, queue_size = 10)
        # self.pointcloud_lidar_pub = rospy.Publisher("/gazebo/lidar/four_centers_gt_cloud", PointCloud2, queue_size = 10)
        # self.pointcloud_cam_pub = rospy.Publisher("/gazebo/cam/four_centers_gt_cloud", PointCloud2, queue_size = 10)



    def callback(self, data):
        # rospy.loginfo("Entered callback!")

        try:
            # "~link_name" value="asphalt_plane::link"/>
            print(self.link_name)
            ind = data.name.index(self.link_name)
            print(ind)
            self.link_pose = data.pose[ind]
            print(self.link_pose)
            
            # link_name_tmp = "calibration_wood_pattern::body"
            # link_name_rectified_tmp = link_name_tmp.replace("::", "_")
            # print(link_name_rectified_tmp)
            # ind_tmp = data.name.index(link_name_tmp)
            # print(ind_tmp)
            # link_pose_calibration_wood_pattern = data.pose[ind_tmp]
            # print(link_pose_calibration_wood_pattern)

            # link_name_tmp = "bumblebee_xb3::link"
            # link_name_rectified_tmp = link_name_tmp.replace("::", "_")
            # print(link_name_rectified_tmp)
            # ind_tmp = data.name.index(link_name_tmp)
            # print(ind_tmp)
            # link_pose_bumblebee_xb3 = data.pose[ind_tmp]
            # print(link_pose_bumblebee_xb3)

            # link_name_tmp = "velodyne-VLP16_0::base_footprint"
            # link_name_rectified_tmp = link_name_tmp.replace("::", "_")
            # print(link_name_rectified_tmp)
            # ind_tmp = data.name.index(link_name_tmp)
            # print(ind_tmp)
            # link_pose_velodyne_VLP16_0 = data.pose[ind_tmp]
            # print(link_pose_velodyne_VLP16_0)

            link_name_tmp = "RealSense_1::base_footprint"
            link_name_rectified_tmp = link_name_tmp.replace("::", "_")
            print(link_name_rectified_tmp)
            ind_tmp = data.name.index(link_name_tmp)
            print(ind_tmp)
            link_pose_sensor_1 = data.pose[ind_tmp]
            print(link_pose_sensor_1)

            link_name_tmp = "RealSense_2::base_footprint"
            link_name_rectified_tmp = link_name_tmp.replace("::", "_")
            print(link_name_rectified_tmp)
            ind_tmp = data.name.index(link_name_tmp)
            print(ind_tmp)
            link_pose_sensor_2 = data.pose[ind_tmp]
            print(link_pose_sensor_2)

            link_name_tmp = "RealSense_3::base_footprint"
            link_name_rectified_tmp = link_name_tmp.replace("::", "_")
            print(link_name_rectified_tmp)
            ind_tmp = data.name.index(link_name_tmp)
            print(ind_tmp)
            link_pose_sensor_3 = data.pose[ind_tmp]
            print(link_pose_sensor_3)


            rospy.loginfo("[Done] get_gazebo_link_pose!")
        except ValueError:
            print("Error!")
            pass
        
        # convert link_pose to transform matrix 
        # Tr_wood2world = self.link_pose2transform_matrix(link_pose_calibration_wood_pattern)
        # print("Tr_wood2world = ", Tr_wood2world)

        # Tr_cam2world = self.link_pose2transform_matrix(link_pose_bumblebee_xb3)
        # print("Tr_cam2world = ", Tr_cam2world)

        # Tr_lidar2world = self.link_pose2transform_matrix(link_pose_velodyne_VLP16_0)
        # print("Tr_lidar2world = ", Tr_lidar2world)       
        # # print(self.mat_from_transform(link_pose_calibration_wood_pattern) )

        Tr_sensor1_to_world = self.link_pose2transform_matrix(link_pose_sensor_1)
        print("Tr_sensor1_to_world = ", Tr_sensor1_to_world)

        Tr_sensor2_to_world = self.link_pose2transform_matrix(link_pose_sensor_2)
        print("Tr_sensor2_to_world = ", Tr_sensor2_to_world)

        Tr_sensor3_to_world = self.link_pose2transform_matrix(link_pose_sensor_3)
        print("Tr_sensor3_to_world = ", Tr_sensor3_to_world)

        # transform matrix from lidar to stereo (medium frame)
        # Tr_lidar2cam = np.dot(np.linalg.inv(Tr_cam2world),  Tr_lidar2world)
        # print("Tr_lidar2cam = ", Tr_lidar2cam)       

        # # calculate transform btn wood2cam and wood2lidar
        # Tr_wood2cam = np.dot(np.linalg.inv(Tr_cam2world),  Tr_wood2world)
        # Tr_cam2cam = np.array([[0,-1,0,0], [0,0,-1,0], [1,0,0,0], [0,0,0,1]])
        # # Tr_wood2cam = np.dot(Tr_cam2cam, Tr_wood2cam)   # if use camera frame (z forward), uncomment this
        # print("Tr_wood2cam = ", Tr_wood2cam)       

        # Tr_wood2lidar = np.dot(np.linalg.inv(Tr_lidar2world),  Tr_wood2world)
        # print("Tr_wood2lidar = ", Tr_wood2lidar)       

        # # four circle centers in calibration board (wood) frame
        # p1_wood = np.array([[0], [-0.15], [0.55]])
        # p1_wood_homo = np.vstack(( p1_wood, np.array([[1]])))

        # p2_wood = np.array([[0], [0.15], [0.55]])
        # p2_wood_homo = np.vstack(( p2_wood, np.array([[1]])))

        # p3_wood = np.array([[0], [-0.15], [0.25]])
        # p3_wood_homo = np.vstack(( p3_wood, np.array([[1]])))

        # p4_wood = np.array([[0], [0.15], [0.25]])
        # p4_wood_homo = np.vstack(( p4_wood, np.array([[1]])))

        # # print("p1_wood = ", p1_wood)
        # # print("p1_wood_homo = ", p1_wood_homo)

        # # print("p2_wood_homo = ", p2_wood_homo)

        # # print("p3_wood_homo = ", p3_wood_homo)


        # # calculate four circle centers in lidar frame
        # # p1_lidar_homo = np.multiply(Tr_wood2lidar, p1_wood_homo)
        # # print("p1_lidar_homo = ", p1_lidar_homo)

        # p1_lidar_homo = np.dot(Tr_wood2lidar, p1_wood_homo)
        # print("p1_lidar_homo = ", p1_lidar_homo)

        # p2_lidar_homo = np.dot(Tr_wood2lidar, p2_wood_homo)
        # print("p2_lidar_homo = ", p2_lidar_homo)

        # p3_lidar_homo = np.dot(Tr_wood2lidar, p3_wood_homo)
        # print("p3_lidar_homo = ", p3_lidar_homo)

        # p4_lidar_homo = np.dot(Tr_wood2lidar, p4_wood_homo)
        # print("p4_lidar_homo = ", p4_lidar_homo)


        # # calculate four circle centers in camera frame
        # p1_cam_homo = np.dot(Tr_wood2cam, p1_wood_homo)
        # print("p1_cam_homo = ", p1_cam_homo)

        # p2_cam_homo = np.dot(Tr_wood2cam, p2_wood_homo)
        # print("p2_cam_homo = ", p2_cam_homo)

        # p3_cam_homo = np.dot(Tr_wood2cam, p3_wood_homo)
        # print("p3_cam_homo = ", p3_cam_homo)

        # p4_cam_homo = np.dot(Tr_wood2cam, p4_wood_homo)
        # print("p4_cam_homo = ", p4_cam_homo)

        # centers_gt = CentersGT()
        
        # p1_lidar_center_gt = Point()
        # p1_lidar_center_gt.x = p1_lidar_homo[0,0]
        # p1_lidar_center_gt.y = p1_lidar_homo[1,0]
        # p1_lidar_center_gt.z = p1_lidar_homo[2,0]
        # centers_gt.lidar_centers_gt.append(p1_lidar_center_gt)

        # p2_lidar_center_gt = Point()
        # p2_lidar_center_gt.x = p2_lidar_homo[0,0]
        # p2_lidar_center_gt.y = p2_lidar_homo[1,0]
        # p2_lidar_center_gt.z = p2_lidar_homo[2,0]
        # centers_gt.lidar_centers_gt.append(p2_lidar_center_gt)

        # p3_lidar_center_gt = Point()
        # p3_lidar_center_gt.x = p3_lidar_homo[0,0]
        # p3_lidar_center_gt.y = p3_lidar_homo[1,0]
        # p3_lidar_center_gt.z = p3_lidar_homo[2,0]
        # centers_gt.lidar_centers_gt.append(p3_lidar_center_gt)

        # p4_lidar_center_gt = Point()
        # p4_lidar_center_gt.x = p4_lidar_homo[0,0]
        # p4_lidar_center_gt.y = p4_lidar_homo[1,0]
        # p4_lidar_center_gt.z = p4_lidar_homo[2,0]
        # centers_gt.lidar_centers_gt.append(p4_lidar_center_gt)


        # p1_cam_center_gt = Point()
        # p1_cam_center_gt.x = p1_cam_homo[0,0]
        # p1_cam_center_gt.y = p1_cam_homo[1,0]
        # p1_cam_center_gt.z = p1_cam_homo[2,0]
        # centers_gt.cam_centers_gt.append(p1_cam_center_gt)

        # p2_cam_center_gt = Point()
        # p2_cam_center_gt.x = p2_cam_homo[0,0]
        # p2_cam_center_gt.y = p2_cam_homo[1,0]
        # p2_cam_center_gt.z = p2_cam_homo[2,0]
        # centers_gt.cam_centers_gt.append(p2_cam_center_gt)

        # p3_cam_center_gt = Point()
        # p3_cam_center_gt.x = p3_cam_homo[0,0]
        # p3_cam_center_gt.y = p3_cam_homo[1,0]
        # p3_cam_center_gt.z = p3_cam_homo[2,0]
        # centers_gt.cam_centers_gt.append(p3_cam_center_gt)

        # p4_cam_center_gt = Point()
        # p4_cam_center_gt.x = p4_cam_homo[0,0]
        # p4_cam_center_gt.y = p4_cam_homo[1,0]
        # p4_cam_center_gt.z = p4_cam_homo[2,0]
        # centers_gt.cam_centers_gt.append(p4_cam_center_gt)


        # self.point_pub.publish(centers_gt)


        # # publish as point cloud 2
        # # lidar
        # points_lidar = []
        # pt1_lidar = [p1_lidar_homo[0,0], p1_lidar_homo[1,0], p1_lidar_homo[2,0]]
        # points_lidar.append(pt1_lidar)
        # pt2_lidar = [p2_lidar_homo[0,0], p2_lidar_homo[1,0], p2_lidar_homo[2,0]]
        # points_lidar.append(pt2_lidar)
        # pt3_lidar = [p3_lidar_homo[0,0], p3_lidar_homo[1,0], p3_lidar_homo[2,0]]
        # points_lidar.append(pt3_lidar)
        # pt4_lidar = [p4_lidar_homo[0,0], p4_lidar_homo[1,0], p4_lidar_homo[2,0]]
        # points_lidar.append(pt4_lidar)

        # print (points_lidar)

        # fields = [PointField('x', 0, PointField.FLOAT32, 1),
        #   PointField('y', 4, PointField.FLOAT32, 1),
        #   PointField('z', 8, PointField.FLOAT32, 1),
        #   # PointField('rgb', 12, PointField.UINT32, 1),
        #   ]
        
        # header = Header()
        # header.frame_id = "velodyne"
        # pc2 = point_cloud2.create_cloud(header, fields, points_lidar)
        # pc2.header.stamp = rospy.Time.now()
        # self.pointcloud_lidar_pub.publish(pc2)


        # # cam
        # points_cam = []
        # pt1_cam = [p1_cam_homo[0,0], p1_cam_homo[1,0], p1_cam_homo[2,0]]
        # points_cam.append(pt1_cam)
        # pt2_cam = [p2_cam_homo[0,0], p2_cam_homo[1,0], p2_cam_homo[2,0]]
        # points_cam.append(pt2_cam)
        # pt3_cam = [p3_cam_homo[0,0], p3_cam_homo[1,0], p3_cam_homo[2,0]]
        # points_cam.append(pt3_cam)
        # pt4_cam = [p4_cam_homo[0,0], p4_cam_homo[1,0], p4_cam_homo[2,0]]
        # points_cam.append(pt4_cam)

        # print (points_cam)

        # fields = [PointField('x', 0, PointField.FLOAT32, 1),
        #   PointField('y', 4, PointField.FLOAT32, 1),
        #   PointField('z', 8, PointField.FLOAT32, 1),
        #   # PointField('rgb', 12, PointField.UINT32, 1),
        #   ]
        
        # header = Header()
        # header.frame_id = "stereo_camera"
        # pc2 = point_cloud2.create_cloud(header, fields, points_cam)
        # pc2.header.stamp = rospy.Time.now()
        # self.pointcloud_cam_pub.publish(pc2)


        # # Send transform from lidar to cam (stereo)

        # # this rotation matrix must be 4x4
        # # m=tf.transformations.random_rotation_matrix(rand=None)
        # # print(m)
        # # rot_matrix = np.eye(4, dtype=float)
        # # print(rot_matrix)
        # # rot_matrix[0:3, 0:3] = Tr_lidar2cam[0:3,0:3]
        # # print(rot_matrix)
        # # q = tf.transformations.quaternion_from_matrix(rot_matrix)
        # # print( q )
        # # print( tf.transformations.euler_from_quaternion(q) )

        # # print( Tr_lidar2cam )
        # q = tf.transformations.quaternion_from_matrix(Tr_lidar2cam)
        # # print( q )
        # # print(tf.transformations.euler_from_quaternion(q))
        # br = tf.TransformBroadcaster()
        # # br.sendTransform((Tr_lidar2cam[0,3], Tr_lidar2cam[1,3], Tr_lidar2cam[2,3]),
        # #                 # tf.transformations.quaternion_from_euler(0, 0, msg.theta),
        # #                 # tf.transformations.quaternion_from_matrix(Tr_lidar2cam),   # this rotation matrix must be 4x4
        # #                 q,
        # #                 rospy.Time.now(),
        # #                 "velodyne_gt",
        # #                 "stereo_camera_gt")
        # br.sendTransform((Tr_lidar2cam[0,3], Tr_lidar2cam[1,3], Tr_lidar2cam[2,3]),
        #                 # tf.transformations.quaternion_from_euler(0, 0, msg.theta),
        #                 # tf.transformations.quaternion_from_matrix(Tr_lidar2cam),   # this rotation matrix must be 4x4
        #                 q,
        #                 rospy.Time.now(),
        #                 "velodyne_gt",
        #                 "stereo_gt")
        
        q1 = tf.transformations.quaternion_from_matrix(Tr_sensor1_to_world)
        br1 = tf.TransformBroadcaster()
        br1.sendTransform((Tr_sensor1_to_world[0,3], Tr_sensor1_to_world[1,3], Tr_sensor1_to_world[2,3]), q1, rospy.Time.now(),"sensor1_gt", "world_gt")

        q2 = tf.transformations.quaternion_from_matrix(Tr_sensor2_to_world)
        br2 = tf.TransformBroadcaster()
        br2.sendTransform((Tr_sensor2_to_world[0,3], Tr_sensor2_to_world[1,3], Tr_sensor2_to_world[2,3]), q2, rospy.Time.now(),"sensor2_gt", "world_gt")
        
        q3 = tf.transformations.quaternion_from_matrix(Tr_sensor3_to_world)
        br3 = tf.TransformBroadcaster()
        br3.sendTransform((Tr_sensor3_to_world[0,3], Tr_sensor3_to_world[1,3], Tr_sensor3_to_world[2,3]), q3, rospy.Time.now(),"sensor3_gt", "world_gt")
        
        # for track in self._tracker.tracks:
        #     if track.is_confirmed() and track.time_since_update >1 :
        #         continue 
        #     bbox = track.to_tlbr()
            
        #     detected = Detected()
        #     detected.object_class = 'Person'
        #     detected.num = np.uint32(track.track_id)
        #     detected.p = 1.0

        #     def range_check(x,min,max):
        #         if x < min: x = min
        #         if x > max: x = max
        #         return x
            
        #     detected.x = np.uint16(range_check(int(bbox[0]),0,cv_image.shape[1]))
        #     detected.y = np.uint16(range_check(int(bbox[1]),0,cv_image.shape[0]))
        #     detected.width = np.uint16(range_check(int(bbox[2]) - int(bbox[0]), 0, cv_image.shape[1]))
        #     detected.height = np.uint16(range_check(int(bbox[3]) - int(bbox[1]), 0, cv_image.shape[0]))

        #     cv2.rectangle(cv_image, (int(detected.x), int(detected.y)), (int(detected.x+detected.width), int(detected.y+detected.height)),(255,0,0), 2)
        #     cv2.putText(cv_image, 'Person:'+str(track.track_id),(int(detected.x), int(detected.y)),0, 5e-3 * 100, (0,255,0),2)

        #     detected_array.size = detected_array.size+1
        #     detected_array.data.append(detected)

        #     # for det in detections:
        #     #     bbox = det.to_tlbr()
        #     #     cv2.rectangle(cv_image,(int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,0), 2)
            

        # # cv2.imshow('', cv_image)
        # detected_full.header.stamp = image_msg.header.stamp
        # detected_full.detections = detected_array
        # if self._detected_pub.get_num_connections() > 0:
        #     self._detected_pub.publish(detected_full)




    def link_pose2transform_matrix(self, link_pose):
        # rot_matrix = quaternion_matrix([link_pose.orientation.w, link_pose.orientation.x, link_pose.orientation.y, link_pose.orientation.z])  # [quat.w, quat.x, quat.y, quat.z]
        # print(rot_matrix)

        rotation = (link_pose.orientation.x, link_pose.orientation.y, link_pose.orientation.z, link_pose.orientation.w)
        # print(rotation)

        translation = (link_pose.position.x, link_pose.position.y, link_pose.position.z)
        # print(translation)

        return tf.TransformerROS().fromTranslationRotation(translation, rotation)

    
    def mat_from_transform(self, transform):
        # vtr = (transform.translation.x, transform.translation.y,
        #                 transform.translation.z)
        vtr = (transform.position.x, transform.position.y,
                        transform.position.z)

        # vrot = (transform.rotation.x, transform.rotation.y, transform.rotation.z,
        #                 transform.rotation.w)
        vrot = (transform.orientation.x, transform.orientation.y, transform.orientation.z,
                        transform.orientation.w)

        return tf.TransformerROS().fromTranslationRotation(vtr, vrot)

if __name__ == '__main__':
    try:
        rospy.init_node('get_gazebo_link_pose_ITS', anonymous=True)
        gp = GazeboLinkPose(rospy.get_param('~link_name'))
        publish_rate = rospy.get_param('publish_rate', 10)
        rospy.spin()

        # rate = rospy.Rate(publish_rate)
        # while not rospy.is_shutdown():
        #     # gp.pose_pub.publish(gp.link_pose)
        #     rate.sleep()

    except rospy.ROSInterruptException:
        pass

# name: ['asphalt_plane::link', 'grey_wall::link', 'calibration_wood_pattern::body', 'bumblebee_xb3::link',
#   'velodyne-VLP16_0::base_footprint']
