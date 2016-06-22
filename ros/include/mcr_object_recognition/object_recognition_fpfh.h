/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Anand Ajmera
 *
 */
#ifndef MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_FPFH_H
#define MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_FPFH_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <mcr_perception_msgs/PointCloud2List.h>
#include <sensor_msgs/PointCloud2.h> 
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/esf.h>

namespace mcr_object_recognition
{

class ObjectRecognitionFPFH : public nodelet::Nodelet
{
	
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

    public:
        ObjectRecognitionFPFH(){}


    protected:
        ros::Publisher pub_feature_list_;

        ros::Subscriber sub_cluster_list_;

        ros::Publisher pub_feature_list_test_;

        ros::Subscriber sub_cluster_list_test_;

        ros::NodeHandle nh_;

        void onInit ();

        void clusterListCallback (const mcr_perception_msgs::PointCloud2List::ConstPtr &clouds);
	
		void calculateFeatures(const PointCloud::ConstPtr &cloud, pcl::PointCloud<pcl::ESFSignature640>::Ptr &descriptors);

        void calculate_keypoints(const PointCloud::ConstPtr &cloud, PointCloud::Ptr &keypoints);

        double computeCloudResolution(const PointCloud::ConstPtr& cloud);

        void clusterListCallbackTest (const sensor_msgs::PointCloud2::ConstPtr &cloudstest);

};
PLUGINLIB_DECLARE_CLASS(mcr_object_recognition, ObjectRecognitionFPFH, mcr_object_recognition::ObjectRecognitionFPFH, nodelet::Nodelet);
}

#endif  // MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_FPFH_H
